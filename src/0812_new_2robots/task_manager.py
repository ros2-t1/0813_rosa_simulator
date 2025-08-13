#!/usr/bin/env python3
# task_manager.py

import rclpy
from rclpy.node import Node
from enum import Enum, auto
import yaml
from functools import partial
import pathlib
import threading
import time

from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rosa_interfaces.srv import UpdateLocationStatus, GetLocationStatus
from config import ROBOT_NAMES, ROBOT_CHARGE_STATIONS, BATTERY_THRESHOLD, ITEM_ARUCO_MAP, LOCATIONS
from simulation_test import SimulationTest
from delivery_interface import DeliveryInterface

class RobotState(Enum):
    CHARGING = auto(); IDLE = auto(); RETURNING = auto(); WAITING = auto()
    AWAITING_PICKUP_RESERVATION = auto(); MOVING_TO_PICKUP_WAIT = auto(); WAITING_AT_PICKUP_QUEUE = auto()
    MOVING_TO_PICKUP = auto(); PICKING_UP = auto(); AWAITING_DEST_RESERVATION = auto()
    MOVING_TO_DEST = auto()
    AWAITING_CONFIRMATION = auto()
    FAILURE = auto() # 주행 실패 상태
    EMERGENCY_STOP = auto(); OFF_DUTY = auto()

class RobotInfo:
    def __init__(self, name: str):
        self.name = name
        self.state: RobotState = RobotState.IDLE
        self.current_pose = None
        self.battery_level: float = 100.0
        self.current_task = None
        self.current_location = None
        self.reservation_failure_logged = False
        self.suspended_task = None
        self.suspended_state = None
        # 타임아웃 처리용 필드
        self.last_activity_time = time.time()
        self.reservation_start_time = None
        self.arm_response_start = None  # 로봇팔 응답 대기 시작 시간
        self.pickup_gui_active = False  # 픽업 확인 GUI 활성 상태
        self.pickup_gui_root = None     # GUI 창 참조

class Task:
    def __init__(self, robot_name, destination, item=None, order_id=None, user_id=None):
        self.robot_name = robot_name
        self.item = item
        self.destination = destination
        self.pickup_location = "픽업대" if item else None
        self.order_id = order_id or f"{robot_name}_{int(time.time())}"  # 고유 업무 ID
        self.user_id = user_id or "user123"  # 주문한 사용자 ID
        self.order_time = time.time()  # 주문 시간
        self.pickup_time = None  # 픽업 시간
        self.delivery_time = None  # 배달 완료 시간
        self.confirmation_time = None  # 수령확인 시간

class TaskManager(Node):
    def __init__(self, simulation_mode=False):
        super().__init__('task_manager')
        self.simulation_mode = simulation_mode
        self.waypoints = self.load_waypoints()
        self.robots: dict[str, RobotInfo] = {name: RobotInfo(name) for name in ROBOT_NAMES}
        
        if self.simulation_mode:
            self.sim_test = SimulationTest(self)
        
        self.status_log_pub = self.create_publisher(String, '/rosa/status_log', 10)
        self.task_status_pub = self.create_publisher(String, '/rosa/task_status_update', 10)  # 업무 상태 전용
        self.task_queue_pub = self.create_publisher(String, '/rosa/task_queue_update', 10)  # TaskQueue 상태 전용
        self.gui_robot_position_pub = self.create_publisher(String, '/rosa/robot_position_update', 10)  # 장소 기반 위치
        self.gui_realtime_pose_pub = self.create_publisher(String, '/rosa/robot_realtime_pose', 10)  # 🗺️ 실시간 좌표
        
        # GUI/DB 연동 인터페이스 초기화 (TaskManager의 Node 기능을 공유)
        self.delivery_interface = DeliveryInterface(node=self, task_manager_callback=self.handle_gui_confirmation_response)
        
        # ✅ 통합된 LocationManager 기능 초기화
        self.setup_location_manager()
        
        # ✅ TaskQueue 시스템 초기화
        from task_queue import TaskQueue
        self.task_queue = TaskQueue(self)
        
        # 초기 상태 설정
        for robot_name, robot in self.robots.items():
            robot.current_location = ROBOT_CHARGE_STATIONS.get(robot_name)
            # if 문을 제거하여 모든 로봇의 초기 상태를 CHARGING으로 설정
            robot.state = RobotState.CHARGING
        
        if self.simulation_mode:
            self.get_logger().info("✅ TaskManager가 [시뮬레이션 모드]로 시작되었습니다.")
        else:
            self.path_pubs = {name: self.create_publisher(Path, f'/{name}/waypoint_path_goal', 10) for name in ROBOT_NAMES}
            self.loc_update_cli = self.create_client(UpdateLocationStatus, 'update_location_status')
            self.arm_cmd_pub = self.create_publisher(Int32, 'robot_arm/user_cmd', 10)
            self.create_subscription(String, 'robot_arm/status', self.arm_status_callback, 10)
            self.setup_robot_subscriptions()
            self.get_logger().info("✅ Task Manager (실제 로봇 모드) 준비 완료.")

        self.task_processor_timer = self.create_timer(1.0, self.process_tasks)
        
        # 🔋 초기 GUI 업데이트 (시작하자마자 배터리 정보 전송)
        self._initial_gui_timer = self.create_timer(2.0, self.initial_gui_update)

    def initial_gui_update(self):
        """시작 2초 후 초기 GUI 업데이트"""
        self.get_logger().info("🔋 초기 GUI 배터리 정보 전송")
        for robot_name in self.robots.keys():
            self.publish_robot_position_with_battery(robot_name)
        # 한 번만 실행하고 타이머 해제
        try:
            self._initial_gui_timer.cancel()
        except Exception:
            pass  # 이미 해제되었을 수 있음

    def cancel_current_navigation(self, robot_name: str):
        """해당 로봇의 현재 내비게이션 목표를 취소합니다."""
        self.publish_status_log(robot_name, "NAVIGATION_CANCELED", "새 업무 할당을 위해 현재 이동 취소")
        
        if self.simulation_mode:
            # 시뮬레이션에서는 타이머를 취소하는 방식으로 구현
            self.sim_test.cancel_move(robot_name)
            return
            
        # 실제 로봇의 경로를 취소하기 위해 빈 경로(Path)를 발행
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pubs[robot_name].publish(path_msg)

    # === 통합된 LocationManager 기능 ===
    def setup_location_manager(self):
        """LocationManager 기능 초기화"""
        # 장소 상태 저장 (충전소는 각 로봇 전용이므로 제외)
        self.location_states = {
            '픽업대': 'available',
            '픽업대기장소': 'available', 
            '왼쪽방': 'available',
            '오른쪽방': 'available',
            '면회실': 'available',
            '출입구': 'available'
        }
        self.location_lock = threading.Lock()
        
        # LocationManager 서비스들
        self.create_service(GetLocationStatus, 'get_location_status', self.get_location_status_callback)
        self.create_service(UpdateLocationStatus, 'reserve_location', self.reserve_location_callback)
        self.create_service(UpdateLocationStatus, 'update_location_status', self.update_location_status_callback)
        
        # GUI용 장소 상태 브로드캐스트 토픽
        self.gui_location_status_pub = self.create_publisher(String, '/rosa/location_status_update', 10)
        
        self.get_logger().info("✅ 통합 LocationManager 기능 준비 완료")

    def get_location_status_callback(self, request, response):
        """장소 상태 조회"""
        with self.location_lock:
            response.status = self.location_states.get(request.location_name, 'unknown')
        return response

    def reserve_location_callback(self, request, response):
        """장소 예약"""
        with self.location_lock:
            current_status = self.location_states.get(request.location_name)
            if current_status == 'available':
                self.location_states[request.location_name] = 'reserved'
                response.success = True
                self.publish_status_log(request.location_name, "RESERVED", f"'{request.location_name}' 예약 완료")
                self.broadcast_location_status_change(request.location_name, 'reserved')
            else:
                response.success = False
                self.publish_status_log(request.location_name, "RESERVE_FAILED", f"'{request.location_name}' 예약 실패 (현재 상태: {current_status})")
        return response

    def update_location_status_callback(self, request, response):
        """장소 상태 업데이트"""
        with self.location_lock:
            if request.location_name in self.location_states:
                old_status = self.location_states[request.location_name]
                self.location_states[request.location_name] = request.status
                response.success = True
                self.publish_status_log(request.location_name, "STATUS_CHANGED", f"'{request.location_name}' 상태 변경: {old_status} → {request.status}")
                self.broadcast_location_status_change(request.location_name, request.status)
            else:
                response.success = False
        return response

    def broadcast_location_status_change(self, location_name: str, new_status: str):
        """GUI용 장소 상태 변경 브로드캐스트"""
        try:
            status_msg = String()
            timestamp = int(time.time())
            status_msg.data = f"{location_name}|{new_status}|{timestamp}"
            self.gui_location_status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"❌ GUI 장소 상태 브로드캐스트 실패: {e}")

    # === 핵심 시스템 함수들 ===
    def publish_status_log(self, entity_name: str, status: str, reason: str):
        log_msg = String()
        log_msg.data = f"{entity_name}|{status}|{reason}"
        self.status_log_pub.publish(log_msg)
    
    def publish_robot_position_with_battery(self, robot_name: str):
        """로봇 위치와 배터리 정보를 GUI로 전송"""
        robot = self.robots.get(robot_name)
        if not robot:
            return
        
        location = robot.current_location if robot.current_location else "위치불명"
        battery = f"{robot.battery_level:.1f}"
        
        # 형식: "로봇이름|위치|배터리레벨"
        position_msg = String()
        position_msg.data = f"{robot.name}|{location}|{battery}"
        self.gui_robot_position_pub.publish(position_msg)
        
        # 디버그용 로그 (5초마다 한 번씩만 출력)
        if not hasattr(self, '_last_battery_log_time'):
            self._last_battery_log_time = {}
        current_time = time.time()
        if robot_name not in self._last_battery_log_time or current_time - self._last_battery_log_time[robot_name] > 10.0:
            self._last_battery_log_time[robot_name] = current_time
            self.get_logger().info(f"🔋 [{robot_name}] GUI 전송: 위치={location}, 배터리={battery}%")
    
    def show_pickup_confirmation_gui(self, robot: RobotInfo):
        """🚫 GUI 크래시 방지를 위한 안전한 픽업 확인 - 자동 처리로 변경"""
        if not robot.current_task or robot.pickup_gui_active:
            self.get_logger().info(f"🚫 [{robot.name}] GUI 이미 활성화되어 있거나 업무 없음 - 중복 생성 방지")
            return
            
        # 크래시 방지를 위해 GUI 대신 자동 픽업 처리
        self.get_logger().info(f"🔄 [{robot.name}] GUI 대신 자동 픽업 완룤 처리 (크래시 방지)")
        
        # 2초 후 자동으로 픽업 완료 처리
        def auto_complete_pickup():
            time.sleep(2.0)  # 2초 대기
            if robot.current_task and robot.name in self.robots:  # 로봇이 아직 존재하고 업무가 있으면
                self.get_logger().info(f"✅ [{robot.name}] 자동 픽업 완료 실행")
                self.handle_pickup_complete(robot)
        
        # 별도 스레드에서 자동 완료 실행 (더 안전함)
        import threading
        try:
            auto_thread = threading.Thread(target=auto_complete_pickup, name=f"AutoPickup_{robot.name}")
            auto_thread.daemon = True
            auto_thread.start()
            robot.pickup_gui_active = True  # GUI 상태 대신 플래그 설정
            self.get_logger().info(f"✅ [{robot.name}] 자동 픽업 스레드 시작 성공")
        except Exception as e:
            self.get_logger().error(f"❌ [{robot.name}] 자동 픽업 스레드 실패: {e}")
            # 스레드 실패 시 즉시 완료 처리
            self.handle_pickup_complete(robot)
        
        # 기존 GUI 코드를 주석 처리하여 비활성화
        
        import tkinter as tk
        from tkinter import messagebox
        
        robot.pickup_gui_active = True
        self.get_logger().info(f"🖱️ [{robot.name}] 픽업 확인 GUI 생성 시작")
        
        # GUI 창 생성 (백그라운드에서 실행)
        def show_confirmation():
            try:
                robot.pickup_gui_root = tk.Tk()
                robot.pickup_gui_root.withdraw()  # 메인 창 숨기기
                
                item_name = robot.current_task.item if robot.current_task else "물품"
                robot_name = robot.name
                
                result = messagebox.askyesno(
                    "픽업 확인", 
                    f"🤖 {robot_name}\n\n"
                    f"'{item_name}' 적재 완료되었나요?\n"
                    f"(로봇팔이 자동으로 완료하면 이 창이 사라집니다)",
                    icon='question'
                )
                
                # GUI가 닫힐 때 상태 리셋
                robot.pickup_gui_active = False
                robot.pickup_gui_root = None
                
                if result:  # Yes 선택
                    self.get_logger().info(f"✅ [{robot_name}] 사용자가 수동으로 픽업 완료 확인")
                    self.handle_pickup_complete(robot)
                else:  # No 선택
                    self.get_logger().info(f"❌ [{robot_name}] 사용자가 픽업 실패 확인")
                    self.publish_status_log(robot_name, "PICKUP_FAILED", "사용자가 픽업 실패 확인")
                    
            except Exception as e:
                self.get_logger().error(f"❌ [{robot.name}] GUI 생성 실패: {e}")
                robot.pickup_gui_active = False
                robot.pickup_gui_root = None
                # GUI 실패 시 콘솔로 대체
                self.get_logger().info(f"🖱️ [{robot.name}] GUI 실패, 자동으로 픽업 완료 처리")
                self.handle_pickup_complete(robot)
        
        # 별도 스레드에서 GUI 실행 (메인 ROS 루프 차단 방지)
        import threading
        try:
            gui_thread = threading.Thread(target=show_confirmation, name=f"PickupGUI_{robot.name}")
            gui_thread.daemon = True
            gui_thread.start()
        except Exception as e:
            self.get_logger().error(f"❌ [{robot.name}] GUI 스레드 생성 실패: {e}")
            # GUI 실패 시 자동으로 픽업 완료 처리
            robot.pickup_gui_active = False
            self.handle_pickup_complete(robot)
        
        self.publish_status_log(robot.name, "MANUAL_CHECK", f"🖱️ 픽업 확인창 표시 (로봇팔 또는 수동 확인 대기)")
    
    def handle_pickup_complete(self, robot: RobotInfo):
        """픽업 완료 처리 (수동 또는 로봇팔 자동)"""
        if robot.current_task:
            robot.current_task.pickup_time = time.time()
        
        # GUI가 활성화되어 있으면 강제로 닫기
        if robot.pickup_gui_active and robot.pickup_gui_root:
            try:
                # 더 안전한 GUI 종료
                robot.pickup_gui_root.after(0, robot.pickup_gui_root.quit)
                robot.pickup_gui_root.after(100, robot.pickup_gui_root.destroy)
            except Exception as e:
                self.get_logger().warn(f"GUI 종료 중 오류: {e}")
            finally:
                robot.pickup_gui_active = False
                robot.pickup_gui_root = None
        
        # 로봇팔 상태 로그 발행 - 대기중으로 복귀
        self.publish_status_log("robot_arm", "IDLE", f"{robot.current_task.item} 픽업 완료 - 대기중")
        
        self.change_robot_state(robot, RobotState.AWAITING_DEST_RESERVATION, "픽업 완료, 목적지 예약 대기")
    
    def publish_task_status_update(self, task_id: str, status: str, robot_name: str, destination: str, item: str = None):
        """GUI용 업무 상태 업데이트 발행"""
        task_msg = String()
        # 메시지 형식: "task_id|status|robot_name|destination|item"
        item_str = item if item else ""
        task_msg.data = f"{task_id}|{status}|{robot_name}|{destination}|{item_str}"
        self.task_status_pub.publish(task_msg)
    
    def publish_task_queue_status(self):
        """TaskQueue 전체 상태를 GUI로 발행"""
        try:
            import json
            queue_status = self.task_queue.get_queue_status()
            
            queue_msg = String()
            queue_msg.data = json.dumps(queue_status)
            self.task_queue_pub.publish(queue_msg)
        except Exception as e:
            self.get_logger().error(f"❌ TaskQueue 상태 발행 오류: {e}")

    def change_robot_state(self, robot: RobotInfo, new_state: RobotState, reason: str = ""):
        old_state = robot.state
        robot.state = new_state
        robot.last_activity_time = time.time() # 상태 변경 시 활동 시간 갱신
        status_description = f"{old_state.name} → {new_state.name}"
        self.publish_status_log(robot.name, status_description, reason)
        
        # ✅ 로봇이 새 업무를 받을 수 있는 상태가 되면 대기열 확인
        if new_state in [RobotState.CHARGING, RobotState.RETURNING]:
            self.task_queue.robot_became_available(robot.name)
            # TaskQueue 상태 업데이트 발행
            self.publish_task_queue_status()

    def load_waypoints(self):
        try:
            script_dir = pathlib.Path(__file__).parent.resolve()
            waypoint_file_path = script_dir / 'waypoints.yaml'
            with open(waypoint_file_path, 'r') as f: 
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint 파일 로드 실패: {e}")
            return None

    def setup_robot_subscriptions(self):
        """모든 로봇 관련 구독을 설정"""
        for name in self.robots.keys():
            # 포즈 구독
            self.create_subscription(
                PoseWithCovarianceStamped, 
                f'/{name}/amcl_pose', 
                lambda msg, rn=name: self.pose_callback(msg, rn), 
                10
            )
            
            # 🗺️ GUI용 실시간 위치 구독 (amcl_pose)
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, rn=name: self.real_time_pose_callback(msg, rn),
                10
            )
            
            # 작업 결과 구독  
            self.create_subscription(
                String, 
                f'/{name}/task_result', 
                self.path_executor_result_callback, 
                10
            )
            
            # 배터리 구독
            self.create_subscription(
                Float32, 
                f'/{name}/battery_present', 
                lambda msg, rn=name: self.battery_callback(msg, rn), 
                10
            )
            
            # initialpose 구독 (2D Pose Estimate)
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/initialpose', 
                lambda msg, rn=name: self.initialpose_callback(msg, rn), 
                10
            )

    def battery_callback(self, msg, robot_name):
        """배터리 레벨 업데이트 및 자동 복귀 판단"""
        robot = self.robots.get(robot_name)
        if not robot:
            return
        
        robot.battery_level = msg.data
        
        # 🔋 GUI로 배터리 정보 전송
        self.publish_robot_position_with_battery(robot_name)
        
        # 배터리 부족 시 자동 복귀
        if (robot.battery_level < BATTERY_THRESHOLD and 
            robot.state not in [RobotState.RETURNING, RobotState.CHARGING, RobotState.OFF_DUTY]):
            
            self.publish_status_log(robot_name, "BATTERY_LOW", f"🔋 배터리 부족({robot.battery_level:.1f}%)! 자동 복귀합니다.")
            self.force_return_to_charge(robot_name)

    def get_item_aruco_id(self, item_name):
        """물품에 해당하는 ArUco ID 반환"""
        aruco_ids = ITEM_ARUCO_MAP.get(item_name, [None])
        return aruco_ids[0] if aruco_ids else None

    # === 장소 관리 ===
    def release_robot_current_location(self, robot: RobotInfo):
        if robot.current_location and robot.current_location not in ROBOT_CHARGE_STATIONS.values():
            self.request_location_update(robot, robot.current_location, 'available', lambda r, f: None)
            robot.current_location = None

    def process_tasks(self):
        current_time = time.time()
        
        # 🔋 주기적으로 모든 로봇의 배터리/위치 정보를 GUI로 전송 (5초마다)
        if not hasattr(self, '_last_gui_update_time'):
            self._last_gui_update_time = 0
        if current_time - self._last_gui_update_time > 5.0:
            self._last_gui_update_time = current_time
            for robot_name in self.robots.keys():
                self.publish_robot_position_with_battery(robot_name)
        
        for robot in self.robots.values():
            if not robot.current_task: 
                continue
                
            # ✅ 비상정지 상태에서는 타임아웃 체크 및 작업 처리 안 함
            if robot.state == RobotState.EMERGENCY_STOP:
                continue
                
            if robot.state == RobotState.AWAITING_PICKUP_RESERVATION:
                self.request_location_update(robot, robot.current_task.pickup_location, 'reserved', self.pickup_reservation_callback)
            elif robot.state == RobotState.WAITING_AT_PICKUP_QUEUE:
                # 픽업대기장소에서 대기 중인 로봇 - 픽업대 예약 재시도
                self.request_location_update(robot, robot.current_task.pickup_location, 'reserved', self.pickup_queue_reservation_callback)
            elif robot.state == RobotState.AWAITING_DEST_RESERVATION:
                self.request_location_update(robot, robot.current_task.destination, 'reserved', self.dest_reservation_callback)

            # ✅ 실제 작업 중인 상태에서만 타임아웃 체크 (1분)
            # 이동 중이거나 배달 중인데 오래 멈춰있는 경우만 체크
            if robot.state in [RobotState.MOVING_TO_PICKUP, RobotState.MOVING_TO_DEST, 
                              RobotState.RETURNING]:
                if current_time - robot.last_activity_time > 60.0:  # 1분 타임아웃
                    self.handle_robot_timeout(robot)

    def handle_reservation_timeout(self, robot):
        """예약 타임아웃 처리"""
        self.publish_status_log(robot.name, "TIMEOUT", f"⏰ 예약 요청이 30초 동안 실패했습니다. 업무를 취소합니다.")
        robot.current_task = None
        robot.reservation_start_time = None
        self.change_robot_state(robot, RobotState.IDLE, "예약 타임아웃으로 업무 취소")

    def handle_robot_timeout(self, robot):
        """로봇 응답 없음 처리 - 주행 중 1분 이상 멈춰있는 경우"""
        reason = f"{robot.state.name} 상태에서 1분 이상 응답이 없어 강제 복귀합니다."
        self.publish_status_log(robot.name, "FAILURE", reason)
        
        # 현재 업무를 강제 취소하고 충전소로 복귀시킴
        self.force_return_to_charge(robot.name)

    def request_location_update(self, robot: RobotInfo, location: str, new_status: str, callback):
        """장소 상태 업데이트 요청"""
        if self.simulation_mode:
            self.sim_test.update_location_status(robot, location, new_status, callback)
            return
        if not self.loc_update_cli.service_is_ready(): 
            return
        request = UpdateLocationStatus.Request(location_name=location, status=new_status)
        future = self.loc_update_cli.call_async(request)
        future.add_done_callback(partial(callback, robot))

    # === 예약 콜백 ===
    def pickup_reservation_callback(self, robot: RobotInfo, future):
        if future.result().success:
            robot.reservation_failure_logged = False
            self.change_robot_state(robot, RobotState.MOVING_TO_PICKUP, "픽업대로 이동 시작")
            self.navigate_robot(robot.name, robot.current_task.pickup_location)
            self.release_robot_current_location(robot)
        else:
            # 픽업대가 busy이면 픽업대기장소로 이동
            if not robot.reservation_failure_logged:
                self.publish_status_log(robot.name, "WAITING", f"⏳ 픽업대가 사용 중입니다. 픽업대기장소로 이동합니다.")
                robot.reservation_failure_logged = True
                self.change_robot_state(robot, RobotState.MOVING_TO_PICKUP_WAIT, "픽업대기장소로 이동")
                self.navigate_robot(robot.name, "픽업대기장소")
                self.release_robot_current_location(robot)
            else:
                self.publish_status_log(robot.name, "WAITING", "픽업대 예약 실패 - 픽업대기장소에서 대기")

    def pickup_queue_reservation_callback(self, robot: RobotInfo, future):
        """픽업대기장소에서 대기 중인 로봇의 픽업대 예약 콜백"""
        if future.result().success:
            # 픽업대 예약 성공 - 픽업대기장소 해제하고 픽업대로 이동
            self.change_robot_state(robot, RobotState.MOVING_TO_PICKUP, "픽업대 예약 성공, 픽업대로 이동")
            self.navigate_robot(robot.name, robot.current_task.pickup_location)
            self.request_location_update(robot, "픽업대기장소", 'available', lambda r, f: None)  # 픽업대기장소 해제
            self.publish_status_log(robot.name, "INFO", f"✅ 픽업대 예약 성공! 픽업대기장소에서 픽업대로 이동합니다.")
        else:
            # 아직 픽업대가 busy - 계속 대기
            self.publish_status_log(robot.name, "WAITING", "픽업대기장소에서 계속 대기")

    def dest_reservation_callback(self, robot: RobotInfo, future):
        if future.result().success:
            robot.reservation_failure_logged = False
            self.change_robot_state(robot, RobotState.MOVING_TO_DEST, f"{robot.current_task.destination}로 이동 시작")
            # GUI 업무 상태를 "작업중"으로 업데이트
            if robot.current_task:
                self.publish_task_status_update(robot.current_task.order_id, "IN_PROGRESS", robot.name, robot.current_task.destination, robot.current_task.item)
            self.navigate_robot(robot.name, robot.current_task.destination)
            self.release_robot_current_location(robot)
        else:
            # 목적지 예약 실패 - 현재 위치에 따라 다르게 처리
            if not robot.reservation_failure_logged:
                current_location = robot.current_location
                destination = robot.current_task.destination
                
                if current_location == "픽업대":
                    # 픽업대에서는 그대로 대기 (굳이 충전소로 갈 필요 없음)
                    self.publish_status_log(robot.name, "WAITING", f"⏳ {destination}이(가) 사용 중입니다. 픽업대에서 대기합니다.")
                    self.publish_status_log(robot.name, "WAITING", f"{destination} 예약 실패 - 픽업대에서 대기")
                elif current_location == "픽업대기장소":
                    # 픽업대기장소에서는 그대로 대기
                    self.publish_status_log(robot.name, "WAITING", f"⏳ {destination}이(가) 사용 중입니다. 픽업대기장소에서 대기합니다.")
                    self.publish_status_log(robot.name, "WAITING", f"{destination} 예약 실패 - 픽업대기장소에서 대기")
                else:
                    # 고객 장소(왼쪽방/오른쪽방/면회실/출입구)에서는 충전소로 복귀
                    self.publish_status_log(robot.name, "RETURNING", f"⏳ {destination}이(가) 사용 중입니다. 충전소로 복귀해서 대기합니다.")
                    self.change_robot_state(robot, RobotState.RETURNING, f"{destination} 예약 실패 - 충전소에서 대기")
                    charge_station = ROBOT_CHARGE_STATIONS.get(robot.name)
                    if charge_station:
                        self.navigate_robot(robot.name, charge_station)
                        self.release_robot_current_location(robot)
                
                robot.reservation_failure_logged = True
            else:
                self.publish_status_log(robot.name, "WAITING", f"{robot.current_task.destination} 예약 실패 - 현재 위치에서 대기")

    # === 이동 및 작업 처리 ===
    def path_executor_result_callback(self, msg: String):
        robot_name, result = msg.data.split('|', 1)
        robot = self.robots.get(robot_name)
        if not robot or result != "SUCCESS": 
            return

        if robot.state == RobotState.MOVING_TO_PICKUP_WAIT:
            # 픽업대기장소 도착 - 픽업대 상태 확인 후 대기
            robot.current_location = "픽업대기장소"
            self.gui_robot_position_pub.publish(String(data=f"{robot.name}|{robot.current_location}"))
            self.change_robot_state(robot, RobotState.WAITING_AT_PICKUP_QUEUE, "픽업대기장소에서 픽업대 사용 가능할 때까지 대기")
            self.request_location_update(robot, "픽업대기장소", 'busy', lambda r, f: None)
            robot.reservation_failure_logged = False  # 다시 예약 시도할 수 있도록 리셋
            self.publish_status_log(robot.name, "INFO", f"🅿️ 픽업대기장소 도착, 픽업대 사용 가능할 때까지 대기")
            
        elif robot.state == RobotState.MOVING_TO_PICKUP:
            if not robot.current_task: 
                return
            robot.current_location = robot.current_task.pickup_location
            self.gui_robot_position_pub.publish(String(data=f"{robot.name}|{robot.current_location}"))
            self.change_robot_state(robot, RobotState.PICKING_UP, "픽업 작업 시작")
            self.request_location_update(robot, robot.current_task.pickup_location, 'busy', lambda r, f: None)
            
            if self.simulation_mode:
                self.sim_test.simulate_pickup(robot.name)
            else:
                # ArUco ID 사용하여 픽업 명령
                aruco_id = self.get_item_aruco_id(robot.current_task.item)
                if aruco_id:
                    self.arm_cmd_pub.publish(Int32(data=aruco_id))
                    self.publish_status_log(robot.name, "ARM_CMD", f"🤖 로봇팔에 ArUco ID {aruco_id} 픽업 명령 전송")
                    # 로봇팔 작업중 상태로 변경
                    self.publish_status_log("robot_arm", "PICKING_UP", f"[{robot.current_task.item}] 픽업중")
                else:
                    self.get_logger().error(f"❌ [{robot.name}] '{robot.current_task.item}'에 대한 ArUco ID를 찾을 수 없습니다.")
                    # 기본값으로 1 사용
                    self.arm_cmd_pub.publish(Int32(data=1))
                    # 로봇팔 작업중 상태로 변경
                    self.publish_status_log("robot_arm", "PICKING_UP", f"[{robot.current_task.item}] 픽업중")
                
                # 🖱️ 픽업 작업 시작하자마자 바로 GUI 표시
                self.show_pickup_confirmation_gui(robot)

        elif robot.state == RobotState.MOVING_TO_DEST:
            if not robot.current_task: 
                return
            destination = robot.current_task.destination
            robot.current_location = destination
            self.gui_robot_position_pub.publish(String(data=f"{robot.name}|{robot.current_location}"))
            
            if robot.current_task.item:
                self.change_robot_state(robot, RobotState.AWAITING_CONFIRMATION, f"{destination}에서 배달 확인 대기")
                self.request_location_update(robot, destination, 'busy', lambda r, f: None)
                self.request_delivery_confirmation(robot) # 바로 GUI 확인 요청
            else:
                # ✅ 이동 업무 완료 - TaskQueue에서 업무 완료 처리
                self.task_queue.complete_task(robot.name)
                
                # GUI 업무 상태를 "작업완료"로 업데이트
                self.publish_task_status_update(robot.current_task.order_id, "COMPLETED", robot.name, robot.current_task.destination, robot.current_task.item)
                
                self.request_location_update(robot, destination, 'busy', lambda r, f: None)
                self.change_robot_state(robot, RobotState.WAITING, f"{destination}에서 호출 대기")
                robot.current_task = None
                self.publish_status_log(robot.name, "INFO", f"✅ 이동 작업이 완료되었습니다. 다음 명령을 대기합니다.")

        elif robot.state in [RobotState.RETURNING, RobotState.OFF_DUTY]:
            charge_station = ROBOT_CHARGE_STATIONS.get(robot.name)
            if charge_station:
                robot.current_location = charge_station
                self.gui_robot_position_pub.publish(String(data=f"{robot.name}|{robot.current_location}"))
            self.change_robot_state(robot, RobotState.CHARGING, "충전소 도착, 충전 시작")
            self.publish_status_log(robot.name, "INFO", f"🏠 복귀 완료. 충전 중입니다.")

    

    def simulate_confirmation_received(self, robot: RobotInfo):
        if robot.state == RobotState.AWAITING_CONFIRMATION and robot.current_task:
            self.get_logger().info(f"[{robot.name}] 배달 확인 완료. 충전소로 복귀를 시작합니다.")
            self.change_robot_state(robot, RobotState.RETURNING, "배달 확인 완료 후 충전소 복귀")
            
            charge_station_name = ROBOT_CHARGE_STATIONS.get(robot.name)
            if charge_station_name:
                self.navigate_robot(robot.name, charge_station_name)
                self.release_robot_current_location(robot)
            else:
                self.get_logger().error(f"[{robot.name}]의 충전소를 찾을 수 없습니다.")
                self.change_robot_state(robot, RobotState.IDLE, "충전소 정보 없음")
            robot.current_task = None

    def request_delivery_confirmation(self, robot: RobotInfo):
        """GUI에 수령확인 요청 (DeliveryInterface 사용)"""
        if robot.current_task:
            # 배달 완료 시간 기록
            robot.current_task.delivery_time = time.time()
            
            # DeliveryInterface를 통해 GUI 확인 요청
            task_info = {
                'order_id': robot.current_task.order_id,
                'user_id': robot.current_task.user_id,
                'robot_name': robot.name,
                'destination': robot.current_task.destination,
                'item': robot.current_task.item,
                'order_time': robot.current_task.order_time,
                'delivery_time': robot.current_task.delivery_time
            }
            
            self.delivery_interface.request_delivery_confirmation(task_info)

    def handle_gui_confirmation_response(self, confirmation_data):
        """DeliveryInterface로부터 GUI 응답을 받는 콜백"""
        try:
            order_id = confirmation_data['order_id']
            user_id = confirmation_data['user_id']
            response = confirmation_data['response']
            confirmation_time = confirmation_data['timestamp']
            
            # 업무 ID로 해당 로봇 찾기
            target_robot = None
            for robot in self.robots.values():
                if (robot.current_task and 
                    robot.current_task.order_id == order_id and
                    robot.current_task.user_id == user_id and
                    robot.state == RobotState.AWAITING_CONFIRMATION):
                    target_robot = robot
                    break
            
            if not target_robot:
                self.publish_status_log("System", "WARN", f"⚠️ 업무 ID '{order_id}' 또는 사용자 '{user_id}'에 해당하는 대기 중인 로봇을 찾을 수 없습니다.")
                return
            
            # 수령확인 시간 기록
            target_robot.current_task.confirmation_time = confirmation_time
            
            if response == "YES":
                self.handle_delivery_confirmed(target_robot)
            elif response == "NO":
                self.handle_delivery_rejected(target_robot)
            else:
                self.get_logger().warn(f"⚠️ 잘못된 응답 형식: {response} (YES 또는 NO만 가능)")
                
        except Exception as e:
            self.get_logger().error(f"❌ GUI 확인 응답 처리 중 오류: {e}")

    # task_manager.py의 handle_delivery_confirmed 함수 전체를 교체

    def handle_delivery_confirmed(self, robot: RobotInfo):
        """수령확인 YES 처리 - 사용자가 명시한 메커니즘에 따라 수정"""
        if not (robot.state == RobotState.AWAITING_CONFIRMATION and robot.current_task):
            return

        # --- ✅ 사용자가 정의한 '작업완료' 메커니즘 시작 ---

        # 1. DB에 최종 기록
        task_info = {
            'order_id': robot.current_task.order_id, 'user_id': robot.current_task.user_id,
            'robot_name': robot.name, 'item': robot.current_task.item,
            'destination': robot.current_task.destination, 'order_time': robot.current_task.order_time,
            'pickup_time': robot.current_task.pickup_time, 'delivery_time': robot.current_task.delivery_time,
            'confirmation_time': robot.current_task.confirmation_time
        }
        self.delivery_interface.log_to_database(task_info, "수령완료")

        # 2. TaskQueue에게 '업무 완료' 보고 (가장 중요!)
        #    이 함수가 호출되면 TaskQueue는 할당된 업무 목록에서 해당 건을 제거하고,
        #    robot_became_available()를 호출하여 새 업무를 찾기 시작함.
        self.task_queue.complete_task(robot.name)
        
        # 3. 로봇의 현재 업무 정보 비우기
        completed_task_dest = robot.current_task.destination
        robot.current_task = None
        
        # 4. 로봇 상태를 '복귀중'으로 변경
        self.change_robot_state(robot, RobotState.RETURNING, "수령확인 완료 후 충전소 복귀")
        
        # 5. 이제서야 '배달지'를 '비어있음' 상태로 변경
        self.request_location_update(robot, completed_task_dest, 'available', lambda r, f: None)
        
        # 6. 충전소로 복귀 시작 (이 명령은 TaskQueue에 의해 즉시 가로채일 수 있음)
        charge_station_name = ROBOT_CHARGE_STATIONS.get(robot.name)
        if charge_station_name:
            self.navigate_robot(robot.name, charge_station_name)
        else:
            self.get_logger().error(f"❌ [{robot.name}]의 충전소를 찾을 수 없습니다.")
            self.change_robot_state(robot, RobotState.IDLE, "충전소 정보 없음")

        # --- ✅ '작업완료' 메커니즘 종료 ---

    def handle_delivery_rejected(self, robot: RobotInfo):
        """수령확인 NO 처리 - 해당 위치에서 대기"""
        if robot.state == RobotState.AWAITING_CONFIRMATION and robot.current_task:
            # DeliveryInterface를 통해 DB 기록 (거부 상태)
            task_info = {
                'order_id': robot.current_task.order_id,
                'user_id': robot.current_task.user_id,
                'robot_name': robot.name,
                'item': robot.current_task.item,
                'destination': robot.current_task.destination,
                'order_time': robot.current_task.order_time,
                'pickup_time': robot.current_task.pickup_time,
                'delivery_time': robot.current_task.delivery_time,
                'confirmation_time': robot.current_task.confirmation_time
            }
            self.delivery_interface.log_to_database(task_info, "수령거부")
            
            self.publish_status_log(robot.name, "WAITING", f"⏳ 수령확인 거부. {robot.current_task.destination}에서 재확인 대기합니다.")
            self.change_robot_state(robot, RobotState.WAITING, f"{robot.current_task.destination}에서 재확인 대기")
            # 주의: 현재 위치는 busy 상태 유지, 업무도 유지
            self.publish_status_log(robot.name, "WAITING_RECONFIRM", f"{robot.current_task.destination}에서 재확인 대기")

    def retry_delivery_confirmation(self, robot_name: str):
        """수령확인 재시도 (수동 명령용)"""
        robot = self.robots.get(robot_name)
        if not robot:
            print(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.") # 명령어 직접 응답
            return
        
        if robot.state == RobotState.WAITING and robot.current_task:
            self.publish_status_log(robot.name, "INFO", f"🔄 수령확인을 재시도합니다.")
            self.change_robot_state(robot, RobotState.AWAITING_CONFIRMATION, f"{robot.current_task.destination}에서 배달 재확인 대기")
            self.request_delivery_confirmation(robot)
        else:
            self.get_logger().warn(f"⚠️ [{robot_name}] 현재 재확인 대기 상태가 아닙니다. 상태: {robot.state.name}")

    def arm_status_callback(self, msg: String):
        """로봇팔 상태 콜백 - 단일 로봇팔이므로 메시지에서 로봇 이름 파싱"""
        # 메시지 형식: "PICKUP_COMPLETE|DP_03" 또는 "PICKUP_COMPLETE"
        parts = msg.data.split('|')
        status = parts[0]
        
        if len(parts) > 1:
            robot_name = parts[1]
        else:
            # 로봇 이름이 없는 경우, 현재 픽업 중인 로봇 찾기
            robot_name = None
            for name, robot in self.robots.items():
                if robot.state == RobotState.PICKING_UP:
                    robot_name = name
                    break
            
            if not robot_name:
                self.get_logger().warn("로봇팔 상태 수신했지만 픽업 중인 로봇을 찾을 수 없습니다.")
                return
        
        robot = self.robots.get(robot_name)
        if not robot or not robot.current_task: 
            return
            
        if status == "PICKUP_COMPLETE" and robot.state == RobotState.PICKING_UP:
            self.get_logger().info(f"🤖 [{robot.name}] 로봇팔에서 PICKUP_COMPLETE 수신!")
            # 🖱️ GUI가 활성화되어 있다면 자동으로 닫고, 픽업 완료 처리
            self.handle_pickup_complete(robot)

    def navigate_robot(self, robot_name: str, destination_name: str):
        self.get_logger().info(f"🚀 navigate_robot 호출: '{robot_name}' → '{destination_name}'")
        if self.simulation_mode:
            self.sim_test.simulate_move(robot_name, destination_name)
            return
        
        # 실제 모드 경로 생성 로직
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().error(f"❌ 로봇 '{robot_name}' 정보를 찾을 수 없습니다!")
            return
        if not robot.current_pose:
            self.get_logger().error(f"❌ 로봇 '{robot_name}'의 위치 정보가 없습니다!")
            return
        
        self.get_logger().info(f"✅ 로봇 '{robot_name}' 위치 확인됨: ({robot.current_pose.position.x:.2f}, {robot.current_pose.position.y:.2f})")
        
        dest_info = next((d for d in self.waypoints['destinations'] if d['name'] == destination_name), None)
        if not dest_info:
            dest_info = self.waypoints.get(destination_name)
        if not dest_info:
            self.get_logger().error(f"'{destination_name}'에 대한 waypoint 정보를 찾을 수 없습니다.")
            return
        
        # 🚀 효율적 경로 생성: 목적지로 바로 이동
        current_x = robot.current_pose.position.x
        current_y = robot.current_pose.position.y
        destination_y = dest_info['pose']['position']['y']
        
        goal_poses = []
        
        # 현재 위치와 목적지 Y좌표 차이가 클 때만 highway 경유
        y_diff = abs(destination_y - current_y)
        if y_diff > 1.0:  # 1m 이상 차이날 때만 highway 경유
            path_name = 'highway_down' if destination_y < current_y else 'highway_up'
            highway_points = self.waypoints.get(path_name, [])
            
            # 가장 가까운 highway 진입점 찾기
            if highway_points:
                closest_point = None
                min_distance = float('inf')
                
                for point in highway_points:
                    point_x = float(point['pose']['position']['x'])
                    point_y = float(point['pose']['position']['y'])
                    distance = ((current_x - point_x)**2 + (current_y - point_y)**2)**0.5
                    
                    if distance < min_distance:
                        min_distance = distance
                        closest_point = point
                
                # 가장 가까운 진입점만 추가
                if closest_point:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(closest_point['pose']['position']['x'])
                    pose.pose.position.y = float(closest_point['pose']['position']['y'])
                    pose.pose.orientation.w = 1.0
                    goal_poses.append(pose)
                    self.get_logger().info(f"🛣️ Highway 경유: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        else:
            self.get_logger().info(f"🎯 목적지까지 직진: Y차이 {y_diff:.2f}m")
        
        final_pose = PoseStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose.position.x = float(dest_info['pose']['position']['x'])
        final_pose.pose.position.y = float(dest_info['pose']['position']['y'])
        final_pose.pose.orientation.w = 1.0
        goal_poses.append(final_pose)

        path_msg = Path(header=final_pose.header, poses=goal_poses)
        waypoint_count = len(goal_poses)
        route_type = "경유" if waypoint_count > 1 else "직진"
        self.get_logger().info(f"📡 Path 발행: '{robot_name}' → '{destination_name}' ({route_type}: {waypoint_count}개 지점)")
        self.path_pubs[robot_name].publish(path_msg)

    # === 업무 할당 ===
    def assign_new_task(self, robot_name, item, destination, user_id="user123"):
        robot = self.robots.get(robot_name)
        if not robot: 
            return
        
        # 🔍 위치 정보 없는 로봇은 업무 할당 거부
        if not robot.current_pose:
            self.get_logger().error(f"❌ [{robot_name}] 위치 정보가 없어서 업무 할당이 불가합니다. 로봇 연결을 확인해주세요.")
            return

        # === 🧠 지능적 업무 할당 로직 시작 ===
        
        # 1. 로봇이 복귀 중(RETURNING)일 경우 - '업무 가로채기' 수행
        if robot.state == RobotState.RETURNING:
            self.get_logger().info(f"🔄 [{robot_name}] 복귀 중인 로봇에게 새 업무 할당. 복귀를 중단합니다.")
            
            # 1-1. 현재 진행중인 복귀 내비게이션을 즉시 '취소'
            self.cancel_current_navigation(robot_name)
            
            # 1-2. 현재 위치 정보는 그대로 유지한 채 새 업무를 할당. (기억상실 방지)
            # release_robot_current_location을 호출하지 않으므로 위치 정보가 유지됩니다.
            
        # 2. 로봇이 충전 중이거나 대기 중일 경우 - 정상적으로 새 업무 시작
        elif robot.state in [RobotState.IDLE, RobotState.CHARGING, RobotState.WAITING]:
            # 이 경우는 정상적인 흐름이므로 별도 처리가 필요 없습니다.
            pass

        # 3. 그 외 상태 (이미 다른 임무 수행 중 등)일 경우, 할당 거부
        else:
            print(f"'{robot_name}'은 현재 새 작업을 받을 수 없는 상태입니다: {robot.state.name}")
            return
        
        # === 🧠 공통 업무 설정 로직 ===
        
        robot.current_task = Task(robot_name, destination, item=item, user_id=user_id)
        robot.reservation_start_time = time.time()
        self.change_robot_state(robot, RobotState.AWAITING_PICKUP_RESERVATION, f"{destination}에 {item} 배달 업무 시작")
        
        self.publish_status_log(robot.name, "NEW_TASK", f"📝 새 배달 업무 할당: '{robot.name}' -> '{destination}'에 '{item}' 배달 (주문자: {user_id})")
        self.publish_status_log(robot.name, "TASK_ID", f"📝 업무 ID: {robot.current_task.order_id}")

    def assign_move_task(self, robot_name, destination):
        robot = self.robots.get(robot_name)
        if not robot:
            return
        
        # 🔍 위치 정보 없는 로봇은 업무 할당 거부
        if not robot.current_pose:
            self.get_logger().error(f"❌ [{robot_name}] 위치 정보가 없어서 이동 할당이 불가합니다. 로봇 연결을 확인해주세요.")
            return
            
        if robot.state not in [RobotState.IDLE, RobotState.CHARGING, RobotState.WAITING]:
            self.get_logger().warn(f"'{robot_name}'은 현재 새 작업을 받을 수 없는 상태입니다: {robot.state.name}")
            return
        robot.current_task = Task(robot_name, destination)
        self.change_robot_state(robot, RobotState.AWAITING_DEST_RESERVATION, f"{destination}로 이동 업무 시작")
        self.get_logger().info(f"📝 새 이동 업무 할당: '{robot.name}' -> '{destination}'(으)로 이동")
        
        # GUI용 업무 상태 업데이트 발행
        self.publish_task_status_update(robot.current_task.order_id, "ASSIGNED", robot_name, destination, None)

    # === 상태 확인 및 제어 ===
    def get_robot_location(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        logical_location = robot.current_location if robot.current_location else "위치 정보 없음"
        coordinate_info = ""
        
        if robot.current_pose and not self.simulation_mode:
            x, y = robot.current_pose.position.x, robot.current_pose.position.y
            coordinate_info = f" (좌표: {x:.2f}, {y:.2f})"
            
            from config import LOCATIONS
            closest_location = None
            min_distance = float('inf')
            
            for loc_name, coords in LOCATIONS.items():
                # config.py 형태에 따라 분기 처리
                if isinstance(coords, dict):  # {'x': 0.0, 'y': 0.9, 'z': 0.0}
                    loc_x, loc_y = coords['x'], coords['y']
                else:  # (0.0, 0.9)
                    loc_x, loc_y = coords
                    
                distance = ((x - loc_x) ** 2 + (y - loc_y) ** 2) ** 0.5
                if distance < min_distance and distance < 0.5:
                    min_distance = distance
                    closest_location = loc_name
            
            if closest_location and closest_location != logical_location:
                coordinate_info += f" → 실제로는 '{closest_location}' 근처"
        
        if self.simulation_mode:
            location_analysis = self.sim_test.analyze_location_occupancy(robot_name, logical_location)
        else:
            location_analysis = "🔍 실제 모드에서는 LocationManager 상태 확인이 필요합니다."
        
        self.get_logger().info(f"📍 [{robot_name}] 논리적 위치: {logical_location}{coordinate_info}")
        if location_analysis:
            self.get_logger().info(f"🔍 [{robot_name}] {location_analysis}")
        
        self.publish_status_log(robot_name, "LOCATION_CHECK", f"위치: {logical_location}{coordinate_info}")

    def get_robot_status(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        state_description = robot.state.name
        
        if robot.current_task:
            if robot.current_task.item:
                task_info = f"{robot.current_task.destination}에 {robot.current_task.item} 배달 중"
            else:
                task_info = f"{robot.current_task.destination}로 이동 중"
        else:
            task_info = "진행 중인 업무 없음"
        
        self.get_logger().info(f"🤖 [{robot_name}] 상태: {state_description} | 업무: {task_info}")
        if not self.simulation_mode:
            self.get_logger().info(f"🔋 [{robot_name}] 배터리: {robot.battery_level:.1f}%")
        
        self.publish_status_log(robot_name, "STATUS_CHECK", f"{state_description} | {task_info}")

    def refresh_robot(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        self.get_logger().info(f"🔄 [{robot_name}] 상태 새로고침을 시작합니다...")
        old_state = robot.state
        current_location = robot.current_location if robot.current_location else "불명"
        
        if robot.current_task:
            if robot.state in [RobotState.AWAITING_PICKUP_RESERVATION, RobotState.AWAITING_DEST_RESERVATION]:
                self.get_logger().info(f"💡 [{robot_name}] 예약 대기 중인 업무를 즉시 재시도합니다.")
                # ⚡ 바로 업무 처리 호출
                self.trigger_immediate_task_processing(robot_name)
            elif robot.state in [RobotState.MOVING_TO_PICKUP, RobotState.MOVING_TO_DEST, RobotState.RETURNING]:
                if self.simulation_mode:
                    self.get_logger().info(f"💡 [{robot_name}] 이동 중인 로봇의 이동을 재시작합니다.")
                    destination = robot.current_task.destination if robot.current_task else "충전소"
                    self.sim_test.simulate_move(robot_name, destination)
                else:
                    self.get_logger().info(f"💡 [{robot_name}] 실제 모드에서는 경로 재전송이 필요할 수 있습니다.")
            elif robot.state == RobotState.PICKING_UP and self.simulation_mode:
                self.get_logger().info(f"💡 [{robot_name}] 픽업 작업을 재시작합니다.")
                self.sim_test.simulate_pickup(robot_name)
            elif robot.state == RobotState.DELIVERING and self.simulation_mode:
                self.get_logger().info(f"💡 [{robot_name}] 배달 작업을 재시작합니다.")
                self.sim_test.simulate_delivery(robot)
            elif robot.state == RobotState.AWAITING_CONFIRMATION and self.simulation_mode:
                self.get_logger().info(f"💡 [{robot_name}] 확인 대기를 재시작합니다.")
                self.sim_test.simulate_confirmation(robot)
        else:
            if robot.state not in [RobotState.IDLE, RobotState.CHARGING, RobotState.WAITING]:
                self.get_logger().info(f"💡 [{robot_name}] 업무가 없는데 비정상 상태입니다. IDLE로 복구합니다.")
                self.change_robot_state(robot, RobotState.IDLE, "수동 복구")
                robot.current_task = None
        
        robot.reservation_failure_logged = False
        self.get_logger().info(f"✅ [{robot_name}] 새로고침 완료. 위치: {current_location}, 상태: {old_state.name} → {robot.state.name}")
        self.publish_status_log(robot_name, "REFRESHED", f"수동 새로고침 완료 - {old_state.name} → {robot.state.name}")

    # task_manager.py의 force_return_to_charge 함수 전체를 교체
    def force_return_to_charge(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return

        old_state = robot.state
        
        # ✅ TaskQueue에 할당된 업무가 있는지 직접 확인하여 취소 및 재등록
        if robot_name in self.task_queue.assigned_tasks:
            self.task_queue.cancel_and_requeue_by_robot(robot_name)
        
        # 로봇의 현재 위치가 있다면 비워줌
        self.release_robot_current_location(robot)
        # 로봇 객체의 업무 정보도 비워줌
        robot.current_task = None

        charge_station_name = ROBOT_CHARGE_STATIONS.get(robot_name)
        if not charge_station_name:
            self.get_logger().error(f"❌ [{robot_name}]의 충전소 정보를 찾을 수 없습니다.")
            return
        
        self.change_robot_state(robot, RobotState.OFF_DUTY, f"강제 복귀 명령 - {old_state.name}에서 중단")
        self.navigate_robot(robot_name, charge_station_name)
        
        self.get_logger().info(f"🏠 [{robot_name}] 충전소로 강제 복귀를 시작합니다.")
        self.publish_status_log(robot_name, "FORCE_RETURN", f"강제 복귀 - 업무 취소하고 충전소로")

    def emergency_stop(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        if robot.state == RobotState.EMERGENCY_STOP:
            self.get_logger().info(f"ℹ️ [{robot_name}] 이미 비상정지 상태입니다.")
            return
        
        robot.suspended_state = robot.state
        robot.suspended_task = robot.current_task
        
        self.change_robot_state(robot, RobotState.EMERGENCY_STOP, f"비상정지 - {robot.suspended_state.name}에서 중단")
        
        self.get_logger().info(f"🛑 [{robot_name}] 비상정지! 현재 위치에서 대기합니다.")
        self.get_logger().info(f"💾 [{robot_name}] 진행 중인 업무가 보존되었습니다. '계속해' 명령으로 재개 가능합니다.")
        
        task_info = ""
        if robot.suspended_task:
            if robot.suspended_task.item:
                task_info = f" (보존된 업무: {robot.suspended_task.destination}에 {robot.suspended_task.item} 배달)"
            else:
                task_info = f" (보존된 업무: {robot.suspended_task.destination}로 이동)"
        
        self.publish_status_log(robot_name, "EMERGENCY_STOP", f"비상정지{task_info}")

    def resume_robot(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        if robot.state != RobotState.EMERGENCY_STOP:
            self.get_logger().warn(f"⚠️ [{robot_name}] 비상정지 상태가 아닙니다. 현재 상태: {robot.state.name}")
            return
        
        if not robot.suspended_state or not robot.suspended_task:
            self.get_logger().warn(f"⚠️ [{robot_name}] 복원할 업무가 없습니다. IDLE 상태로 전환합니다.")
            self.change_robot_state(robot, RobotState.IDLE, "비상정지 해제 - 복원할 업무 없음")
            robot.suspended_state = None
            robot.suspended_task = None
            return
        
        restored_state = robot.suspended_state
        robot.current_task = robot.suspended_task
        robot.suspended_state = None
        robot.suspended_task = None
        
        # ✅ 재개 시 타임아웃 시간 리셋
        robot.reservation_start_time = time.time()
        robot.last_activity_time = time.time()
        
        self.change_robot_state(robot, restored_state, f"업무 재개 - {restored_state.name}로 복원")
        
        if robot.current_task.item:
            self.get_logger().info(f"▶️ [{robot_name}] 배달 업무 재개: {robot.current_task.destination}에 {robot.current_task.item} 배달")
        else:
            self.get_logger().info(f"▶️ [{robot_name}] 이동 업무 재개: {robot.current_task.destination}로 이동")
        
        # 상태에 따른 추가 처리
        if restored_state in [RobotState.AWAITING_PICKUP_RESERVATION, RobotState.AWAITING_DEST_RESERVATION]:
            self.get_logger().info(f"🔄 [{robot_name}] 예약 작업을 즉시 재시도합니다.")
            robot.reservation_failure_logged = False
        elif restored_state in [RobotState.MOVING_TO_PICKUP, RobotState.MOVING_TO_DEST, RobotState.RETURNING]:
            if self.simulation_mode:
                self.get_logger().info(f"🚶 [{robot_name}] 이동을 재시작합니다.")
                destination = robot.current_task.destination if robot.current_task else "충전소"
                self.sim_test.simulate_move(robot_name, destination)
        elif restored_state == RobotState.PICKING_UP and self.simulation_mode:
            self.get_logger().info(f"📦 [{robot_name}] 픽업 작업을 재시작합니다.")
            self.sim_test.simulate_pickup(robot_name)
        elif restored_state == RobotState.DELIVERING and self.simulation_mode:
            self.get_logger().info(f"🚚 [{robot_name}] 배달 작업을 재시작합니다.")
            self.sim_test.simulate_delivery(robot)
        elif restored_state == RobotState.AWAITING_CONFIRMATION and self.simulation_mode:
            self.get_logger().info(f"⏳ [{robot_name}] 확인 대기를 재시작합니다.")
            self.sim_test.simulate_confirmation(robot)
        
        self.publish_status_log(robot_name, "RESUMED", f"업무 재개 - {restored_state.name}")

    def pose_callback(self, msg, robot_name):
        robot = self.robots[robot_name]
        was_pose_missing = robot.current_pose is None
        robot.current_pose = msg.pose.pose
        robot.last_activity_time = time.time()  # 활동 시간 업데이트
        
        # 🚀 위치 정보를 처음 받았을 때 처리
        if was_pose_missing:
            if robot.current_task:
                self.get_logger().info(f"📡 [{robot_name}] 위치 정보 수신! 대기 중인 업무를 즉시 처리합니다.")
                # 타이머를 기다리지 않고 바로 처리
                self.trigger_immediate_task_processing(robot_name)
            else:
                # 업무가 없으면 대기열에서 자동 할당 시도
                self.get_logger().info(f"📡 [{robot_name}] 위치 정보 수신! 작업 가능한 로봇이 되었습니다.")
                self.task_queue.robot_became_available(robot_name)

    def real_time_pose_callback(self, msg, robot_name):
        """🗺️ GUI용 실시간 로봇 위치 브로드캐스트"""
        try:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            
            # 메시지 형식: "robot_name|x|y"
            pose_msg = String()
            pose_msg.data = f"{robot_name}|{x:.3f}|{y:.3f}"
            self.gui_realtime_pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ [{robot_name}] 실시간 위치 브로드캐스트 실패: {e}")

    def trigger_immediate_task_processing(self, robot_name: str):
        """특정 로봇의 업무를 즉시 처리 (타이머 대기 없이)"""
        robot = self.robots.get(robot_name)
        if not robot or not robot.current_task:
            return
            
        # process_tasks의 로직을 개별 로봇에 대해 실행
        if robot.state == RobotState.AWAITING_PICKUP_RESERVATION:
            self.request_location_update(robot, robot.current_task.pickup_location, 'reserved', self.pickup_reservation_callback)
        elif robot.state == RobotState.WAITING_AT_PICKUP_QUEUE:
            self.request_location_update(robot, robot.current_task.pickup_location, 'reserved', self.pickup_queue_reservation_callback)
        elif robot.state == RobotState.AWAITING_DEST_RESERVATION:
            self.request_location_update(robot, robot.current_task.destination, 'reserved', self.dest_reservation_callback)
        
        self.get_logger().info(f"⚡ [{robot_name}] 즉시 업무 처리 완료!")

    def initialpose_callback(self, msg, robot_name):
        """초기 위치 설정 시 현재 위치 알림"""
        robot = self.robots[robot_name]
        if not robot:
            return
        
        # 위치 좌표 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 가장 가까운 장소 찾기
        from config import LOCATIONS
        closest_location = None
        min_distance = float('inf')
        
        for location_name, location_data in LOCATIONS.items():
            if 'pose' in location_data:
                loc_x = location_data['pose']['position']['x']
                loc_y = location_data['pose']['position']['y']
                distance = ((x - loc_x)**2 + (y - loc_y)**2)**0.5
                
                if distance < min_distance and distance < 1.0:  # 1m 이내
                    min_distance = distance
                    closest_location = location_name
        
        # 위치 알림 메시지
        if closest_location:
            location_msg = f"장소({closest_location})에 있음"
        else:
            location_msg = f"좌표({x:.2f}, {y:.2f})"
        
        self.get_logger().info(f"📍 {robot_name} 현재 위치 수신! {location_msg}")
        self.publish_status_log(robot_name, "POSITION_UPDATE", f"현재 위치: {location_msg}")

    def manual_arrival(self, robot_name: str):
        """수동 도착 처리 - 테스트용"""
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        if robot.state == RobotState.MOVING_TO_PICKUP:
            self.get_logger().info(f"📦 [{robot_name}] 수동 픽업대 도착 처리")
            # 시뮬레이트 SUCCESS 결과
            self.path_executor_result_callback(String(data=f"{robot_name}|SUCCESS"))
        elif robot.state == RobotState.MOVING_TO_DEST:
            self.get_logger().info(f"🎯 [{robot_name}] 수동 목적지 도착 처리")
            # 시뮬레이트 SUCCESS 결과  
            self.path_executor_result_callback(String(data=f"{robot_name}|SUCCESS"))
        elif robot.state == RobotState.RETURNING:
            self.get_logger().info(f"🏠 [{robot_name}] 수동 충전소 도착 처리")
            # 시뮬레이트 SUCCESS 결과
            self.path_executor_result_callback(String(data=f"{robot_name}|SUCCESS"))
        else:
            self.get_logger().warn(f"⚠️ [{robot_name}] 이동 중이 아님 (현재 상태: {robot.state.name})")

    # === 전체 로봇 명령어 함수들 ===
    def get_all_robot_status(self):
        """모든 로봇의 현재 상태 조회"""
        self.get_logger().info("📊 전체 로봇 상태 조회 시작")
        for robot_name, robot in self.robots.items():
            current_location = robot.current_location or "위치 불명"
            if robot.current_task:
                if robot.current_task.item:
                    task_info = f"{robot.current_task.item} → {robot.current_task.destination}"
                else:
                    task_info = f"이동 → {robot.current_task.destination}"
            else:
                task_info = "대기 중"
            
            self.get_logger().info(f"🤖 [{robot_name}] 상태: {robot.state.name}, 위치: {current_location}, 업무: {task_info}, 배터리: {robot.battery_level:.1f}%")

    def get_all_robot_locations(self):
        """모든 로봇의 현재 위치 조회"""
        self.get_logger().info("📍 전체 로봇 위치 조회 시작")
        for robot_name, robot in self.robots.items():
            current_location = robot.current_location or "위치 불명"
            self.get_logger().info(f"🤖 [{robot_name}] 현재 위치: {current_location}")

    def refresh_all_robots(self):
        """모든 로봇 상태 새로고침"""
        self.get_logger().info("🔄 전체 로봇 새로고침 시작")
        for robot_name in self.robots.keys():
            self.refresh_robot(robot_name)
        self.get_logger().info("✅ 전체 로봇 새로고침 완료")

    def emergency_stop_all_robots(self):
        """모든 로봇 비상정지"""
        self.get_logger().info("🛑 전체 로봇 비상정지 시작")
        for robot_name in self.robots.keys():
            self.emergency_stop(robot_name)
        self.get_logger().info("🛑 전체 로봇 비상정지 완료")

    def return_all_robots_to_charge(self):
        """모든 로봇을 충전소로 복귀"""
        self.get_logger().info("🏠 전체 로봇 충전소 복귀 시작")
        for robot_name in self.robots.keys():
            self.force_return_to_charge(robot_name)
        self.get_logger().info("🏠 전체 로봇 충전소 복귀 명령 완료")