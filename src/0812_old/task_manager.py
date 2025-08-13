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
    MOVING_TO_DEST = auto(); DELIVERING = auto()
    AWAITING_CONFIRMATION = auto()
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
        
        # GUI/DB 연동 인터페이스 초기화
        self.delivery_interface = DeliveryInterface(task_manager_callback=self.handle_gui_confirmation_response)
        
        # ✅ 통합된 LocationManager 기능 초기화
        self.setup_location_manager()
        
        # 초기 상태 설정
        for robot_name, robot in self.robots.items():
            robot.current_location = ROBOT_CHARGE_STATIONS.get(robot_name)
            if robot_name == "DP_03":
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

        self.task_processor_timer = self.create_timer(10.0, self.process_tasks)

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
                self.get_logger().info(f"📍 '{request.location_name}' 예약 완료")
                self.broadcast_location_status_change(request.location_name, 'reserved')
            else:
                response.success = False
                self.get_logger().warn(f"📍 '{request.location_name}' 예약 실패 (현재 상태: {current_status})")
        return response

    def update_location_status_callback(self, request, response):
        """장소 상태 업데이트"""
        with self.location_lock:
            if request.location_name in self.location_states:
                old_status = self.location_states[request.location_name]
                self.location_states[request.location_name] = request.status
                response.success = True
                self.get_logger().info(f"📍 '{request.location_name}' 상태 변경: {old_status} → {request.status}")
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

    def change_robot_state(self, robot: RobotInfo, new_state: RobotState, reason: str = ""):
        old_state = robot.state
        robot.state = new_state
        status_description = f"{old_state.name} → {new_state.name}"
        self.publish_status_log(robot.name, status_description, reason)

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

    def battery_callback(self, msg, robot_name):
        """배터리 레벨 업데이트 및 자동 복귀 판단"""
        robot = self.robots.get(robot_name)
        if not robot:
            return
        
        robot.battery_level = msg.data
        
        # 배터리 부족 시 자동 복귀
        if (robot.battery_level < BATTERY_THRESHOLD and 
            robot.state not in [RobotState.RETURNING, RobotState.CHARGING, RobotState.OFF_DUTY]):
            
            self.get_logger().warn(f"🔋 [{robot_name}] 배터리 부족({robot.battery_level:.1f}%)! 자동 복귀합니다.")
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
                              RobotState.RETURNING, RobotState.DELIVERING]:
                if current_time - robot.last_activity_time > 60.0:  # 1분 타임아웃
                    self.handle_robot_timeout(robot)

    def handle_reservation_timeout(self, robot):
        """예약 타임아웃 처리"""
        self.get_logger().warn(f"⏰ [{robot.name}] 예약 요청이 30초 동안 실패했습니다. 업무를 취소합니다.")
        robot.current_task = None
        robot.reservation_start_time = None
        self.change_robot_state(robot, RobotState.IDLE, "예약 타임아웃으로 업무 취소")

    def handle_robot_timeout(self, robot):
        """로봇 응답 없음 처리 - 실제 작업 중 멈춰있는 경우만"""
        self.get_logger().error(f"🚨 [{robot.name}] 로봇이 1분 동안 응답하지 않습니다! 상태: {robot.state.name}")
        
        # 상태에 따른 구체적인 에러 메시지
        if robot.state == RobotState.MOVING_TO_PICKUP:
            reason = "픽업대로 이동 중 응답 없음"
        elif robot.state == RobotState.MOVING_TO_DEST:
            reason = f"{robot.current_task.destination}로 이동 중 응답 없음"
        elif robot.state == RobotState.RETURNING:
            reason = "충전소 복귀 중 응답 없음"
        elif robot.state == RobotState.DELIVERING:
            reason = "배달 작업 중 응답 없음"
        else:
            reason = f"{robot.state.name} 상태에서 응답 없음"
            
        self.publish_status_log(robot.name, "TIMEOUT", reason)
        
        # 자동 복구 시도 (선택사항)
        # self.refresh_robot(robot.name)

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
                self.get_logger().warn(f"⏳ [{robot.name}] 픽업대가 사용 중입니다. 픽업대기장소로 이동합니다.")
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
            self.get_logger().info(f"✅ [{robot.name}] 픽업대 예약 성공! 픽업대기장소에서 픽업대로 이동합니다.")
        else:
            # 아직 픽업대가 busy - 계속 대기
            self.publish_status_log(robot.name, "WAITING", "픽업대기장소에서 계속 대기")

    def dest_reservation_callback(self, robot: RobotInfo, future):
        if future.result().success:
            robot.reservation_failure_logged = False
            self.change_robot_state(robot, RobotState.MOVING_TO_DEST, f"{robot.current_task.destination}로 이동 시작")
            self.navigate_robot(robot.name, robot.current_task.destination)
            self.release_robot_current_location(robot)
        else:
            # 목적지 예약 실패 - 현재 위치에 따라 다르게 처리
            if not robot.reservation_failure_logged:
                current_location = robot.current_location
                destination = robot.current_task.destination
                
                if current_location == "픽업대":
                    # 픽업대에서는 그대로 대기 (굳이 충전소로 갈 필요 없음)
                    self.get_logger().warn(f"⏳ [{robot.name}] {destination}이(가) 사용 중입니다. 픽업대에서 대기합니다.")
                    self.publish_status_log(robot.name, "WAITING", f"{destination} 예약 실패 - 픽업대에서 대기")
                elif current_location == "픽업대기장소":
                    # 픽업대기장소에서는 그대로 대기
                    self.get_logger().warn(f"⏳ [{robot.name}] {destination}이(가) 사용 중입니다. 픽업대기장소에서 대기합니다.")
                    self.publish_status_log(robot.name, "WAITING", f"{destination} 예약 실패 - 픽업대기장소에서 대기")
                else:
                    # 고객 장소(왼쪽방/오른쪽방/면회실/출입구)에서는 충전소로 복귀
                    self.get_logger().warn(f"⏳ [{robot.name}] {destination}이(가) 사용 중입니다. 충전소로 복귀해서 대기합니다.")
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
            self.change_robot_state(robot, RobotState.WAITING_AT_PICKUP_QUEUE, "픽업대기장소에서 픽업대 사용 가능할 때까지 대기")
            self.request_location_update(robot, "픽업대기장소", 'busy', lambda r, f: None)
            robot.reservation_failure_logged = False  # 다시 예약 시도할 수 있도록 리셋
            self.get_logger().info(f"🅿️ [{robot.name}] 픽업대기장소 도착, 픽업대 사용 가능할 때까지 대기")
            
        elif robot.state == RobotState.MOVING_TO_PICKUP:
            if not robot.current_task: 
                return
            self.change_robot_state(robot, RobotState.PICKING_UP, "픽업 작업 시작")
            robot.current_location = robot.current_task.pickup_location
            self.request_location_update(robot, robot.current_task.pickup_location, 'busy', lambda r, f: None)
            
            if self.simulation_mode:
                self.sim_test.simulate_pickup(robot.name)
            else:
                # ArUco ID 사용하여 픽업 명령
                aruco_id = self.get_item_aruco_id(robot.current_task.item)
                if aruco_id:
                    self.arm_cmd_pub.publish(Int32(data=aruco_id))
                    self.get_logger().info(f"🤖 [{robot.name}] 로봇팔에 ArUco ID {aruco_id} 픽업 명령 전송")
                else:
                    self.get_logger().error(f"❌ [{robot.name}] '{robot.current_task.item}'에 대한 ArUco ID를 찾을 수 없습니다.")
                    # 기본값으로 1 사용
                    self.arm_cmd_pub.publish(Int32(data=1))

        elif robot.state == RobotState.MOVING_TO_DEST:
            if not robot.current_task: 
                return
            destination = robot.current_task.destination
            robot.current_location = destination
            
            if robot.current_task.item:
                self.change_robot_state(robot, RobotState.DELIVERING, f"{destination}에서 {robot.current_task.item} 배달 중")
                self.request_location_update(robot, destination, 'busy', lambda r, f: None)
                if self.simulation_mode:
                    self.sim_test.simulate_delivery(robot)
            else:
                self.request_location_update(robot, destination, 'busy', lambda r, f: None)
                self.change_robot_state(robot, RobotState.WAITING, f"{destination}에서 호출 대기")
                robot.current_task = None
                self.get_logger().info(f"✅ [{robot.name}]의 이동 작업이 완료되었습니다. 다음 명령을 대기합니다.")

        elif robot.state in [RobotState.RETURNING, RobotState.OFF_DUTY]:
            charge_station = ROBOT_CHARGE_STATIONS.get(robot.name)
            if charge_station:
                robot.current_location = charge_station
            self.change_robot_state(robot, RobotState.CHARGING, "충전소 도착, 충전 시작")
            self.get_logger().info(f"🏠 [{robot.name}] 복귀 완료. 충전 중입니다.")

    def simulate_delivery_completion(self, robot: RobotInfo):
        if robot.state == RobotState.DELIVERING and robot.current_task:
            self.change_robot_state(robot, RobotState.AWAITING_CONFIRMATION, f"{robot.current_task.destination}에서 배달 확인 대기")
            
            # 시뮬레이션 모드에서도 실제 GUI 확인 요청을 보냄
            self.get_logger().info(f"📋 [{robot.name}] 배달 완료! GUI 수령확인 요청을 보냅니다.")
            self.request_delivery_confirmation(robot)
            
            # 시뮬레이션에서 자동 확인은 제거 (GUI에서 수동으로 해야 함)
            # if self.simulation_mode:
            #     self.sim_test.simulate_confirmation(robot)

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
                self.get_logger().warn(f"⚠️ 업무 ID '{order_id}' 또는 사용자 '{user_id}'에 해당하는 대기 중인 로봇을 찾을 수 없습니다.")
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

    def handle_delivery_confirmed(self, robot: RobotInfo):
        """수령확인 YES 처리 - 업무 완료 후 복귀"""
        if robot.state == RobotState.AWAITING_CONFIRMATION and robot.current_task:
            # DeliveryInterface를 통해 DB 기록
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
            self.delivery_interface.log_to_database(task_info, "수령완료")
            
            self.get_logger().info(f"✅ [{robot.name}] 수령확인 완료! 충전소로 복귀를 시작합니다.")
            self.change_robot_state(robot, RobotState.RETURNING, "수령확인 완료 후 충전소 복귀")
            
            # 현재 위치(배달지) 상태를 available로 변경
            self.release_robot_current_location(robot)
            
            # 충전소로 복귀
            charge_station_name = ROBOT_CHARGE_STATIONS.get(robot.name)
            if charge_station_name:
                self.navigate_robot(robot.name, charge_station_name)
            else:
                self.get_logger().error(f"❌ [{robot.name}]의 충전소를 찾을 수 없습니다.")
                self.change_robot_state(robot, RobotState.IDLE, "충전소 정보 없음")
            
            robot.current_task = None

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
            
            self.get_logger().info(f"⏳ [{robot.name}] 수령확인 거부. {robot.current_task.destination}에서 재확인 대기합니다.")
            self.change_robot_state(robot, RobotState.WAITING, f"{robot.current_task.destination}에서 재확인 대기")
            # 주의: 현재 위치는 busy 상태 유지, 업무도 유지
            self.publish_status_log(robot.name, "WAITING_RECONFIRM", f"{robot.current_task.destination}에서 재확인 대기")

    def retry_delivery_confirmation(self, robot_name: str):
        """수령확인 재시도 (수동 명령용)"""
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        if robot.state == RobotState.WAITING and robot.current_task:
            self.get_logger().info(f"🔄 [{robot_name}] 수령확인을 재시도합니다.")
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
            # 픽업 시간 기록
            if robot.current_task:
                robot.current_task.pickup_time = time.time()
            
            # 로봇팔 상태 로그 발행
            self.publish_status_log("robot_arm", "PICKUP_COMPLETE", f"{robot.current_task.item} 픽업 완료 for {robot_name}")
            
            self.change_robot_state(robot, RobotState.AWAITING_DEST_RESERVATION, "픽업 완료, 목적지 예약 대기")

    def navigate_robot(self, robot_name: str, destination_name: str):
        self.get_logger().debug(f"➡️ '{robot_name}'에게 '{destination_name}'으로 이동 명령")
        if self.simulation_mode:
            self.sim_test.simulate_move(robot_name, destination_name)
            return
        
        # 실제 모드 경로 생성 로직
        robot = self.robots.get(robot_name)
        if not robot or not robot.current_pose: 
            return
        
        dest_info = next((d for d in self.waypoints['destinations'] if d['name'] == destination_name), None)
        if not dest_info:
            dest_info = self.waypoints.get(destination_name)
        if not dest_info:
            self.get_logger().error(f"'{destination_name}'에 대한 waypoint 정보를 찾을 수 없습니다.")
            return
        
        # 경로 생성 및 발행
        current_y = robot.current_pose.position.y
        destination_y = dest_info['pose']['position']['y']
        path_name = 'highway_down' if destination_y < current_y else 'highway_up'
        
        goal_poses = []
        for point in self.waypoints.get(path_name, []):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point['pose']['position']['x'])
            pose.pose.position.y = float(point['pose']['position']['y'])
            pose.pose.orientation.w = 1.0
            goal_poses.append(pose)
        
        final_pose = PoseStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose.position.x = float(dest_info['pose']['position']['x'])
        final_pose.pose.position.y = float(dest_info['pose']['position']['y'])
        final_pose.pose.orientation.w = 1.0
        goal_poses.append(final_pose)

        path_msg = Path(header=final_pose.header, poses=goal_poses)
        self.path_pubs[robot_name].publish(path_msg)

    # === 업무 할당 ===
    def assign_new_task(self, robot_name, item, destination, user_id="user123"):
        robot = self.robots.get(robot_name)
        if not robot or robot.state not in [RobotState.IDLE, RobotState.CHARGING, RobotState.WAITING]:
            self.get_logger().warn(f"'{robot_name}'은 현재 새 작업을 받을 수 없는 상태입니다: {robot.state.name}")
            return
        
        robot.current_task = Task(robot_name, destination, item=item, user_id=user_id)
        robot.reservation_start_time = time.time()  # 예약 시작 시간 설정
        self.change_robot_state(robot, RobotState.AWAITING_PICKUP_RESERVATION, f"{destination}에 {item} 배달 업무 시작")
        self.get_logger().info(f"📝 새 배달 업무 할당: '{robot.name}' -> '{destination}'에 '{item}' 배달 (주문자: {user_id})")
        self.get_logger().info(f"📝 업무 ID: {robot.current_task.order_id}")

    def assign_move_task(self, robot_name, destination):
        robot = self.robots.get(robot_name)
        if not robot or robot.state not in [RobotState.IDLE, RobotState.CHARGING, RobotState.WAITING]:
            self.get_logger().warn(f"'{robot_name}'은 현재 새 작업을 받을 수 없는 상태입니다: {robot.state.name}")
            return
        robot.current_task = Task(robot_name, destination)
        self.change_robot_state(robot, RobotState.AWAITING_DEST_RESERVATION, f"{destination}로 이동 업무 시작")
        self.get_logger().info(f"📝 새 이동 업무 할당: '{robot.name}' -> '{destination}'(으)로 이동")

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
                if robot.state == RobotState.AWAITING_PICKUP_RESERVATION:
                    self.request_location_update(robot, robot.current_task.pickup_location, 'reserved', self.pickup_reservation_callback)
                elif robot.state == RobotState.AWAITING_DEST_RESERVATION:
                    self.request_location_update(robot, robot.current_task.destination, 'reserved', self.dest_reservation_callback)
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

    def force_return_to_charge(self, robot_name: str):
        robot = self.robots.get(robot_name)
        if not robot:
            self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
            return
        
        old_state = robot.state
        old_task = robot.current_task
        
        self.release_robot_current_location(robot)
        
        if old_task:
            if old_task.item:
                self.get_logger().info(f"🗑️ [{robot_name}] 진행 중인 배달 업무 취소: {old_task.destination}에 {old_task.item} 배달")
            else:
                self.get_logger().info(f"🗑️ [{robot_name}] 진행 중인 이동 업무 취소: {old_task.destination}로 이동")
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
        robot.current_pose = msg.pose.pose
        robot.last_activity_time = time.time()  # 활동 시간 업데이트

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