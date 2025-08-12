# robot_manager.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # 1. ActionClient import 추가
from enum import Enum, auto
import yaml  # 1. YAML 파일을 읽기 위해 추가
from std_msgs.msg import String # 상태 방송을 위한 String 메시지 추가

# --- 추가된 import ---
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Path # Path 메시지를 사용하기 위해 추가
# from nav2_msgs.action import FollowWaypoints # 2. FollowWaypoints 액션 타입 import
# from nav2_simple_commander.robot_navigator import BasicNavigator # 3. BasicNavigator는 더 이상 사용 안 함
# rclpy.action.client의 GoalHandle을 사용하기 위해 import 추가
from rclpy.action.client import ClientGoalHandle, GoalStatus

from config import ROBOT_NAMES

class RobotState(Enum):
    """로봇의 현재 상태를 정의합니다."""

    # == 업무 할당 가능 상태 ==
    CHARGING = auto()   # 충전소에 있으며, 즉시 업무 할당이 가능한 기본 상태
    RETURNING = auto()  # 충전소로 복귀 중이지만, 새 업무를 받을 수 있는 상태

    # == 업무 수행중 상태 (신규 업무 할당 불가) ==
    MOVING = auto()                 # 이동중: 특정 목적지를 향해 움직이는 상태
    PARKING = auto()                # 정밀주차: 목표 지점에서 정밀 주차 프로세스를 수행 중인 상태
    PICKING_UP = auto()             # 픽업중: 픽업대에서 물건을 싣고 있는 상태
    DELIVERING = auto()             # 배달중: 물건을 싣고 최종 목적지로 이동하는 상태
    WAITING = auto()                # 현장대기: 특정 위치에서 사용자의 요청으로 대기 중인 상태
    AWAITING_CONFIRMATION = auto()  # 확인대기: 사용자나 시스템의 응답을 기다리는 상태

class RobotInfo:
    """개별 로봇의 모든 실시간 정보를 담는 클래스입니다."""
    def __init__(self, name: str):
        self.name = name  # 로봇의 고유 이름 (예: 'DP_03')
        
        # 1. 상태 정보
        self.state: RobotState = RobotState.CHARGING  # 로봇의 현재 상태, 초기값은 CHARGING
        self.is_moving: bool = False  # 물리적으로 움직이는 중인지 여부

        # 2. 위치 정보
        self.current_pose = None  # 로봇의 현재 좌표 (x, y) 및 방향 정보
        self.last_goal_pos = None # 마지막으로 내렸던 목표 좌표

        # 3. 배터리 정보
        self.battery_level: float = 100.0  # 현재 배터리 잔량
        
        # 4. 작업 정보
        self.current_task = None  # 현재 수행 중인 작업 객체

class ROSARobotManager(Node):
    """다중 로봇의 상태를 통합 관리하고 ROS 통신을 총괄하는 클래스입니다."""

    def __init__(self):
        # 'rosa_robot_manager' 라는 이름으로 ROS 2 노드를 초기화합니다.
        super().__init__('rosa_robot_manager')

        self.status_pub = self.create_publisher(String, '/rosa/status_log', 10)
        self.get_logger().info("📢 로봇 상태 이벤트 방송을 시작합니다.")

        # --- 3. Waypoint 데이터 로딩 로직 추가 ---
        self.waypoints = None
        try:
            waypoint_file_path = '/home/addinedu/jeong/multi_robot_project/0812_new_by_g/waypoints.yaml' 
            with open(waypoint_file_path, 'r') as f:
                self.waypoints = yaml.safe_load(f)
            self.get_logger().info(f"✅ Waypoint 파일 로드 성공: {waypoint_file_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Waypoint 파일 로드 실패: {e}")

        # 로봇 이름(str)을 키로, RobotInfo 객체를 값으로 갖는 딕셔너리
        self.robots: dict[str, RobotInfo] = {}
        # --- 4. Navigator 대신 ActionClient 딕셔너리 생성 ---
        # --- 2. ActionClient 대신 Publisher 딕셔너리 생성 ---
        self.path_pubs: dict[str, rclpy.publisher.Publisher] = {}

        # config.py에 정의된 이름들을 기반으로 RobotInfo 객체들을 생성합니다.
        for name in ROBOT_NAMES:
            self.robots[name] = RobotInfo(name)
            # 모든 로봇에 대해 동일한 이름의 액션 클라이언트를 생성
            # self.action_clients[name] = ActionClient(self, FollowWaypoints, '/follow_waypoints')
            # self.get_logger().info(f"🤖 '{name}' 로봇 매니저 및 액션 클라이언트 생성 완료. (타겟: /follow_waypoints)")
            # '경로 실행기'로 Path 메시지를 보낼 Publisher를 생성합니다.
            self.path_pubs[name] = self.create_publisher(Path, f'/{name}/waypoint_path_goal', 10)
            self.get_logger().info(f"🤖 '{name}' 로봇 매니저 및 경로 퍼블리셔 생성 완료.")
        
        self.setup_robot_subscriptions()

            
    def setup_robot_subscriptions(self):
        """각 로봇의 상태 토픽들을 구독(subscribe)합니다."""
        self.get_logger().info("📡 로봇 상태 토픽 구독을 시작합니다...")
        for name in self.robots.keys():
            # 1. 위치(Pose) 정보 구독
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, robot_name=name: self.pose_callback(msg, robot_name),
                10)
            
            # 2. 배터리(Battery) 정보 구독
            self.create_subscription(
                Float32,
                f'/{name}/battery_present',
                lambda msg, robot_name=name: self.battery_callback(msg, robot_name),
                10)

            # 3. 속도(Velocity) 정보 구독 (움직임 감지용)
            self.create_subscription(
                Twist,
                f'/{name}/cmd_vel',
                lambda msg, robot_name=name: self.cmd_vel_callback(msg, robot_name),
                10)
            
            # ▼▼▼▼▼ 각 로봇의 고유 결과 토픽을 여기서 구독 ▼▼▼▼▼
            self.create_subscription(
                String,
                f'/{name}/task_result',
                self.task_result_callback,
                10)

    def pose_callback(self, msg: PoseWithCovarianceStamped, robot_name: str):
        """로봇의 위치 정보를 수신하면 호출됩니다."""
        robot = self.robots[robot_name]
        robot.current_pose = msg.pose.pose # None 대신 실제 좌표 데이터로 채움
        # self.get_logger().info(f"위치 수신: {robot_name}") # (디버그용, 로그가 너무 많아지니 주석 처리)

    def battery_callback(self, msg: Float32, robot_name: str):
        """로봇의 배터리 정보를 수신하면 호출됩니다."""
        robot = self.robots[robot_name]
        robot.battery_level = msg.data # 100.0 대신 실제 배터리 값으로 덮어씀
    
    def cmd_vel_callback(self, msg: Twist, robot_name: str):
        """로봇의 속도 명령을 수신하여 움직임 여부를 판단하고, 상태 변경 시 방송합니다."""
        robot = self.robots[robot_name]
        is_currently_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

        # is_moving 상태가 실제로 변경되었을 때만 방송
        if robot.is_moving != is_currently_moving:
            robot.is_moving = is_currently_moving
            new_state = "MOVING" if is_currently_moving else "WAITING"
            reason = "이동 시작" if is_currently_moving else "목표 도착 또는 정지"
            self.broadcast_status(robot_name, new_state, reason)


    # --- 6. 기존 move_robot_to_location 함수를 아래 navigate_robot 함수로 교체 ---
    def navigate_robot(self, robot_name: str, destination_name: str):
        """고속도로(상/하행선) 개념을 적용하여 로봇을 목적지로 이동시킵니다."""
        
        robot = self.robots.get(robot_name)
        if not robot or not robot.current_pose:
            self.get_logger().error(f"'{robot_name}'의 위치를 몰라 경로를 계획할 수 없습니다.")
            return

        dest_info = next((d for d in self.waypoints['destinations'] if d['name'] == destination_name), None)
        if not dest_info:
            self.get_logger().error(f"알 수 없는 목적지입니다: '{destination_name}'")
            return
            
        # 경로 결정 (상행선 vs 하행선)
        current_y = robot.current_pose.position.y
        destination_y = dest_info['pose']['position']['y']
        
        path_name = None
        if destination_y > current_y + 0.1:
            path_name = 'highway_up'
        elif destination_y < current_y - 0.1:
            path_name = 'highway_down'
        
        goal_poses = []
        if path_name:
            self.get_logger().info(f"🛣️  '{robot_name}' -> '{path_name}' 경로 선택됨.")
            for point in self.waypoints[path_name]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(point['pose']['position']['x'])
                pose.pose.position.y = float(point['pose']['position']['y'])
                pose.pose.orientation.w = 1.0
                goal_poses.append(pose)
        
        # 최종 목적지 추가
        final_pose = PoseStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose.position.x = float(dest_info['pose']['position']['x'])
        final_pose.pose.position.y = float(dest_info['pose']['position']['y'])
        final_pose.pose.orientation.w = 1.0
        goal_poses.append(final_pose)


        # --- 최종적으로 Path 메시지를 만들어서 Publish ---
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = goal_poses # 계산된 PoseStamped 리스트를 그대로 담습니다.

        # 해당 로봇의 Publisher를 통해 Path 메시지를 발행(전송)합니다.
        self.path_pubs[robot_name].publish(path_msg)
        self.get_logger().info(f"✅ '{robot_name}'에게 '{destination_name}' 경로 토픽 전송 완료.")

        # ▼▼▼▼▼ 여기서 broadcast_status를 호출하는 부분을 수정합니다 ▼▼▼▼▼
        # 로봇의 현재 업무 정보를 기록해둡니다.
        robot = self.robots.get(robot_name)
        if robot:
            # Task 객체를 생성해서 현재 업무를 저장해두는 것이 정석이지만,
            # 여기서는 간단하게 목적지만 저장하겠습니다.
            robot.current_task = {'destination': destination_name}

        # "이동 명령 수신" 방송
        self.broadcast_status(robot_name, "MOVING", f"'{destination_name}'으로 이동 명령 수신")

    def broadcast_status(self, robot_name: str, status: str, reason: str):
        """특정 이벤트가 발생했을 때만 상태를 방송합니다."""
        # '로봇이름|상태|이유' 형식의 문자열로 만들어 전송
        status_msg = f"{robot_name}|{status}|{reason}"
        self.status_pub.publish(String(data=status_msg))

    def task_result_callback(self, msg: String):
        """로봇으로부터 작업 결과 보고를 받으면 호출됩니다."""
        try:
            robot_name, result = msg.data.split('|', 1)

            robot = self.robots.get(robot_name)
            destination = "목표 지점"
            if robot and robot.current_task:
                destination = f"'{robot.current_task['destination']}'"

            if result == "SUCCESS":
                self.broadcast_status(robot_name, "WAITING", f"{destination} 도착")
            else:
                self.broadcast_status(robot_name, "WAITING", f"{destination} 작업 실패/취소 ({result})")

            # 작업이 끝났으므로 현재 업무 정보 초기화
            if robot:
                robot.current_task = None

        except ValueError:
            self.get_logger().warn(f"잘못된 형식의 결과 메시지 수신: {msg.data}")