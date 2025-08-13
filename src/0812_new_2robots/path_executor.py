#!/usr/bin/env python3
# path_executor.py (최종 완성 버전)
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
import os

class PathExecutorNode(Node):
    """
    관제 PC로부터 경로(Path) 토픽을 수신하여 로컬 Nav2 스택에 작업을 지시하고,
    완료 시 그 결과를 다시 토픽으로 보고하는 프록시 노드.
    """
    def __init__(self):
        super().__init__('path_executor')
        
        # 환경변수에서 로봇 이름을 가져와 다중 로봇에 재사용 가능하도록 함
        # 예: export ROBOT_NAME=DP_03
        self.robot_name = os.environ.get('ROBOT_NAME', 'DP_03')

        # 1. 관제 PC로부터 경로 명령을 수신하는 Subscriber
        self.subscription = self.create_subscription(
            Path, 
            f'/{self.robot_name}/waypoint_path_goal', 
            self.path_goal_callback, 
            10)
        
        # 2. 관제 PC로 작업 결과를 보고하는 Publisher
        self.result_pub = self.create_publisher(
            String, 
            f'/{self.robot_name}/task_result', 
            10)
        
        # 3. 로봇 내부의 Nav2를 제어할 Navigator (네임스페이스 없이 생성)
        self.navigator = BasicNavigator()
        
        # 4. 작업 완료 여부를 주기적으로 확인할 타이머
        self.task_monitor_timer = self.create_timer(1.0, self.monitor_task_completion)
        self.is_task_running = False
        
        self.get_logger().info(f"✅ [{self.robot_name}] 경로 실행기(최종본) 준비 완료.")
        self.get_logger().info(f"   - 구독 토픽: /{self.robot_name}/waypoint_path_goal")
        self.get_logger().info(f"   - 발행 토픽: /{self.robot_name}/task_result")

    def path_goal_callback(self, msg: Path):
        """경로 명령을 수신하면 Nav2에 작업을 전달하는 콜백"""
        if self.is_task_running:
            self.get_logger().warn(f"[{self.robot_name}] 이전 작업이 아직 진행 중입니다. 새 명령을 무시합니다.")
            return

        self.get_logger().info(f"[{self.robot_name}] 경로 명령 수신! {len(msg.poses)}개 지점 주행 시작.")
        self.navigator.followWaypoints(msg.poses)
        self.is_task_running = True # 작업 시작 플래그 설정

    def monitor_task_completion(self):
        """1초마다 Nav2 작업이 완료되었는지 확인하는 함수"""
        if not self.is_task_running:
            return # 실행 중인 작업이 없으면 아무것도 안 함

        if not self.navigator.isTaskComplete():
            # 작업이 아직 진행 중
            return

        # 작업이 완료되었을 때만 아래 코드가 실행됨
        result = self.navigator.getResult()
        result_status = "UNKNOWN"

        if result == TaskResult.SUCCEEDED:
            result_status = "SUCCESS"
            self.get_logger().info(f"[{self.robot_name}] 목표 지점 도착 성공!")
            # 도착 성공 시, 'PARKING_COMPLETE' 같은 더 구체적인 메시지를 보낼 수도 있음
            # 예: result_msg = f"{self.robot_name}|PARKING_COMPLETE"
        else:
            result_status = "FAILED" # 취소/실패 등 모든 비-성공 케이스
            self.get_logger().warn(f"[{self.robot_name}] 작업이 성공하지 못함 (상태: {result})")
        
        # '로봇이름|결과' 형식으로 관제 PC에 보고
        result_msg = f"{self.robot_name}|{result_status}"
        self.result_pub.publish(String(data=result_msg))
        
        self.is_task_running = False # 작업 완료 플래그 해제

def main(args=None):
    """메인 실행 함수"""
    rclpy.init(args=args)
    node = PathExecutorNode()
    
    # waitUntilNav2Active()를 사용하지 않고 바로 spin으로 진입하여
    # amcl 네임스페이스 불일치 에러를 회피합니다.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
