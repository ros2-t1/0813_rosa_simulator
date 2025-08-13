# status_logger.py (최종 수정)
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class StatusLoggerNode(Node):
    """'/rosa/status_log' 토픽을 구독하여 이벤트 로그를 한 줄로 출력합니다."""
    def __init__(self):
        super().__init__('rosa_status_logger')
        self.subscription = self.create_subscription(
            String,
            '/rosa/status_log',
            self.log_callback,
            10)
        
        print("="*60)
        print("🤖 ROSA 실시간 이벤트 로그 모니터 시작")
        print("="*60)

    def log_callback(self, msg: String):
        """로그 메시지를 수신하여 한 줄 형식으로 출력합니다."""
        try:
            robot_name, status, reason = msg.data.split('|', 2)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 요청하신 형식으로 한 줄 로그를 출력합니다.
            print(f"✅ [{timestamp}] {robot_name} | {status} | {reason}")

        except ValueError:
            self.get_logger().warn(f"잘못된 형식의 로그 메시지 수신: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    logger_node = StatusLoggerNode()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()