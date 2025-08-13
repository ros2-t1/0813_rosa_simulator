#!/usr/bin/env python3
# tf_aggregator_2robots.py - DP_03, DP_09 전용 TF 중계기

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
import time
import threading

class TwoRobotTFAggregator(Node):
    """2대 로봇 TF 집계기 (DP_03, DP_09) - QoS 문제 해결"""
    
    def __init__(self):
        super().__init__('multi_robot_tf_aggregator')
        
        # QoS 프로필 설정
        self.tf_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.tf_static_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 각 로봇의 TF 구독 (QoS 적용) - DP_03, DP_09만 사용
        self.tf_sub_dp03 = self.create_subscription(
            TFMessage, '/DP_03/tf', 
            lambda msg: self.tf_callback(msg, 'DP_03'), self.tf_qos)
        self.tf_sub_dp09 = self.create_subscription(
            TFMessage, '/DP_09/tf', 
            lambda msg: self.tf_callback(msg, 'DP_09'), self.tf_qos)
        
        # 각 로봇의 TF Static 구독 (QoS 적용) - DP_03, DP_09만 사용
        self.tf_static_sub_dp03 = self.create_subscription(
            TFMessage, '/DP_03/tf_static', 
            lambda msg: self.tf_static_callback(msg, 'DP_03'), self.tf_static_qos)
        self.tf_static_sub_dp09 = self.create_subscription(
            TFMessage, '/DP_09/tf_static', 
            lambda msg: self.tf_static_callback(msg, 'DP_09'), self.tf_static_qos)
        
        # 통합 TF 발행 (QoS 적용)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', self.tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', self.tf_static_qos)
        
        # 상태 추적 - DP_03, DP_09만 사용
        self.robot_status = {'DP_03': 0, 'DP_09': 0}
        self.lock = threading.Lock()
        
        # 상태 출력 타이머
        self.create_timer(5.0, self.print_status)
        
        print("🔗 QoS 수정된 Multi-Robot TF Aggregator 시작! (2대 로봇 버전)")
        print("   - DP_03 (도메인 13) → TF 집계")
        print("   - DP_09 (도메인 19) → TF 집계")
        print("   → 통합 TF 발행: /tf, /tf_static")
    
    def tf_callback(self, msg, robot_name):
        """실시간 TF 메시지 처리"""
        try:
            with self.lock:
                self.robot_status[robot_name] = time.time()
            
            # 받은 TF를 그대로 통합 TF로 발행
            self.tf_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"TF 처리 오류 ({robot_name}): {e}")
    
    def tf_static_callback(self, msg, robot_name):
        """정적 TF 메시지 처리"""
        try:
            # 정적 TF도 통합해서 발행
            self.tf_static_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"TF Static 처리 오류 ({robot_name}): {e}")
    
    def print_status(self):
        """로봇별 TF 상태 출력"""
        current_time = time.time()
        status_msg = "\n🤖 Multi-Robot TF 상태:\n"
        
        with self.lock:
            for robot_name, last_time in self.robot_status.items():
                if last_time == 0:
                    status = "❌ 연결 안됨"
                elif current_time - last_time < 2.0:
                    status = f"✅ 활성 ({current_time - last_time:.1f}초 전)"
                else:
                    status = f"⚠️ 지연 ({current_time - last_time:.1f}초 전)"
                
                status_msg += f"  {robot_name}: {status}\n"
        
        print(status_msg)

def main():
    rclpy.init()
    node = TwoRobotTFAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 TF Aggregator 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()