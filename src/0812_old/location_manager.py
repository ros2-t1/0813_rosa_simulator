#!/usr/bin/env python3
# location_manager.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rosa_interfaces.srv import GetLocationStatus, UpdateLocationStatus # 사용자 정의 서비스 타입
import threading
import time

class LocationManager(Node):
    """모든 주요 장소의 상태(비어있음, 예약중, 업무중)를 중앙에서 관리합니다."""

    def __init__(self):
        super().__init__('location_manager')
        
        # 장소 상태를 저장할 딕셔너리 (예: 'pickup_zone': 'available')
        # 충전소는 각 로봇 전용이므로 상태 관리 불필요 (제거됨)
        self.location_states = {
            '픽업대': 'available', # available, reserved, busy
            '픽업대기장소': 'available', # 픽업대가 busy일 때 대기하는 장소
            '왼쪽방': 'available',
            '오른쪽방': 'available',
            '면회실': 'available',
            '출입구': 'available'
        }
        self.lock = threading.Lock() # 동시 접근을 막기 위한 Lock

        # 1. 장소 상태 조회 서비스
        self.create_service(GetLocationStatus, 'get_location_status', self.get_status_callback)
        
        # 2. 장소 예약 서비스
        self.create_service(UpdateLocationStatus, 'reserve_location', self.reserve_callback)

        # 3. 장소 상태 업데이트 서비스 (사용 완료 후 'available'로 변경 등)
        self.create_service(UpdateLocationStatus, 'update_location_status', self.update_status_callback)
        
        # 4. GUI용 장소 상태 브로드캐스트 토픽
        self.gui_status_pub = self.create_publisher(String, '/rosa/location_status_update', 10)

        self.get_logger().info("✅ Location Manager 준비 완료.")

    def get_status_callback(self, request, response):
        with self.lock:
            response.status = self.location_states.get(request.location_name, 'unknown')
        return response

    def reserve_callback(self, request, response):
        with self.lock:
            current_status = self.location_states.get(request.location_name)
            if current_status == 'available':
                self.location_states[request.location_name] = 'reserved'
                response.success = True
                self.get_logger().info(f"📍 '{request.location_name}' 예약 완료.")
                self.broadcast_status_change(request.location_name, 'reserved')
            else:
                response.success = False
                self.get_logger().warn(f"📍 '{request.location_name}' 예약 실패 (현재 상태: {current_status}).")
        return response

    def update_status_callback(self, request, response):
        with self.lock:
            if request.location_name in self.location_states:
                old_status = self.location_states[request.location_name]
                self.location_states[request.location_name] = request.status
                response.success = True
                self.get_logger().info(f"📍 '{request.location_name}' 상태 변경 -> {request.status}.")
                self.broadcast_status_change(request.location_name, request.status)
            else:
                response.success = False
        return response

    def broadcast_status_change(self, location_name: str, new_status: str):
        """GUI용 장소 상태 변경 브로드캐스트"""
        try:
            status_msg = String()
            timestamp = int(time.time())
            status_msg.data = f"{location_name}|{new_status}|{timestamp}"
            self.gui_status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"❌ GUI 상태 브로드캐스트 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LocationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
