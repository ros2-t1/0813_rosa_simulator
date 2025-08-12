#!/usr/bin/env python3
# location_manager.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from your_custom_interfaces.srv import GetLocationStatus, UpdateLocationStatus # 사용자 정의 서비스 타입
import threading

class LocationManager(Node):
    """모든 주요 장소의 상태(비어있음, 예약중, 업무중)를 중앙에서 관리합니다."""

    def __init__(self):
        super().__init__('location_manager')
        
        # 장소 상태를 저장할 딕셔너리 (예: 'pickup_zone': 'available')
        self.location_states = {
            '픽업대': 'available', # available, reserved, busy
            '왼쪽방': 'available',
            '오른쪽방': 'available',
            '3번 충전소': 'busy', # 초기 상태는 로봇이 있다고 가정
            '8번 충전소': 'available',
            '9번 충전소': 'available'
        }
        self.lock = threading.Lock() # 동시 접근을 막기 위한 Lock

        # 1. 장소 상태 조회 서비스
        self.create_service(GetLocationStatus, 'get_location_status', self.get_status_callback)
        
        # 2. 장소 예약 서비스
        self.create_service(UpdateLocationStatus, 'reserve_location', self.reserve_callback)

        # 3. 장소 상태 업데이트 서비스 (사용 완료 후 'available'로 변경 등)
        self.create_service(UpdateLocationStatus, 'update_location_status', self.update_status_callback)

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
            else:
                response.success = False
                self.get_logger().warn(f"📍 '{request.location_name}' 예약 실패 (현재 상태: {current_status}).")
        return response

    def update_status_callback(self, request, response):
        with self.lock:
            if request.location_name in self.location_states:
                self.location_states[request.location_name] = request.status
                response.success = True
                self.get_logger().info(f"📍 '{request.location_name}' 상태 변경 -> {request.status}.")
            else:
                response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LocationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
