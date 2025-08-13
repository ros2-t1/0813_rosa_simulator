#!/usr/bin/env python3
# simulation_test.py

import threading
import time
from std_msgs.msg import String

class SimulationTest:
    """시뮬레이션 모드 전용 테스트 헬퍼 클래스"""
    
    def __init__(self, task_manager):
        self.task_manager = task_manager
        
        # ⭐️ 시뮬레이션 타이밍 설정 (테스트하기 쉽게 조정)
        self.MOVE_TIME = 2.0      # 이동 시간 (초)
        self.PICKUP_TIME = 2.0    # 픽업 시간 (초) 
        self.DELIVERY_TIME = 2.0  # 배달 시간 (초)
        self.CONFIRM_TIME = 30.0  # 확인 대기 시간 (초) - GUI에서 수동으로 하므로 길게

        # ✅ 누락되었던 이동 타이머 관리용 딕셔너리 추가
        self.move_timers = {}
        
        # 시뮬레이션 모드용 장소 상태 추적
        self.simulated_location_states = {
            '픽업대': 'available',
            '왼쪽방': 'available', 
            '오른쪽방': 'available',
            '면회실': 'available',
            '출입구': 'available',
            '픽업대기장소': 'available',
            '3번 충전소': 'busy',  # DP_03이 초기에 있음
            '8번 충전소': 'available',
            '9번 충전소': 'available'
        }
    
    # simulate_move 함수 수정

    def simulate_move(self, robot_name: str, destination: str):
        """이동 시뮬레이션"""
        self.task_manager.get_logger().debug(f"(시뮬레이션) '{destination}'으로 이동 중... ({self.MOVE_TIME}초 소요)")
        
        # ✅ 이동 타이머를 생성하고 딕셔너리에 저장
        timer = threading.Timer(self.MOVE_TIME, 
                               lambda: self.task_manager.path_executor_result_callback(
                                   String(data=f"{robot_name}|SUCCESS")))
        self.move_timers[robot_name] = timer
        timer.start()
    
    # simulation_test.py 의 SimulationTest 클래스 안에 추가
    def cancel_move(self, robot_name):
        """진행중인 이동 시뮬레이션(타이머)을 취소합니다."""
        if robot_name in self.move_timers:
            self.move_timers[robot_name].cancel()
            del self.move_timers[robot_name]
            # ✅ 수정된 코드: self.task_manager 를 통해 로그 기능 사용
            self.task_manager.get_logger().info(f"✅ (시뮬레이션) [{robot_name}] 이동 타이머 취소 완료.")

    def simulate_pickup(self, robot_name: str):
        """픽업 시뮬레이션"""
        # 픽업 시작 로그
        self.task_manager.publish_status_log("robot_arm", "PICKUP_START", f"ArUco 마커 인식 및 픽업 시작 for {robot_name}")
        
        self.task_manager.get_logger().info(f"🤖 (시뮬레이션) 로봇팔 픽업 작업 시작... ({self.PICKUP_TIME}초 소요)")
        threading.Timer(self.PICKUP_TIME, 
                       lambda: self.task_manager.arm_status_callback(
                           String(data=f"PICKUP_COMPLETE|{robot_name}"))).start()
    
    def simulate_delivery(self, robot):
        """배달 시뮬레이션"""
        self.task_manager.get_logger().info(f"🚚 (시뮬레이션) 배달 작업 진행 중... ({self.DELIVERY_TIME}초 소요)")
        
        # 배달 시간 기록
        if robot.current_task:
            robot.current_task.delivery_time = time.time()
            
        threading.Timer(self.DELIVERY_TIME, 
                       lambda: self.task_manager.simulate_delivery_completion(robot)).start()
    
    def simulate_confirmation(self, robot):
        """확인 대기 시뮬레이션"""
        self.task_manager.get_logger().debug(f"(시뮬레이션) 배달 확인 대기... ({self.CONFIRM_TIME}초 소요)")
        threading.Timer(self.CONFIRM_TIME, 
                       lambda: self.task_manager.simulate_confirmation_received(robot)).start()
    
    def check_location_status(self, location: str):
        """장소 상태 확인"""
        return self.simulated_location_states.get(location, 'unknown')
    
    def update_location_status(self, robot, location: str, new_status: str, callback):
        """장소 상태 업데이트 (시뮬레이션용)"""
        self.task_manager.get_logger().debug(f"(시뮬레이션) '{location}' 상태를 '{new_status}'(으)로 변경 요청...")
        time.sleep(0.5)
        
        current_status = self.simulated_location_states.get(location, 'unknown')
        success = False
        
        if new_status == 'reserved':
            if current_status == 'available':
                self.simulated_location_states[location] = 'reserved'
                success = True
                # ✅ GUI용으로 상태 변경을 브로드캐스트합니다.
                self.task_manager.broadcast_location_status_change(location, "reserved")
            else:
                success = False
                
        elif new_status == 'busy':
            # 로봇은 '예약된' 장소에만 '점유' 상태로 변경할 수 있습니다.
            if current_status == 'reserved':
                self.simulated_location_states[location] = 'busy'
                success = True
                # ✅ GUI용으로 상태 변경을 브로드캐스트합니다.
                self.task_manager.broadcast_location_status_change(location, "busy")
            else:
                success = False
                
        elif new_status == 'available':
            self.simulated_location_states[location] = 'available'
            success = True
            # ✅ GUI용으로 상태 변경을 브로드캐스트합니다.
            self.task_manager.broadcast_location_status_change(location, "available")
        
        class MockFuture:
            def __init__(self, success_val):
                self.success_val = success_val
            def result(self):
                class MockResponse: 
                    def __init__(self, success_val):
                        self.success = success_val
                return MockResponse(self.success_val)
        
        callback(robot, MockFuture(success))
    
    def analyze_location_occupancy(self, robot_name: str, location: str):
        """장소 점유 상태 분석"""
        if not location or location == "위치 정보 없음":
            return "⚠️  위치 정보가 없습니다. 로봇이 미아 상태일 수 있습니다."
        
        current_status = self.simulated_location_states.get(location, 'unknown')
        
        if current_status == 'busy':
            # 해당 장소를 점유하고 있는 다른 로봇이 있는지 확인
            other_robots = [name for name, robot in self.task_manager.robots.items() 
                          if robot.current_location == location and name != robot_name]
            
            if other_robots:
                return f"⚠️  '{location}'이 BUSY 상태이며, 다른 로봇({', '.join(other_robots)})도 같은 위치에 있습니다!"
            else:
                return f"✅ '{location}'을 정상적으로 점유 중입니다. (BUSY)"
                
        elif current_status == 'reserved':
            return f"🔄 '{location}'이 RESERVED 상태입니다. 다른 로봇이 이곳으로 오는 중일 수 있습니다."
            
        elif current_status == 'available':
            return f"⚠️  '{location}'이 AVAILABLE 상태인데 로봇이 여기 있습니다. 상태 불일치!"
            
        else:
            return f"❓ '{location}' 상태를 알 수 없습니다. ({current_status})"