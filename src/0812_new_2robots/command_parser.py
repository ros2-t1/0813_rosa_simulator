# command_parser.py

import re
from config import LOCATIONS, ROBOT_NAMES  # ⭐️ config에서 장소 목록 가져오기
from task_queue import TaskPriority

class CommandParser:
    def __init__(self, task_manager):
        self.task_manager = task_manager
        self.delivery_items = ["식판", "물", "영양제"]
        
        # ⭐️ config.py의 LOCATIONS만 사용 (추가 별칭 없음)
        self.destinations = {}
        for location in LOCATIONS.keys():
            self.destinations[location] = location
        
        # 2대 로봇 버전 - DP_03, DP_09만 사용
        self.robot_aliases = {
            "3번": "DP_03", "3번로봇": "DP_03", "dp_03": "DP_03",
            "9번": "DP_09", "9번로봇": "DP_09", "dp_09": "DP_09",
        }
        print("✅ 명령어 해석기 준비 완료.")

    def parse_command(self, command: str):
        command_lower = command.lower().strip()

        # 전체 로봇 명령어 처리 (우선 처리)
        if any(kw in command for kw in ["다들", "모두", "전체", "모든로봇"]):
            if "뭐해" in command or "상태" in command or "업무" in command:
                print("✅ 전체 상태 확인: 모든 로봇의 현재 업무 조회")
                self.task_manager.get_all_robot_status()
                return
            elif "어디야" in command or "위치" in command:
                print("✅ 전체 위치 확인: 모든 로봇의 현재 위치 조회")
                self.task_manager.get_all_robot_locations()
                return
            elif "정신차려" in command or "새로고침" in command or "리셋" in command:
                print("✅ 전체 새로고침: 모든 로봇 상태 새로고침")
                self.task_manager.refresh_all_robots()
                return
            elif any(kw in command for kw in ["멈춰", "정지", "스톱"]):
                print("🛑 전체 비상정지: 모든 로봇 즉시 정지")
                self.task_manager.emergency_stop_all_robots()
                return
            elif any(kw in command for kw in ["퇴근해", "퇴근", "집가"]):
                print("🏠 전체 퇴근: 모든 로봇 충전소 복귀")
                self.task_manager.return_all_robots_to_charge()
                return

        # 로봇 찾기
        found_robot = None
        for alias, name in self.robot_aliases.items():
            if alias in command_lower:
                found_robot = name
                break

        # 로봇별 명령어 처리
        if found_robot:
            # 상태 확인 명령어들
            if "어디야" in command or "위치" in command:
                print(f"✅ 위치 확인: '{found_robot}' 현재 위치 조회")
                self.task_manager.get_robot_location(found_robot)
                return
            elif "뭐해" in command or "상태" in command or "업무" in command:
                print(f"✅ 상태 확인: '{found_robot}' 현재 업무 조회")
                self.task_manager.get_robot_status(found_robot)
                return
            elif "도착" in command or "arrived" in command:
                print(f"✅ 수동 도착: '{found_robot}' 목적지 도착 처리")
                self.task_manager.manual_arrival(found_robot)
                return
            elif "정신차려" in command or "새로고침" in command or "리셋" in command:
                print(f"✅ 새로고침: '{found_robot}' 상태 새로고침")
                self.task_manager.refresh_robot(found_robot)
                return
            # 제어 명령어들
            elif any(kw in command for kw in ["복귀해", "퇴근해", "충전해"]) or ("충전소" in command and "가" in command):
                print(f"🏠 복귀 명령: '{found_robot}' 즉시 충전소 복귀")
                self.task_manager.force_return_to_charge(found_robot)
                return
            elif any(kw in command for kw in ["멈춰", "정지", "스톱"]):
                print(f"🛑 비상정지: '{found_robot}' 즉시 정지")
                self.task_manager.emergency_stop(found_robot)
                return
            elif any(kw in command for kw in ["계속해", "재개", "다시"]):
                print(f"▶️ 재개 명령: '{found_robot}' 업무 재개")
                self.task_manager.resume_robot(found_robot)
                return
            elif any(kw in command for kw in ["확인", "수령확인", "재확인"]):
                print(f"📋 재확인 요청: '{found_robot}' 수령확인 재시도")
                self.task_manager.retry_delivery_confirmation(found_robot)
                return

        # 물품 찾기 (정확한 단어 매칭)
        found_item = None
        for item in self.delivery_items:
            if re.search(r'\b' + re.escape(item) + r'\b', command):
                found_item = item
                break
        
        # 목적지 찾기 (config.py의 LOCATIONS만 사용)
        found_dest = None
        for location in self.destinations.keys():
            if location in command_lower:
                found_dest = location
                break

        # 작업 명령 처리 - TaskQueue 사용
        if found_robot and found_item and found_dest:
            # 로봇 지정 배달 명령
            print(f"✅ 배달 명령 (로봇 지정): '{found_robot}' → '{found_item}'을(를) '{found_dest}'에 배달")
            task_id = self.task_manager.task_queue.add_task(
                robot_name=found_robot, 
                item=found_item, 
                destination=found_dest,
                priority=TaskPriority.NORMAL
            )
            print(f"📝 업무 ID: {task_id}")
            return
        elif found_item and found_dest:
            # 자동 할당 배달 명령
            print(f"✅ 배달 명령 (자동 할당): '{found_item}'을(를) '{found_dest}'에 배달 (최적 로봇 자동 선택)")
            task_id = self.task_manager.task_queue.add_task(
                robot_name=None,  # 자동 할당
                item=found_item, 
                destination=found_dest,
                priority=TaskPriority.NORMAL
            )
            print(f"📝 업무 ID: {task_id}")
            return

        # 이동 명령 처리 - TaskQueue 사용
        move_keywords = ['이동', '가', '가서', 'move']
        if found_robot and found_dest and any(kw in command for kw in move_keywords):
            # 로봇 지정 이동 명령
            print(f"✅ 이동 명령 (로봇 지정): '{found_robot}' → '{found_dest}'으로 이동")
            task_id = self.task_manager.task_queue.add_task(
                robot_name=found_robot, 
                item=None, 
                destination=found_dest,
                priority=TaskPriority.NORMAL
            )
            print(f"📝 업무 ID: {task_id}")
            return
        elif found_dest and any(kw in command for kw in move_keywords):
            # 자동 할당 이동 명령
            print(f"✅ 이동 명령 (자동 할당): '{found_dest}'으로 이동 (최적 로봇 자동 선택)")
            task_id = self.task_manager.task_queue.add_task(
                robot_name=None,  # 자동 할당
                item=None, 
                destination=found_dest,
                priority=TaskPriority.NORMAL
            )
            print(f"📝 업무 ID: {task_id}")
            return

        # 대기열 상태 확인
        if "대기열" in command or "큐" in command or "업무" in command:
            print("📊 업무 대기열 상태:")
            status = self.task_manager.task_queue.get_queue_status()
            print(f"   대기 중: {status['pending_count']}개, 할당됨: {status['assigned_count']}개")
            if status['pending_tasks']:
                print("   📋 대기 업무:")
                for task in status['pending_tasks'][:5]:  # 최대 5개
                    priority_icon = "🔴" if task['priority'] == 'HIGH' else "🟡"
                    robot_info = f" ({task['robot_name']})" if task['robot_name'] else " (자동할당)"
                    print(f"     {priority_icon} {task['task_id']}: {task['destination']}{robot_info}")
            return

        print(f"❓ 이해할 수 없는 명령어: '{command}'")
        print("💡 사용 예시:")
        print("   - 로봇 지정: '3번 물 왼쪽방', '9번 오른쪽방 가'")
        print("   - 자동 할당: '물 왼쪽방', '오른쪽방 가' (최적 로봇 자동 선택)")
        print("   - 전체 명령: '다들 뭐해', '모두 어디야', '다들 퇴근해'")
        print("   - 상태 확인: '3번 어디야', '9번 뭐해', '대기열'")