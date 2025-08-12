# command_parser.py

import re
from config import LOCATIONS  # ⭐️ config에서 장소 목록 가져오기

class CommandParser:
    def __init__(self, task_manager):
        self.task_manager = task_manager
        self.delivery_items = ["식판", "물", "영양제"]
        
        # ⭐️ config.py의 LOCATIONS만 사용 (추가 별칭 없음)
        self.destinations = {}
        for location in LOCATIONS.keys():
            self.destinations[location] = location
        
        self.robot_aliases = {
            "3번": "DP_03", "3번로봇": "DP_03", "dp_03": "DP_03",
            "8번": "DP_08", "8번로봇": "DP_08", "dp_08": "DP_08", 
            "9번": "DP_09", "9번로봇": "DP_09", "dp_09": "DP_09",
        }
        print("✅ 명령어 해석기 준비 완료.")

    def parse_command(self, command: str):
        command_lower = command.lower().strip()

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
            elif "정신차려" in command or "새로고침" in command or "리셋" in command:
                print(f"✅ 새로고침: '{found_robot}' 상태 새로고침")
                self.task_manager.refresh_robot(found_robot)
                return
            # 제어 명령어들
            elif any(kw in command for kw in ["복귀해", "충전해"]) or ("충전소" in command and "가" in command):
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

        # 작업 명령 처리
        if found_robot and found_item and found_dest:
            print(f"✅ 배달 명령: '{found_robot}' → '{found_item}'을(를) '{found_dest}'에 배달")
            self.task_manager.assign_new_task(found_robot, found_item, found_dest)
            return

        # 이동 명령 처리
        move_keywords = ['이동', '가', '가서', 'move']
        if found_robot and found_dest and any(kw in command for kw in move_keywords):
            print(f"✅ 이동 명령: '{found_robot}' → '{found_dest}'으로 이동")
            self.task_manager.assign_move_task(found_robot, found_dest)
            return
            
        # 에러 처리
        if found_item and found_dest:
            print(f"❓ 로봇을 지정해주세요. (예: 3번 {found_item} {found_dest})")
            return

        print(f"❓ 이해할 수 없는 명령어: '{command}'")