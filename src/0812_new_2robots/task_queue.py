#!/usr/bin/env python3
# task_queue.py

"""
ROSA 업무 대기열 및 자동 로봇 할당 시스템
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
import math
from config import ROBOT_NAMES, LOCATIONS, ROBOT_CHARGE_STATIONS

class TaskPriority(Enum):
    """업무 우선순위"""
    HIGH = auto()      # 재등록된 업무 (중단된 업무)
    NORMAL = auto()    # 일반 신규 업무
    LOW = auto()       # 낮은 우선순위

class RobotSelectionCriteria:
    """로봇 선택 기준"""
    @staticmethod
    def calculate_distance(robot_location: str, target_location: str) -> float:
        """두 위치 간 거리 계산"""
        if robot_location not in LOCATIONS or target_location not in LOCATIONS:
            return float('inf')
        
        robot_pos = LOCATIONS[robot_location]
        target_pos = LOCATIONS[target_location]
        
        # 유클리드 거리 계산
        dx = robot_pos[0] - target_pos[0]
        dy = robot_pos[1] - target_pos[1]
        return math.sqrt(dx*dx + dy*dy)

@dataclass
class QueuedTask:
    """대기열에 있는 업무"""
    task_id: str
    robot_name: Optional[str]  # None이면 자동 할당
    item: Optional[str]
    destination: str
    user_id: str = "user123"
    priority: TaskPriority = TaskPriority.NORMAL
    created_time: float = field(default_factory=time.time)
    retry_count: int = 0
    original_robot: Optional[str] = None  # 원래 할당된 로봇 (재등록 시)

    def __post_init__(self):
        if not self.task_id:
            self.task_id = f"task_{int(self.created_time)}_{self.destination}"

class TaskQueue:
    """ROSA 업무 대기열 관리자"""
    
    def __init__(self, task_manager):
        self.task_manager = task_manager
        self.pending_tasks: List[QueuedTask] = []
        self.assigned_tasks: Dict[str, QueuedTask] = {}  # robot_name -> task
        self.task_counter = 0
        
        self.logger = task_manager.get_logger()
        self.logger.info("✅ TaskQueue 시스템 초기화 완료")

    def add_task(self, robot_name: Optional[str], item: Optional[str], destination: str, 
                 user_id: str = "user123", priority: TaskPriority = TaskPriority.NORMAL,
                 original_robot: Optional[str] = None) -> str:
        """새 업무를 대기열에 추가"""
        
        self.task_counter += 1
        if self.task_counter > 99:
            self.task_counter = 1  # 100개 이상 시 순환
        task_id = f"ROSA_{self.task_counter:02d}"
        
        queued_task = QueuedTask(
            task_id=task_id,
            robot_name=robot_name,
            item=item,
            destination=destination,
            user_id=user_id,
            priority=priority,
            original_robot=original_robot
        )
        
        # 우선순위에 따라 삽입 위치 결정
        if priority == TaskPriority.HIGH:
            # 재등록된 업무는 맨 앞에
            self.pending_tasks.insert(0, queued_task)
            self.logger.info(f"🔄 재등록 업무 추가 (최우선): {task_id} - {destination}")
        else:
            # 일반 업무는 뒤에
            self.pending_tasks.append(queued_task)
            self.logger.info(f"📝 신규 업무 추가: {task_id} - {destination}")
        
        # GUI에 "등록" 상태로 업무 추가 발행 (TaskManager 참조)
        robot_display = robot_name if robot_name else "자동할당"
        self.task_manager.publish_task_status_update(task_id, "PENDING", robot_display, destination, item)
        
        # 바로 할당 시도
        self._try_assign_pending_tasks()
        
        # TaskQueue 상태 업데이트 발행
        self.task_manager.publish_task_queue_status()
        
        return task_id

    def requeue_task(self, robot_name: str, reason: str = "업무 중단"):
        """중단된 업무를 재등록"""
        if robot_name not in self.assigned_tasks:
            return
        
        failed_task = self.assigned_tasks.pop(robot_name)
        failed_task.retry_count += 1
        failed_task.priority = TaskPriority.HIGH  # 최우선순위로 변경
        failed_task.original_robot = robot_name
        
        # 맨 앞에 재등록
        self.pending_tasks.insert(0, failed_task)
        
        self.logger.info(f"🔄 업무 재등록: {failed_task.task_id} (이유: {reason}, 시도횟수: {failed_task.retry_count})")
        self.task_manager.publish_status_log("TaskQueue", "REQUEUE", 
                                            f"업무 재등록: {failed_task.task_id} - {reason}")
        
        # 바로 할당 시도
        self._try_assign_pending_tasks()
        
        # TaskQueue 상태 업데이트 발행
        self.task_manager.publish_task_queue_status()

    def robot_became_available(self, robot_name: str):
        """로봇이 사용 가능해졌을 때 호출"""
        self.logger.info(f"🤖 {robot_name} 사용 가능해짐 - 대기열 확인 중...")
        self._try_assign_pending_tasks()
        
        # TaskQueue 상태 업데이트 발행
        self.task_manager.publish_task_queue_status()

    def complete_task(self, robot_name: str):
        """업무 완료 처리"""
        if robot_name in self.assigned_tasks:
            completed_task = self.assigned_tasks.pop(robot_name)
            self.logger.info(f"✅ 업무 완료: {completed_task.task_id}")
            self.task_manager.publish_status_log("TaskQueue", "COMPLETED", 
                                                f"업무 완료: {completed_task.task_id}")
            
            # TaskQueue 상태 업데이트 발행
            self.task_manager.publish_task_queue_status()

    def cancel_and_requeue_by_robot(self, robot_name: str, reason: str = "관리자 명령으로 강제 복귀"):
        """외부 요청(TaskManager)으로 로봇의 현재 업무를 취소하고 재등록합니다."""
        if robot_name in self.assigned_tasks:
            failed_task = self.assigned_tasks.pop(robot_name)  # 할당 목록에서 제거
            
            # 재등록 로직 (requeue_task와 유사)
            failed_task.retry_count += 1
            failed_task.priority = TaskPriority.HIGH
            failed_task.original_robot = robot_name
            self.pending_tasks.insert(0, failed_task) # 대기열 맨 앞에 추가
            
            self.logger.info(f"🔄 외부 요청으로 업무 재등록: {failed_task.task_id} (로봇: {robot_name}, 이유: {reason})")
            self.task_manager.publish_status_log("TaskQueue", "REQUEUE", f"업무 강제 재등록: {failed_task.task_id}")
            self.task_manager.publish_task_queue_status() # ✅ TaskManager를 통해 호출하도록 수정
            return True
        return False

    def _try_assign_pending_tasks(self):
        """대기 중인 업무들을 할당 시도"""
        if not self.pending_tasks:
            return
        
        assigned_count = 0
        remaining_tasks = []
        
        for task in self.pending_tasks:
            if self._assign_single_task(task):
                assigned_count += 1
            else:
                remaining_tasks.append(task)
        
        self.pending_tasks = remaining_tasks
        
        if assigned_count > 0:
            self.logger.info(f"📋 대기열에서 {assigned_count}개 업무 할당 완료")
            self._print_queue_status()
            
            # TaskQueue 상태 업데이트 발행
            self.task_manager.publish_task_queue_status()

    def _assign_single_task(self, task: QueuedTask) -> bool:
        """단일 업무 할당 시도"""
        
        # 1. 로봇이 지정된 경우
        if task.robot_name:
            if self._can_assign_to_robot(task.robot_name):
                self._execute_assignment(task.robot_name, task)
                return True
            else:
                # 지정된 로봇이 사용불가면 대기
                return False
        
        # 2. 자동 할당 - 가용한 로봇 찾기
        best_robot = self._find_best_available_robot(task)
        if best_robot:
            self._execute_assignment(best_robot, task)
            return True
        
        return False

    def _can_assign_to_robot(self, robot_name: str) -> bool:
        """로봇이 업무를 받을 수 있는 상태인지 확인"""
        if robot_name not in self.task_manager.robots:
            return False
        
        robot = self.task_manager.robots[robot_name]
        
        # 🔍 위치 정보가 없으면 작업 불가능한 로봇으로 간주
        if not robot.current_pose:
            return False
        
        # 이미 업무가 할당된 로봇은 불가
        if robot_name in self.assigned_tasks:
            return False
        
        # 다시 원래대로 복구
        from task_manager import RobotState
        assignable_states = [RobotState.CHARGING, RobotState.RETURNING]
        
        return robot.state in assignable_states

    def _find_best_available_robot(self, task: QueuedTask) -> Optional[str]:
        """최적의 가용 로봇 찾기 (우선순위 기반)"""
        available_robots = []
        
        for robot_name in ROBOT_NAMES:
            if self._can_assign_to_robot(robot_name):
                robot = self.task_manager.robots[robot_name]
                available_robots.append((robot_name, robot))
        
        if not available_robots:
            # 🔍 사용 가능한 로봇이 없는 이유 분석
            connected_robots = []
            disconnected_robots = []
            
            for robot_name in ROBOT_NAMES:
                robot = self.task_manager.robots[robot_name]
                if robot.current_pose:
                    connected_robots.append(robot_name)
                else:
                    disconnected_robots.append(robot_name)
            
            # 🔇 중복 메시지 방지: 최근 5초 내에 같은 메시지 출력했으면 스킵
            current_time = time.time()
            if not hasattr(self, '_last_warning_time'):
                self._last_warning_time = {}
            
            if not connected_robots:
                warning_key = "no_connected_robots"
                if warning_key not in self._last_warning_time or current_time - self._last_warning_time[warning_key] > 5.0:
                    self._last_warning_time[warning_key] = current_time
                    self.logger.error("❌ 위치 정보가 없어 작업이 가능한 로봇이 없습니다. 로봇 연결을 확인해주세요.")
                    self.logger.info(f"   연결 상태: {', '.join([f'{name}(연결안됨)' for name in disconnected_robots])}")
            else:
                busy_status = []
                for name in connected_robots:
                    robot = self.task_manager.robots[name]
                    if name in self.assigned_tasks:
                        busy_status.append(f"{name}(업무중)")
                    else:
                        busy_status.append(f"{name}({robot.state.name})")
                
                warning_key = "robots_busy"
                if warning_key not in self._last_warning_time or current_time - self._last_warning_time[warning_key] > 5.0:
                    self._last_warning_time[warning_key] = current_time
                    self.logger.info(f"📝 업무가 대기열에 추가되었습니다. 모든 로봇이 사용 중: {', '.join(busy_status)}")
            
            return None
        
        # 우선순위 기반 정렬
        def priority_score(robot_tuple):
            robot_name, robot = robot_tuple
            
            # 1. 상태 우선순위: 충전중 > 복귀중
            from task_manager import RobotState
            state_priority = 0
            if robot.state == RobotState.CHARGING:
                state_priority = 100
            elif robot.state == RobotState.RETURNING:
                state_priority = 50
            
            # 2. 배터리 잔량 (높을수록 좋음)
            battery_score = robot.battery_level
            
            # 3. 거리 (가까울수록 좋음, 역수 사용)
            distance = RobotSelectionCriteria.calculate_distance(
                robot.current_location or ROBOT_CHARGE_STATIONS.get(robot_name, "3번 충전소"),
                task.destination
            )
            distance_score = 100 / (distance + 1) if distance != float('inf') else 0
            
            # 총점 계산 (상태 > 배터리 > 거리)
            total_score = state_priority * 1000 + battery_score * 10 + distance_score
            
            return total_score
        
        # 점수가 높은 순으로 정렬
        sorted_robots = sorted(available_robots, key=priority_score, reverse=True)
        best_robot_name = sorted_robots[0][0]
        
        self.logger.info(f"🎯 최적 로봇 선택: {best_robot_name} (총 {len(available_robots)}대 중)")
        return best_robot_name

    def _execute_assignment(self, robot_name: str, task: QueuedTask):
        """실제 업무 할당 실행"""
        # 대기열에서 할당된 업무로 이동
        self.assigned_tasks[robot_name] = task
        
        # TaskManager의 기존 assign_new_task 호출
        if task.item:
            self.task_manager.assign_new_task(robot_name, task.item, task.destination, task.user_id)
        else:
            self.task_manager.assign_move_task(robot_name, task.destination)
        
        self.logger.info(f"🚀 업무 할당 실행: {task.task_id} → {robot_name}")
        self.task_manager.publish_status_log("TaskQueue", "ASSIGNED", 
                                            f"업무 할당: {task.task_id} → {robot_name}")

    def _print_queue_status(self):
        """대기열 상태 출력"""
        pending_count = len(self.pending_tasks)
        assigned_count = len(self.assigned_tasks)
        
        status_msg = f"\n📊 업무 대기열 상태:"
        status_msg += f"\n   대기 중: {pending_count}개"
        status_msg += f"\n   할당됨: {assigned_count}개"
        
        if self.pending_tasks:
            status_msg += f"\n   📋 대기 업무:"
            for i, task in enumerate(self.pending_tasks[:3]):  # 최대 3개만 표시
                priority_icon = "🔴" if task.priority == TaskPriority.HIGH else "🟡"
                status_msg += f"\n     {i+1}. {priority_icon} {task.task_id}: {task.destination}"
            if len(self.pending_tasks) > 3:
                status_msg += f"\n     ... 외 {len(self.pending_tasks)-3}개"
        
        if self.assigned_tasks:
            status_msg += f"\n   🤖 할당된 업무:"
            for robot_name, task in self.assigned_tasks.items():
                status_msg += f"\n     {robot_name}: {task.task_id} ({task.destination})"
        
        self.logger.info(status_msg)

    def get_queue_status(self) -> Dict:
        """GUI용 대기열 상태 반환"""
        return {
            'pending_count': len(self.pending_tasks),
            'assigned_count': len(self.assigned_tasks),
            'pending_tasks': [
                {
                    'task_id': task.task_id,
                    'destination': task.destination,
                    'item': task.item,
                    'priority': task.priority.name,
                    'robot_name': task.robot_name,
                    'retry_count': task.retry_count
                }
                for task in self.pending_tasks
            ],
            'assigned_tasks': {
                robot_name: {
                    'task_id': task.task_id,
                    'destination': task.destination,
                    'item': task.item
                }
                for robot_name, task in self.assigned_tasks.items()
            }
        }