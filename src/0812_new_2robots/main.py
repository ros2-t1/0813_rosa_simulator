# main.py

import rclpy
import threading

from task_manager import TaskManager
from command_parser import CommandParser

def main():
    """ROSA 시스템의 메인 실행 함수"""

    # 0. 시작 모드 선택
    sim_mode = False
    while True:
        mode = input("실행 모드를 선택하세요 (1: 실제 로봇, 2: 시뮬레이션): ")
        if mode == '1':
            sim_mode = False
            print("\n[실제 로봇 모드]로 시작합니다.\n")
            break
        elif mode == '2':
            sim_mode = True
            print("\n[시뮬레이션 모드]로 시작합니다.\n")
            break
        else:
            print("잘못된 입력입니다. 1 또는 2를 입력해주세요.")

    # 1. ROS 2 시스템을 초기화합니다.
    rclpy.init()

    # 2. 사용자가 선택한 모드로 TaskManager를 생성합니다.
    task_manager = TaskManager(simulation_mode=sim_mode)

    # 3. 명령어 해석기 객체를 생성하고, TaskManager와 연결합니다.
    command_parser = CommandParser(task_manager)
    
    # 4. 사용자 입력을 별도의 스레드에서 계속 받도록 처리합니다.
    def get_user_input():
        while True:
            try:
                command = input("명령어 입력 > ").strip()
                if command:
                    command_parser.parse_command(command)
            except (KeyboardInterrupt, EOFError):
                break
            except UnicodeDecodeError:
                print("❌ 입력 오류가 발생했습니다. 다시 입력해주세요.")
                continue
            except Exception as e:
                print(f"❌ 명령어 처리 중 오류가 발생했습니다: {e}")
                continue
    
    input_thread = threading.Thread(target=get_user_input, daemon=True)
    input_thread.start()

    # 5. ROS 2 노드를 실행합니다.
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
