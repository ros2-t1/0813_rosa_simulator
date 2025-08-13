#!/usr/bin/env python3
# gui_example.py

"""
GUI 수령확인 창 예시
팀원이 GUI 구현할 때 참고용 예시 코드
"""

import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class DeliveryConfirmationGUI(Node):
    """
    배달 수령확인 GUI 예시
    실제 구현시 팀원의 GUI 프레임워크에 맞게 수정 필요
    """
    
    def __init__(self):
        super().__init__('delivery_confirmation_gui')
        
        # ROS 토픽 구독/발행
        self.create_subscription(String, '/rosa/delivery_confirmation_request', self.confirmation_request_callback, 10)
        self.response_pub = self.create_publisher(String, '/rosa/delivery_confirmation_response', 10)
        
        # GUI 관련 변수
        self.current_task_info = None
        self.confirmation_window = None
        self.user_id = "user123"  # 실제 구현시 로그인된 사용자 ID 사용
        
        # 메인 윈도우 생성
        self.setup_main_window()
        
        self.get_logger().info("✅ 배달 수령확인 GUI 시작")

    def setup_main_window(self):
        """메인 윈도우 설정"""
        self.root = tk.Tk()
        self.root.title("ROSA 배달 시스템 - 수령확인")
        self.root.geometry("400x300")
        
        # 현재 사용자 표시
        user_label = tk.Label(self.root, text=f"현재 사용자: {self.user_id}", font=("Arial", 12))
        user_label.pack(pady=10)
        
        # 상태 표시
        self.status_label = tk.Label(self.root, text="배달 요청 대기 중...", font=("Arial", 10))
        self.status_label.pack(pady=5)
        
        # 로그 표시
        self.log_text = tk.Text(self.root, height=15, width=50)
        self.log_text.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)
        
        self.add_log("GUI 시스템 시작됨")

    def add_log(self, message):
        """로그 추가"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)

    def confirmation_request_callback(self, msg: String):
        """배달 확인 요청 수신"""
        try:
            # 메시지 파싱: "업무ID|사용자ID|로봇명|목적지|물품명|주문시간|배달완료시간"
            parts = msg.data.split('|')
            
            if len(parts) >= 7:
                order_id = parts[0]
                user_id = parts[1]
                robot_name = parts[2]
                destination = parts[3]
                item = parts[4]
                order_time = parts[5]
                delivery_time = parts[6]
                
                # 해당 사용자의 요청인지 확인
                if user_id == self.user_id:
                    self.current_task_info = {
                        'order_id': order_id,
                        'user_id': user_id,
                        'robot_name': robot_name,
                        'destination': destination,
                        'item': item,
                        'order_time': order_time,
                        'delivery_time': delivery_time
                    }
                    
                    self.add_log(f"배달 완료 알림: {robot_name}이 {destination}에 {item} 배달 완료")
                    
                    # 수령확인 창 표시
                    self.show_confirmation_dialog()
                else:
                    self.add_log(f"다른 사용자({user_id})의 배달 요청 (무시)")
                    
        except Exception as e:
            self.get_logger().error(f"❌ 확인 요청 처리 중 오류: {e}")
            self.add_log(f"오류: {e}")

    def show_confirmation_dialog(self):
        """수령확인 대화상자 표시"""
        if not self.current_task_info:
            return
            
        # 기존 창이 있으면 닫기
        if self.confirmation_window:
            self.confirmation_window.destroy()
            
        # 새 확인 창 생성
        self.confirmation_window = tk.Toplevel(self.root)
        self.confirmation_window.title("배달 수령확인")
        self.confirmation_window.geometry("350x250")
        self.confirmation_window.transient(self.root)
        self.confirmation_window.grab_set()  # 모달 윈도우로 설정
        
        # 창을 화면 중앙에 배치
        self.confirmation_window.update_idletasks()
        x = (self.confirmation_window.winfo_screenwidth() // 2) - (350 // 2)
        y = (self.confirmation_window.winfo_screenheight() // 2) - (250 // 2)
        self.confirmation_window.geometry(f"350x250+{x}+{y}")
        
        # 아이콘 및 제목
        title_label = tk.Label(self.confirmation_window, text="🤖 배달 완료!", 
                              font=("Arial", 16, "bold"), fg="blue")
        title_label.pack(pady=10)
        
        # 메인 질문 - 사용자가 원하는 형태
        question_text = f"{self.current_task_info['item']}을(를) 받았습니까?"
        self.question_label = tk.Label(self.confirmation_window, text=question_text, 
                                 font=("Arial", 14, "bold"), fg="red")
        self.question_label.pack(pady=10)
        
        # 배달 정보 표시
        info_frame = tk.Frame(self.confirmation_window)
        info_frame.pack(pady=10)
        
        info_text = f"""
주문번호: {self.current_task_info['order_id']}
배달지: {self.current_task_info['destination']}
로봇: {self.current_task_info['robot_name']}
배달시간: {time.strftime('%H:%M:%S', time.localtime(float(self.current_task_info['delivery_time'])))}
        """
        
        info_label = tk.Label(info_frame, text=info_text, font=("Arial", 10), justify=tk.LEFT)
        info_label.pack()
        
        # 버튼 프레임
        button_frame = tk.Frame(self.confirmation_window)
        button_frame.pack(pady=20)
        
        # YES 버튼 (초록색)
        self.yes_button = tk.Button(button_frame, text="예 (Y)", font=("Arial", 12, "bold"),
                              bg="lightgreen", fg="darkgreen", width=8, height=2,
                              command=lambda: self.send_response("YES"))
        self.yes_button.pack(side=tk.LEFT, padx=10)
        
        # NO 버튼 (빨간색)
        self.no_button = tk.Button(button_frame, text="아니오 (N)", font=("Arial", 12, "bold"),
                             bg="lightcoral", fg="darkred", width=8, height=2,
                             command=lambda: self.send_response("NO"))
        self.no_button.pack(side=tk.LEFT, padx=10)
        
        # 키보드 단축키 설정
        self.confirmation_window.bind('<Return>', lambda e: self.send_response("YES"))  # Enter = YES
        self.confirmation_window.bind('<y>', lambda e: self.send_response("YES"))
        self.confirmation_window.bind('<Y>', lambda e: self.send_response("YES"))
        self.confirmation_window.bind('<n>', lambda e: self.send_response("NO"))
        self.confirmation_window.bind('<N>', lambda e: self.send_response("NO"))
        
        # 포커스 설정
        self.confirmation_window.focus_set()
        
        # 5분 타임아웃 설정
        self.confirmation_window.after(300000, self.timeout_response)  # 5분 = 300,000ms
        
        self.add_log("수령확인 창 표시됨")

    def send_response(self, response):
        """응답 전송"""
        if not self.current_task_info:
            return
            
        try:
            # 응답 메시지 생성: "업무ID|사용자ID|YES/NO"
            response_msg = String()
            response_msg.data = f"{self.current_task_info['order_id']}|{self.current_task_info['user_id']}|{response}"
            self.response_pub.publish(response_msg)
            self.add_log(f"응답 전송: {response}")

            if response == "YES":
                self.add_log("✅ 수령 완료로 처리됨")
                messagebox.showinfo("완료", f"{self.current_task_info['item']} 수령이 확인되었습니다!")
                
                # 확인 창 닫기
                if self.confirmation_window:
                    self.confirmation_window.destroy()
                    self.confirmation_window = None
                    
                self.current_task_info = None
                self.status_label.config(text="배달 요청 대기 중...")

            elif response == "NO":
                self.add_log("⏳ 수령 거부 - 로봇이 대기 상태로 전환됨")
                self.question_label.config(text="로봇 대기 중. 수령 후 '예'를 눌러주세요.", fg="orange")
                self.no_button.config(state=tk.DISABLED)

        except Exception as e:
            self.get_logger().error(f"❌ 응답 전송 중 오류: {e}")
            self.add_log(f"응답 전송 오류: {e}")

    def timeout_response(self):
        """타임아웃 처리"""
        if self.confirmation_window:
            self.add_log("⏰ 5분 타임아웃 - 자동으로 NO 응답")
            self.send_response("NO")

    def run(self):
        """GUI 실행"""
        def ros_spin():
            rclpy.spin(self)
            
        # ROS spin을 별도 스레드에서 실행
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
        
        # GUI 메인루프 실행
        self.root.mainloop()

def main():
    """메인 함수"""
    rclpy.init()
    
    try:
        gui = DeliveryConfirmationGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()