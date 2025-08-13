#!/usr/bin/env python3
# admin_gui.py

"""
ROSA 시스템 관리자용 GUI
- 맵에 로봇 위치 및 장소 상태 표시
- 로봇 상태 모니터링 
- 최근 작업 로그 표시
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import numpy as np
from PIL import Image, ImageTk
import os
from config import LOCATIONS, ROBOT_NAMES, ROBOT_CHARGE_STATIONS

class AdminGUI(Node):
    def __init__(self):
        super().__init__('admin_gui')
        
        # ROS 구독
        self.create_subscription(String, '/rosa/status_log', self.status_log_callback, 50)
        self.create_subscription(String, '/rosa/location_status_update', self.location_status_callback, 10)
        self.create_subscription(String, '/rosa/db_log', self.db_log_callback, 50)
        
        # 상태 저장
        self.robot_states = {name: {'state': 'UNKNOWN', 'location': '위치불명', 'battery': 100.0, 'task': '대기중'} 
                           for name in ROBOT_NAMES + ['robot_arm']}
        self.location_states = {location: 'available' for location in LOCATIONS.keys()}
        self.recent_logs = []
        self.robot_markers = {}  # 로봇 마커 저장
        
        # GUI 설정
        self.setup_gui()
        
        # 맵 정보
        self.map_file = "/home/addinedu/map_1753257471.pgm"
        self.map_yaml = "/home/addinedu/map_1753257471.yaml"
        self.map_resolution = 0.05  # yaml에서 가져옴
        self.map_origin = [-0.683, -1.28, 0]  # yaml에서 가져옴
        
        self.load_map()
        self.get_logger().info("✅ 관리자 GUI 시작")

    def setup_gui(self):
        """GUI 창 설정"""
        self.root = tk.Tk()
        self.root.title("ROSA 관리자 모니터링")
        self.root.geometry("1500x800")
        
        # 메인 프레임 구성
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 왼쪽: 맵 + 로봇 상태
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 맵 영역
        self.setup_map_area(left_frame)
        
        # 로봇 상태 영역
        self.setup_robot_status_area(left_frame)
        
        # 오른쪽: 로그 영역
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(10,0))
        
        self.setup_log_area(right_frame)

    def setup_map_area(self, parent):
        """맵 표시 영역"""
        map_frame = ttk.LabelFrame(parent, text="🗺️ 시설 맵 & 실시간 상태")
        map_frame.pack(fill=tk.BOTH, expand=True, pady=(0,5))
        
        # 맵 캔버스 (크기 증가)
        self.map_canvas = tk.Canvas(map_frame, bg='white', width=700, height=500)
        self.map_canvas.pack(padx=10, pady=5)
        
        # 범례
        legend_frame = ttk.Frame(map_frame)
        legend_frame.pack(fill=tk.X, padx=10, pady=(0,10))
        
        ttk.Label(legend_frame, text="🟢 비어있음").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🟡 예약중").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🔵 사용중").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🔴 비상상황").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🤖 로봇위치").pack(side=tk.LEFT, padx=5)

    def setup_robot_status_area(self, parent):
        """로봇 상태 표시 영역"""
        status_frame = ttk.LabelFrame(parent, text="🤖 로봇 상태 모니터링")
        status_frame.pack(fill=tk.X, pady=(0,5))
        
        # 로봇별 상태 표시
        self.robot_status_labels = {}
        for i, robot_name in enumerate(ROBOT_NAMES + ['robot_arm']):
            frame = ttk.Frame(status_frame)
            frame.pack(fill=tk.X, padx=10, pady=5)
            
            # 로봇 이름
            name_label = ttk.Label(frame, text=f"{robot_name}:", font=("Arial", 10, "bold"), width=12)
            name_label.pack(side=tk.LEFT)
            
            # 상태 표시
            status_label = ttk.Label(frame, text="연결 대기 중...", foreground="gray")
            status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            self.robot_status_labels[robot_name] = status_label

    def setup_log_area(self, parent):
        """로그 표시 영역"""
        # 상태 로그
        status_log_frame = ttk.LabelFrame(parent, text="⚠️ 시스템 알림 (오류/경고)")
        status_log_frame.pack(fill=tk.BOTH, expand=True, pady=(0,10))
        
        self.status_log_text = scrolledtext.ScrolledText(status_log_frame, height=15, width=50)
        self.status_log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # DB 작업 로그
        db_log_frame = ttk.LabelFrame(parent, text="📋 최근 작업 완료 기록")
        db_log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.db_log_text = scrolledtext.ScrolledText(db_log_frame, height=10, width=50)
        self.db_log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def load_map(self):
        """맵 이미지 로드"""
        try:
            if os.path.exists(self.map_file):
                # PGM 파일 로드
                map_image = Image.open(self.map_file)
                # 크기 조정 (700x500 캔버스에 맞게)
                map_image = map_image.resize((700, 500), Image.Resampling.LANCZOS)
                self.map_photo = ImageTk.PhotoImage(map_image)
                
                self.map_canvas.create_image(350, 250, image=self.map_photo)
                self.get_logger().info("✅ 맵 이미지 로드 완료")
            else:
                self.get_logger().warn("⚠️ 맵 파일을 찾을 수 없습니다. 기본 맵을 표시합니다.")
                self.draw_default_map()
        except Exception as e:
            self.get_logger().error(f"❌ 맵 로드 실패: {e}")
            self.draw_default_map()
        
        # 장소 마커 그리기
        self.draw_location_markers()
        
        # 로봇 초기 위치 설정 및 표시
        self.init_robot_positions()
        self.draw_robot_markers()
        
        # 초기 상태 표시
        self.update_all_robot_displays()
        self.update_all_location_displays()

    def draw_default_map(self):
        """기본 맵 그리기 (맵 파일이 없을 때)"""
        # 배경
        self.map_canvas.create_rectangle(50, 50, 550, 350, fill='lightgray', outline='black')
        
        # 방들 그리기
        rooms = {
            '왼쪽방': (100, 80, 200, 150),
            '오른쪽방': (250, 80, 350, 150),
            '면회실': (400, 180, 500, 250),
            '출입구': (400, 280, 500, 320),
        }
        
        for room_name, (x1, y1, x2, y2) in rooms.items():
            self.map_canvas.create_rectangle(x1, y1, x2, y2, fill='white', outline='black')
            self.map_canvas.create_text((x1+x2)/2, (y1+y2)/2, text=room_name, font=("Arial", 8))

    def world_to_canvas(self, x, y):
        """실제 좌표를 캔버스 좌표로 변환"""
        # 맵 origin과 resolution을 고려한 좌표 변환
        map_x = (x - self.map_origin[0]) / self.map_resolution
        map_y = (y - self.map_origin[1]) / self.map_resolution
        
        # 캔버스 크기에 맞게 스케일링 (700x500)
        # 실제 맵 크기를 추정해서 적절히 스케일링
        canvas_x = int(350 + map_x * 300)  # 중앙(350) 기준으로 스케일링
        canvas_y = int(250 - map_y * 200)  # 중앙(250) 기준, Y축 뒤집기
        
        return max(20, min(680, canvas_x)), max(20, min(480, canvas_y))

    def draw_location_markers(self):
        """장소 마커 그리기"""
        self.location_markers = {}
        
        for location, (x, y) in LOCATIONS.items():
            canvas_x, canvas_y = self.world_to_canvas(x, y)
            
            # 상태에 따른 색상 (기본은 초록색)
            color = self.get_status_color('available')
            
            # 원형 마커
            marker = self.map_canvas.create_oval(
                canvas_x-15, canvas_y-15, canvas_x+15, canvas_y+15,
                fill=color, outline='black', width=2
            )
            
            # 텍스트 라벨  
            text = self.map_canvas.create_text(
                canvas_x, canvas_y+25, text=location, 
                font=("Arial", 8), fill='black'
            )
            
            self.location_markers[location] = {'marker': marker, 'text': text}

    def init_robot_positions(self):
        """로봇 초기 위치 설정"""
        # 초기 위치는 각각의 충전소로 설정
        for robot_name in ROBOT_NAMES:
            charge_station = ROBOT_CHARGE_STATIONS.get(robot_name)
            if charge_station and charge_station in LOCATIONS:
                self.robot_states[robot_name]['location'] = charge_station
            
    def draw_robot_markers(self):
        """로봇 마커 그리기"""
        robot_colors = {
            'DP_03': '#FF6B6B',  # 빨간색
            'DP_08': '#4ECDC4',  # 청록색  
            'DP_09': '#45B7D1'   # 파란색
        }
        
        for robot_name in ROBOT_NAMES:
            robot_info = self.robot_states[robot_name]
            location = robot_info['location']
            
            if location in LOCATIONS:
                x, y = LOCATIONS[location]
                canvas_x, canvas_y = self.world_to_canvas(x, y)
                
                color = robot_colors.get(robot_name, '#888888')
                
                # 로봇 마커 (사각형)
                marker = self.map_canvas.create_rectangle(
                    canvas_x-8, canvas_y-8, canvas_x+8, canvas_y+8,
                    fill=color, outline='black', width=2
                )
                
                # 로봇 이름 라벨
                text = self.map_canvas.create_text(
                    canvas_x, canvas_y-20, text=robot_name, 
                    font=("Arial", 8, "bold"), fill=color
                )
                
                self.robot_markers[robot_name] = {'marker': marker, 'text': text}

    def update_robot_position(self, robot_name, new_location):
        """로봇 위치 업데이트"""
        if robot_name not in ROBOT_NAMES or new_location not in LOCATIONS:
            return
            
        # 로봇 상태 업데이트
        self.robot_states[robot_name]['location'] = new_location
        
        # 기존 마커 제거
        if robot_name in self.robot_markers:
            self.map_canvas.delete(self.robot_markers[robot_name]['marker'])
            self.map_canvas.delete(self.robot_markers[robot_name]['text'])
        
        # 새 위치에 마커 그리기
        x, y = LOCATIONS[new_location]
        canvas_x, canvas_y = self.world_to_canvas(x, y)
        
        robot_colors = {
            'DP_03': '#FF6B6B',  
            'DP_08': '#4ECDC4',   
            'DP_09': '#45B7D1'   
        }
        color = robot_colors.get(robot_name, '#888888')
        
        marker = self.map_canvas.create_rectangle(
            canvas_x-8, canvas_y-8, canvas_x+8, canvas_y+8,
            fill=color, outline='black', width=2
        )
        
        text = self.map_canvas.create_text(
            canvas_x, canvas_y-20, text=robot_name, 
            font=("Arial", 8, "bold"), fill=color
        )
        
        self.robot_markers[robot_name] = {'marker': marker, 'text': text}

    def update_all_robot_displays(self):
        """모든 로봇 상태 표시 업데이트"""
        for robot_name in ROBOT_NAMES + ['robot_arm']:
            self.update_robot_status_display(robot_name)
            
    def update_all_location_displays(self):
        """모든 장소 상태 표시 업데이트"""
        for location, status in self.location_states.items():
            self.update_location_marker(location, status)

    def get_status_color(self, status):
        """상태에 따른 색상 반환"""
        colors = {
            'available': '#90EE90',    # 연한 초록 (비어있음)
            'reserved': '#FFD700',     # 금색 (예약중)
            'busy': '#87CEEB',         # 하늘색 (사용중)
            'emergency': '#FF6B6B'     # 빨간색 (비상상황)
        }
        return colors.get(status, '#D3D3D3')  # 기본은 회색

    def update_location_marker(self, location, status):
        """장소 마커 색상 업데이트"""
        if location in self.location_markers:
            color = self.get_status_color(status)
            self.map_canvas.itemconfig(self.location_markers[location]['marker'], fill=color)

    def status_log_callback(self, msg: String):
        """상태 로그 콜백"""
        try:
            # 메시지 파싱: "robot_name|status|reason"
            parts = msg.data.split('|', 2)
            if len(parts) >= 3:
                robot_name, status, reason = parts
                
                # 로봇 상태 업데이트
                if robot_name in self.robot_states:
                    # 상태 정보 파싱 및 업데이트
                    self.update_robot_status_from_log(robot_name, status, reason)
                
                # 시스템 문제만 실시간 상태 로그에 표시
                if any(keyword in reason.lower() for keyword in ['오류', '실패', '타임아웃', '응답 없음', '통신', '연결 실패', '비상정지']):
                    timestamp = time.strftime("%H:%M:%S")
                    log_entry = f"[{timestamp}] ⚠️ {robot_name}: {reason}\n"
                    
                    self.status_log_text.insert(tk.END, log_entry)
                    self.status_log_text.see(tk.END)
                    
                    # 최대 500줄 유지
                    lines = self.status_log_text.get("1.0", tk.END).count('\n')
                    if lines > 500:
                        self.status_log_text.delete("1.0", "100.0")
                    
        except Exception as e:
            self.get_logger().error(f"❌ 상태 로그 처리 오류: {e}")

    def update_robot_status_from_log(self, robot_name, status, reason):
        """로그에서 로봇 상태 업데이트"""
        if robot_name not in self.robot_states:
            return
            
        robot_info = self.robot_states[robot_name]
        
        # 상태 변화 추적
        if "→" in status:
            # 상태 변화: "IDLE → MOVING_TO_PICKUP"
            new_state = status.split("→")[-1].strip()
            robot_info['state'] = new_state
        else:
            robot_info['state'] = status
            
        # 위치 및 작업 정보 추출
        old_location = robot_info['location']
        new_location = None
        
        if "이동" in reason:
            if "픽업대로" in reason:
                robot_info['task'] = "픽업대로 이동중"
                new_location = "픽업대"
            elif "충전소" in reason:
                robot_info['task'] = "충전소로 복귀중"
                new_location = ROBOT_CHARGE_STATIONS.get(robot_name)
            else:
                for location in LOCATIONS.keys():
                    if location in reason:
                        robot_info['task'] = f"{location}로 이동중"
                        new_location = location
                        break
        elif "픽업" in reason:
            robot_info['task'] = "물품 픽업중"
            if "완료" in reason:
                new_location = "픽업대"
        elif "배달" in reason:
            robot_info['task'] = "배달 진행중"
        elif "확인" in reason:
            robot_info['task'] = "수령확인 대기중"
        elif "충전" in reason:
            robot_info['task'] = "충전중"
            new_location = ROBOT_CHARGE_STATIONS.get(robot_name, "충전소")
        elif "대기" in reason:
            robot_info['task'] = "대기중"
        elif "도착" in reason:
            # 도착한 경우 위치 업데이트
            for location in LOCATIONS.keys():
                if location in reason:
                    new_location = location
                    break
        
        # 위치가 변경된 경우 맵에서 로봇 위치 업데이트
        if new_location and new_location != old_location and robot_name in ROBOT_NAMES:
            robot_info['location'] = new_location
            self.update_robot_position(robot_name, new_location)
            
        # GUI 업데이트
        self.update_robot_status_display(robot_name)

    def update_robot_status_display(self, robot_name):
        """로봇 상태 표시 업데이트"""
        if robot_name in self.robot_status_labels:
            robot_info = self.robot_states[robot_name]
            
            if robot_name == 'robot_arm':
                status_text = f"상태: {robot_info['state']} | 작업: {robot_info['task']}"
            else:
                status_text = (f"🔋{robot_info['battery']:.1f}% | "
                             f"📍{robot_info['location']} | "
                             f"⚙️{robot_info['state']} | "
                             f"📋{robot_info['task']}")
            
            self.robot_status_labels[robot_name].config(text=status_text)

    def location_status_callback(self, msg: String):
        """장소 상태 업데이트 콜백"""
        try:
            # 메시지 파싱: "location|status|timestamp"
            parts = msg.data.split('|')
            if len(parts) >= 2:
                location, status = parts[0], parts[1]
                self.location_states[location] = status
                self.update_location_marker(location, status)
                
        except Exception as e:
            self.get_logger().error(f"❌ 장소 상태 업데이트 오류: {e}")

    def db_log_callback(self, msg: String):
        """DB 로그 콜백"""
        try:
            if msg.data.startswith("DB_RECORD|"):
                # DB 기록 메시지 파싱
                record_data = msg.data[10:]  # "DB_RECORD|" 제거
                
                # 파싱하여 읽기 쉽게 표시
                fields = record_data.split('|')
                parsed_info = {}
                for field in fields:
                    if '=' in field:
                        key, value = field.split('=', 1)
                        parsed_info[key] = value
                
                timestamp = time.strftime("%H:%M:%S")
                
                # 작업 완료 정보를 보기 좋게 포맷팅
                robot = parsed_info.get('ROBOT', '?')
                item = parsed_info.get('ITEM', '?')
                destination = parsed_info.get('DESTINATION', '?')
                status = parsed_info.get('STATUS', '?')
                delivery_time = parsed_info.get('DELIVERY_TIME', '?')
                
                log_entry = f"[{timestamp}] ✅ {robot}: {item} → {destination} ({status})\n"
                
                self.db_log_text.insert(tk.END, log_entry)
                self.db_log_text.see(tk.END)
                
                # 최대 300줄 유지
                lines = self.db_log_text.get("1.0", tk.END).count('\n')
                if lines > 300:
                    self.db_log_text.delete("1.0", "50.0")
                    
        except Exception as e:
            self.get_logger().error(f"❌ DB 로그 처리 오류: {e}")

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
        admin_gui = AdminGUI()
        admin_gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()