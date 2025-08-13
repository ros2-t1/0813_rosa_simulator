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
        self.create_subscription(String, '/rosa/robot_position_update', self.robot_position_callback, 10)
        self.create_subscription(String, '/rosa/robot_realtime_pose', self.realtime_pose_callback, 10)  # 🗺️ 실시간 좌표
        self.create_subscription(String, '/rosa/db_log', self.db_log_callback, 50)
        self.create_subscription(String, '/rosa/task_status_update', self.task_status_callback, 10)

        # 상태 저장
        self.robot_states = {name: {'state': 'UNKNOWN', 'location': '위치불명', 'battery': 100.0, 'task': '대기중'}
                           for name in ROBOT_NAMES + ['robot_arm']}
        self.location_states = {location: 'available' for location in LOCATIONS.keys()}
        self.robot_markers = {}  # 로봇 마커 저장
        self.robot_realtime_poses = {}  # 🗺️ 로봇 실시간 좌표 (x, y) 저장
        self.realtime_markers = {}  # 실시간 위치 마커

        # 업무 대기열 상태
        self.task_queue_data = {
            'pending_tasks': [],
            'assigned_tasks': {},
            'completed_tasks': []
        }

        # TaskManager 참조를 위한 구독 추가
        self.create_subscription(String, '/rosa/task_queue_update', self.task_queue_callback, 10)

        # GUI 설정
        self.setup_gui()

        # 맵 정보
        self.map_file = "/home/addinedu/map_1753257471.pgm"
        self.map_yaml = "/home/addinedu/map_1753257471.yaml"
        self.map_resolution = 0.05
        self.map_origin = [-0.683, -1.28, 0]
        self.map_image = None
        self.map_photo = None

        self.load_map()
        self.get_logger().info("✅ 관리자 GUI 시작")

    def setup_gui(self):
        """GUI 창 설정"""
        self.root = tk.Tk()
        self.root.title("ROSA 관리자 모니터링")
        self.root.geometry("1500x800")

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.setup_map_area(left_frame)
        self.setup_task_queue_area(left_frame)
        self.setup_robot_status_area(left_frame)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(10,0))

        self.setup_log_area(right_frame)

    def setup_map_area(self, parent):
        """맵 표시 영역"""
        map_frame = ttk.LabelFrame(parent, text="🗺️ 시설 맵 & 실시간 상태")
        map_frame.pack(fill=tk.BOTH, expand=True, pady=(0,5))

        self.map_canvas = tk.Canvas(map_frame, bg='white', width=700, height=500)
        self.map_canvas.pack(padx=10, pady=5)

        legend_frame = ttk.Frame(map_frame)
        legend_frame.pack(fill=tk.X, padx=10, pady=(0,10))

        ttk.Label(legend_frame, text="🔵 비어있음").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🟢 예약중").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🟡 작업중").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🔴 비상상황").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🤖 로봇위치(장소)").pack(side=tk.LEFT, padx=5)
        ttk.Label(legend_frame, text="🗺️ 실시간좌표", foreground="red").pack(side=tk.LEFT, padx=5)


    def setup_task_queue_area(self, parent):
        """업무 대기열 표시 영역"""
        queue_frame = ttk.LabelFrame(parent, text="📋 업무 대기열 & 진행 상황")
        queue_frame.pack(fill=tk.X, pady=5)

        task_columns = ('작업순위', '작업상태', '작업ID', '로봇', '목적지', '물품')
        self.task_tree = ttk.Treeview(queue_frame, columns=task_columns, show='headings', height=6)

        self.task_tree.heading('작업순위', text='작업순위')
        self.task_tree.heading('작업상태', text='작업상태')
        self.task_tree.heading('작업ID', text='작업ID')
        self.task_tree.heading('로봇', text='로봇')
        self.task_tree.heading('목적지', text='목적지')
        self.task_tree.heading('물품', text='물품')

        self.task_tree.column('작업순위', width=80, anchor='center')
        self.task_tree.column('작업상태', width=100, anchor='center')
        self.task_tree.column('작업ID', width=120, anchor='center')
        self.task_tree.column('로봇', width=80, anchor='center')
        self.task_tree.column('목적지', width=100, anchor='center')
        self.task_tree.column('물품', width=80, anchor='center')

        task_scrollbar = ttk.Scrollbar(queue_frame, orient=tk.VERTICAL, command=self.task_tree.yview)
        self.task_tree.configure(yscrollcommand=task_scrollbar.set)

        self.task_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        task_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)


    def setup_robot_status_area(self, parent):
        """로봇 상태 표시 영역"""
        status_frame = ttk.LabelFrame(parent, text="🤖 로봇 상태 모니터링")
        status_frame.pack(fill=tk.X, pady=(0,5))

        self.robot_status_labels = {}
        for i, robot_name in enumerate(ROBOT_NAMES + ['robot_arm']):
            frame = ttk.Frame(status_frame)
            frame.pack(fill=tk.X, padx=10, pady=5)

            name_label = ttk.Label(frame, text=f"{robot_name}:", font=("Arial", 10, "bold"), width=12)
            name_label.pack(side=tk.LEFT)

            status_label = ttk.Label(frame, text="연결 대기 중...", foreground="gray")
            status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

            self.robot_status_labels[robot_name] = status_label

    def setup_log_area(self, parent):
        """로그 표시 영역"""
        status_log_frame = ttk.LabelFrame(parent, text="⚠️ 시스템 알림 (오류/경고)")
        status_log_frame.pack(fill=tk.BOTH, expand=True, pady=(0,10))

        self.status_log_text = scrolledtext.ScrolledText(status_log_frame, height=15, width=50)
        self.status_log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        db_log_frame = ttk.LabelFrame(parent, text="📋 최근 작업 완료 기록")
        db_log_frame.pack(fill=tk.BOTH, expand=True)

        self.db_log_text = scrolledtext.ScrolledText(db_log_frame, height=10, width=50)
        self.db_log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def load_map(self):
        """맵 이미지 로드 (원본 비율 유지, 회전 없음)"""
        try:
            if os.path.exists(self.map_file):
                self.map_image = Image.open(self.map_file)
                
                # 맵 이미지 회전 제거!
                
                # 원본 비율을 유지하며 캔버스 너비에 맞게 리사이즈
                original_width, original_height = self.map_image.size
                target_width = 700 # 캔버스 너비와 일치
                aspect_ratio = original_height / original_width
                target_height = int(target_width * aspect_ratio)

                resized_image = self.map_image.resize((target_width, target_height), Image.Resampling.LANCZOS)
                self.map_photo = ImageTk.PhotoImage(resized_image)

                self.map_canvas.config(width=target_width, height=target_height)
                self.map_canvas.create_image(target_width / 2, target_height / 2, image=self.map_photo)

                self.get_logger().info(f"✅ 맵 이미지 로드 완료 (회전 없음, 비율유지: {target_width}x{target_height})")
            else:
                self.get_logger().warn("⚠️ 맵 파일을 찾을 수 없습니다. 기본 맵을 표시합니다.")
                self.draw_default_map()
        except Exception as e:
            self.get_logger().error(f"❌ 맵 로드 실패: {e}")
            self.draw_default_map()

        self.draw_location_markers()
        self.init_robot_positions()
        self.draw_robot_markers()
        self.update_all_robot_displays()
        self.update_all_location_displays()

    def draw_default_map(self):
        """기본 맵 그리기 (맵 파일이 없을 때)"""
        self.map_canvas.create_rectangle(50, 50, 550, 350, fill='lightgray', outline='black')
        rooms = {
            '왼쪽방': (100, 80, 200, 150), '오른쪽방': (250, 80, 350, 150),
            '면회실': (400, 180, 500, 250), '출입구': (400, 280, 500, 320),
        }
        for room_name, (x1, y1, x2, y2) in rooms.items():
            self.map_canvas.create_rectangle(x1, y1, x2, y2, fill='white', outline='black')
            self.map_canvas.create_text((x1+x2)/2, (y1+y2)/2, text=room_name, font=("Arial", 8))

    def world_to_canvas(self, x, y, location_name="Unknown"):
        """[최종 테스트] 실제 world 좌표를 '회전 없이' 캔버스로 변환"""
        print(f"\n--- [{location_name}] 좌표 변환 테스트 (회전 없음) ---")
        print(f"  입력 월드 좌표: x={x}, y={y}")

        if not self.map_image:
            return (x * 100 + 300), (400 - y * 100)

        # --- 회전 로직 없이 원래 좌표를 그대로 사용 ---
        original_world_x = x
        original_world_y = y

        # --- 원본 좌표를 픽셀로 변환 ---
        original_width = self.map_image.width
        original_height = self.map_image.height
        map_pixel_x = (original_world_x - self.map_origin[0]) / self.map_resolution
        map_pixel_y = original_height - ((original_world_y - self.map_origin[1]) / self.map_resolution)
        
        # --- 픽셀 좌표를 캔버스 좌표로 스케일링 ---
        canvas_width = self.map_canvas.winfo_width()
        canvas_height = self.map_canvas.winfo_height()

        if canvas_width <= 1: canvas_width = 700
        if canvas_height <= 1: canvas_height = int((original_height / original_width) * canvas_width)
        
        canvas_x = (map_pixel_x / original_width) * canvas_width
        canvas_y = (map_pixel_y / original_height) * canvas_height
        
        print(f"  최종 캔버스 좌표 (회전 없음): canvas_x={canvas_x:.3f}, canvas_y={canvas_y:.3f}")

        return canvas_x, canvas_y

    def draw_location_markers(self):
        """장소 마커 그리기"""
        self.location_markers = {}
        for location, (x, y) in LOCATIONS.items():
            # 호출 시 불필요한 location 인자 제거
            canvas_x, canvas_y = self.world_to_canvas(x, y)
            
            color = self.get_status_color('available')
            marker = self.map_canvas.create_oval(
                canvas_x-15, canvas_y-15, canvas_x+15, canvas_y+15,
                fill=color, outline='black', width=2
            )
            text = self.map_canvas.create_text(
                canvas_x, canvas_y+25, text=location,
                font=("Arial", 8), fill='black'
            )
            self.location_markers[location] = {'marker': marker, 'text': text}

    def init_robot_positions(self):
        """로봇 초기 위치 설정"""
        for robot_name in ROBOT_NAMES:
            charge_station = ROBOT_CHARGE_STATIONS.get(robot_name)
            if charge_station and charge_station in LOCATIONS:
                self.robot_states[robot_name]['location'] = charge_station

    def draw_robot_markers(self):
        """로봇 마커 그리기"""
        for robot_name in ROBOT_NAMES:
            robot_info = self.robot_states[robot_name]
            location = robot_info['location']
            if location in LOCATIONS:
                x, y = LOCATIONS[location]
                canvas_x, canvas_y = self.world_to_canvas(x, y)
                color = self.get_robot_status_color(robot_info['state'])
                marker = self.map_canvas.create_rectangle(
                    canvas_x-8, canvas_y-8, canvas_x+8, canvas_y+8,
                    fill=color, outline='black', width=2
                )
                text = self.map_canvas.create_text(
                    canvas_x, canvas_y-20, text=robot_name,
                    font=("Arial", 8, "bold"), fill='#333333'
                )
                self.robot_markers[robot_name] = {'marker': marker, 'text': text}

    def update_robot_position(self, robot_name, new_location):
        """로봇 위치 및 색상 업데이트 (효율적인 방식) - 장소 기반"""
        if robot_name not in ROBOT_NAMES or new_location not in LOCATIONS:
            return

        robot_info = self.robot_states[robot_name]
        robot_info['location'] = new_location

        x, y = LOCATIONS[new_location]
        canvas_x, canvas_y = self.world_to_canvas(x, y)
        color = self.get_robot_status_color(robot_info['state'])

        if robot_name in self.robot_markers:
            self.map_canvas.coords(self.robot_markers[robot_name]['marker'],
                                 canvas_x-8, canvas_y-8, canvas_x+8, canvas_y+8)
            self.map_canvas.coords(self.robot_markers[robot_name]['text'],
                                 canvas_x, canvas_y-20)
            self.map_canvas.itemconfig(self.robot_markers[robot_name]['marker'], fill=color)
        else:
            self.draw_robot_markers() # 마커가 없으면 다시 그리기

    def update_realtime_robot_marker(self, robot_name, world_x, world_y):
        """🗺️ 실시간 로봇 마커 업데이트 (amcl_pose 기반)"""
        if robot_name not in ROBOT_NAMES:
            return
            
        try:
            # world 좌표를 canvas 좌표로 변환
            canvas_x, canvas_y = self.world_to_canvas(world_x, world_y, robot_name)
            
            robot_info = self.robot_states[robot_name]
            color = self.get_robot_status_color(robot_info['state'])
            
            # 실시간 마커 업데이트 또는 생성
            if robot_name in self.realtime_markers:
                # 기존 마커 업데이트
                self.map_canvas.coords(self.realtime_markers[robot_name]['marker'],
                                     canvas_x-10, canvas_y-10, canvas_x+10, canvas_y+10)
                self.map_canvas.coords(self.realtime_markers[robot_name]['text'],
                                     canvas_x, canvas_y-25)
                self.map_canvas.itemconfig(self.realtime_markers[robot_name]['marker'], fill=color)
            else:
                # 새 마커 생성 (원형으로 구별)
                marker = self.map_canvas.create_oval(
                    canvas_x-10, canvas_y-10, canvas_x+10, canvas_y+10,
                    fill=color, outline='red', width=3  # 빨간 테두리로 실시간 위치 강조
                )
                text = self.map_canvas.create_text(
                    canvas_x, canvas_y-25, text=f"{robot_name}\n({world_x:.1f},{world_y:.1f})",
                    font=("Arial", 7, "bold"), fill='red'
                )
                self.realtime_markers[robot_name] = {'marker': marker, 'text': text}
            
            # 기존 정적 마커는 연하게 표시 또는 숨기기
            if robot_name in self.robot_markers:
                self.map_canvas.itemconfig(self.robot_markers[robot_name]['marker'], 
                                         outline='gray', width=1)  # 연하게 만들기
                
        except Exception as e:
            self.get_logger().error(f"❌ 실시간 마커 업데이트 오류: {e}")

    def get_status_color(self, status):
        """장소 상태에 따른 색상 반환"""
        colors = {
            'available': '#87CEEB', 'reserved': '#90EE90',
            'busy': '#FFD700', 'emergency': '#FF6B6B'
        }
        return colors.get(status, '#D3D3D3')

    def get_robot_status_color(self, state):
        """로봇 상태에 따른 색상 반환"""
        if state in ['CHARGING', 'RETURNING', 'OFF_DUTY']: return '#45B7D1'
        elif state in ['IDLE', 'WAITING', 'AWAITING_PICKUP_RESERVATION', 'MOVING_TO_PICKUP_WAIT',
                       'WAITING_AT_PICKUP_QUEUE', 'AWAITING_DEST_RESERVATION', 'MOVING_TO_DEST',
                       'AWAITING_CONFIRMATION']: return '#4ECDC4'
        elif state in ['PICKING_UP', 'DELIVERING']: return '#FFD700'
        elif state == 'EMERGENCY_STOP': return '#FF6B6B'
        else: return '#888888'

    def update_location_marker(self, location, status):
        """장소 마커 색상 업데이트"""
        if location in self.location_markers:
            color = self.get_status_color(status)
            self.map_canvas.itemconfig(self.location_markers[location]['marker'], fill=color)

    def robot_position_callback(self, msg: String):
        """로봇 위치 업데이트 콜백 (장소 기반)"""
        try:
            robot_name, new_location = msg.data.split('|')
            self.update_robot_position(robot_name, new_location)
        except Exception as e:
            self.get_logger().error(f"❌ 로봇 위치 업데이트 오류: {e}")

    def realtime_pose_callback(self, msg: String):
        """🗺️ 실시간 로봇 좌표 콜백"""
        try:
            parts = msg.data.split('|')
            if len(parts) >= 3:
                robot_name, x_str, y_str = parts[:3]
                x, y = float(x_str), float(y_str)
                
                # 실시간 좌표 저장
                self.robot_realtime_poses[robot_name] = (x, y)
                
                # 맵에 실시간 위치 업데이트
                self.update_realtime_robot_marker(robot_name, x, y)
                
        except Exception as e:
            self.get_logger().error(f"❌ 실시간 로봇 좌표 처리 오류: {e}")

    def status_log_callback(self, msg: String):
        """상태 로그 콜백"""
        try:
            parts = msg.data.split('|', 2)
            if len(parts) >= 3:
                robot_name, status, reason = parts
                if robot_name in self.robot_states:
                    self.update_robot_status_from_log(robot_name, status, reason)
                if any(keyword in reason.lower() for keyword in ['오류', '실패', '타임아웃', '응답 없음', '통신', '연결 실패', '비상정지']):
                    timestamp = time.strftime("%H:%M:%S")
                    log_entry = f"[{timestamp}] ⚠️ {robot_name}: {reason}\n"
                    self.status_log_text.insert(tk.END, log_entry)
                    self.status_log_text.see(tk.END)
        except Exception as e:
            self.get_logger().error(f"❌ 상태 로그 처리 오류: {e}")

    def update_robot_status_from_log(self, robot_name, status, reason):
        """로그에서 로봇 상태 업데이트"""
        if robot_name not in self.robot_states: return
        robot_info = self.robot_states[robot_name]

        if "→" in status: robot_info['state'] = status.split("→")[-1].strip()
        else: robot_info['state'] = status

        if robot_name == "robot_arm":
            # ... (robot_arm logic is fine)
            self.update_robot_status_display(robot_name)
            return

        self.update_robot_status_display(robot_name)
        if robot_name in self.robot_markers:
            color = self.get_robot_status_color(robot_info['state'])
            self.map_canvas.itemconfig(self.robot_markers[robot_name]['marker'], fill=color)

    def update_robot_status_display(self, robot_name):
        """로봇 상태 표시 업데이트"""
        if robot_name in self.robot_status_labels:
            robot_info = self.robot_states[robot_name]
            if robot_name == 'robot_arm':
                status_text = f"상태: {robot_info['state']} | 작업: {robot_info['task']}"
            else:
                status_text = (f"🔋{robot_info['battery']:.1f}% | 📍{robot_info['location']} | "
                             f"⚙️{robot_info['state']} | 📋{robot_info['task']}")
            self.robot_status_labels[robot_name].config(text=status_text)

    def location_status_callback(self, msg: String):
        """장소 상태 업데이트 콜백"""
        try:
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
                record_data = msg.data[10:]
                fields = record_data.split('|')
                parsed_info = {k: v for f in fields if '=' in f for k, v in [f.split('=', 1)]}
                timestamp = time.strftime("%H:%M:%S")
                log_entry = (f"[{timestamp}] ✅ {parsed_info.get('ROBOT', '?')}: "
                             f"{parsed_info.get('ITEM', '?')} → {parsed_info.get('DESTINATION', '?')} "
                             f"({parsed_info.get('STATUS', '?')})\n")
                self.db_log_text.insert(tk.END, log_entry)
                self.db_log_text.see(tk.END)
        except Exception as e:
            self.get_logger().error(f"❌ DB 로그 처리 오류: {e}")

    def update_task_status_display(self):
        """업무 상태 트리뷰 업데이트"""
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)

        all_tasks = []
        priority_counter = 1

        for task_id, task_info in self.task_queue_data.get('pending_tasks', []):
            all_tasks.append({'priority': priority_counter, 'status': "등록", 'task_id': task_id,
                              'robot': task_info.get('robot_name', '할당전'), 'destination': task_info.get('destination', ''),
                              'item': task_info.get('item', '-'), 'tag': 'pending'})
            priority_counter += 1

        for robot_name, task_info in self.task_queue_data.get('assigned_tasks', {}).items():
            robot_state = self.robot_states.get(robot_name, {}).get('state', 'UNKNOWN')
            status = "작업중" if robot_state in ['MOVING_TO_DEST', 'PICKING_UP', 'AWAITING_CONFIRMATION'] else "배정완료"
            tag = 'working' if status == "작업중" else 'assigned'
            all_tasks.append({'priority': priority_counter, 'status': status, 'task_id': task_info.get('task_id', ''),
                              'robot': robot_name, 'destination': task_info.get('destination', ''),
                              'item': task_info.get('item', '-'), 'tag': tag})
            priority_counter += 1

        for task in all_tasks:
            self.task_tree.insert('', 'end', values=(
                task['priority'], task['status'], task['task_id'], task['robot'],
                task['destination'], task['item']
            ), tags=(task['tag'],))
        
        self.task_tree.tag_configure("pending", foreground="orange")
        self.task_tree.tag_configure("assigned", foreground="blue")
        self.task_tree.tag_configure("working", foreground="green")

    def task_queue_callback(self, msg: String):
        """TaskManager의 TaskQueue 상태 업데이트 콜백"""
        try:
            import json
            queue_status = json.loads(msg.data)
            self.task_queue_data['pending_tasks'] = [
                (task['task_id'], task) for task in queue_status.get('pending_tasks', [])
            ]
            self.task_queue_data['assigned_tasks'] = queue_status.get('assigned_tasks', {})
            self.update_task_status_display()
        except Exception as e:
            self.get_logger().error(f"❌ TaskQueue 상태 업데이트 오류: {e}")

    # The rest of the methods (task_status_callback, handle_task_status_update, etc.) are omitted for brevity but assumed to be present and correct.
    # The main execution block is also assumed to be correct.


    def task_status_callback(self, msg: String):
        """업무 상태 업데이트 콜백"""
        try:
            # 메시지 파싱: "task_id|status|robot_name|destination|item"
            parts = msg.data.split('|')
            if len(parts) >= 4:
                task_id, status, robot_name, destination = parts[:4]
                item = parts[4] if len(parts) > 4 else None
                
                self.handle_task_status_update(task_id, status, robot_name, destination, item)
                
        except Exception as e:
            self.get_logger().error(f"❌ 업무 상태 업데이트 오류: {e}")

    def handle_task_status_update(self, task_id: str, status: str, robot_name: str, 
                                destination: str, item: str = None):
        """업무 상태 업데이트 처리"""
        current_time = time.time()
        
        # 상태별 처리
        if status == 'PENDING':
            # 등록 상태 - pending_tasks에 추가
            task_info = {
                'task_id': task_id,
                'robot_name': robot_name if robot_name != '자동할당' else None,
                'destination': destination,
                'item': item
            }
            
            # 중복 제거
            self.task_queue_data['pending_tasks'] = [
                (tid, tinfo) for tid, tinfo in self.task_queue_data['pending_tasks'] 
                if tid != task_id
            ]
            self.task_queue_data['pending_tasks'].append((task_id, task_info))
            
        elif status == 'ASSIGNED':
            # 배정완료 상태 - assigned_tasks에 추가, pending에서 제거
            self.task_queue_data['pending_tasks'] = [
                (tid, tinfo) for tid, tinfo in self.task_queue_data['pending_tasks'] 
                if tid != task_id
            ]
            
            self.task_queue_data['assigned_tasks'][robot_name] = {
                'task_id': task_id,
                'destination': destination,
                'item': item
            }
            
        elif status == 'IN_PROGRESS':
            # 작업중 상태 - (트리뷰에서는 "작업중"으로 표시되도록 assigned_tasks를 수정)
            if robot_name in self.task_queue_data['assigned_tasks']:
                task_info = self.task_queue_data['assigned_tasks'][robot_name]
                task_info['status'] = 'IN_PROGRESS'  # 작업중 표시용
            
        elif status == 'COMPLETED':
            # 완료 상태 - assigned에서 제거하고 completed에 추가
            if robot_name in self.task_queue_data['assigned_tasks']:
                del self.task_queue_data['assigned_tasks'][robot_name]
            
            completed_task = {
                'task_id': task_id,
                'robot_name': robot_name,
                'destination': destination,
                'item': item,
                'completion_time': current_time
            }
            self.task_queue_data['completed_tasks'].append(completed_task)
            
            self.root.after(3000, lambda: self.remove_completed_task_from_list(task_id))
        
        # 화면 업데이트
        self.update_task_status_display()

    def remove_completed_task_from_list(self, task_id: str):
        """완료된 업무를 리스트에서 제거하고 최근 완료 기록으로 이동"""
        self.task_queue_data['completed_tasks'] = [
            task for task in self.task_queue_data['completed_tasks'] 
            if task.get('task_id') != task_id
        ]
        self.update_task_status_display()

    def update_all_robot_displays(self):
        for name in ROBOT_NAMES + ['robot_arm']:
            self.update_robot_status_display(name)
            
    def update_all_location_displays(self):
        for location, status in self.location_states.items():
            self.update_location_marker(location, status)

    def run(self):
        """GUI 실행"""
        def ros_spin():
            rclpy.spin(self)
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()
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