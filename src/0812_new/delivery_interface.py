#!/usr/bin/env python3
# delivery_interface.py

"""
GUI/DB 연동 전담 모듈
팀원과의 연동을 위해 별도 분리된 파일
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime

class DeliveryInterface:
    """
    GUI 및 DB 연동을 전담하는 클래스
    다른 팀원의 GUI/DB 시스템과 연동하기 위한 인터페이스 역할
    """
    
    def __init__(self, node: Node, task_manager_callback=None):
        self.node = node
        self.logger = node.get_logger()
        self.task_manager_callback = task_manager_callback
        
        # GUI 수령확인 관련 토픽
        self.delivery_confirm_pub = self.node.create_publisher(String, '/rosa/delivery_confirmation_request', 10)
        self.node.create_subscription(String, '/rosa/delivery_confirmation_response', self.delivery_confirmation_callback, 10)
        
        # DB 로그 관련 토픽 (필요시 추가)
        self.db_log_pub = self.node.create_publisher(String, '/rosa/db_log', 10)
        
        self.logger.info("✅ DeliveryInterface 초기화 완료 (GUI/DB 연동 전담)")

    def request_delivery_confirmation(self, task_info):
        """
        GUI에 수령확인 요청을 보내는 함수
        
        Args:
            task_info (dict): 업무 정보
                - order_id: 업무 고유 ID
                - user_id: 주문한 사용자 ID  
                - robot_name: 로봇명
                - destination: 배달지
                - item: 물품명
                - order_time: 주문 시간
                - delivery_time: 배달 완료 시간
        """
        try:
            confirm_msg = String()
            
            # GUI 연동을 위한 메시지 형식
            # "업무ID|사용자ID|로봇명|목적지|물품명|주문시간|배달완료시간"
            confirm_msg.data = (f"{task_info['order_id']}|"
                               f"{task_info['user_id']}|"
                               f"{task_info['robot_name']}|"
                               f"{task_info['destination']}|"
                               f"{task_info['item']}|"
                               f"{task_info['order_time']:.0f}|"
                               f"{task_info['delivery_time']:.0f}")
            
            self.delivery_confirm_pub.publish(confirm_msg)
            
            self.logger.info(f"📋 GUI 확인 요청 발송: {task_info['user_id']}님께 '{task_info['item']}' 수령확인 요청")
            self.logger.info(f"📋 업무 ID: {task_info['order_id']}, 배달지: {task_info['destination']}")
            
        except Exception as e:
            self.logger.error(f"❌ GUI 확인 요청 발송 실패: {e}")

    def delivery_confirmation_callback(self, msg: String):
        """
        GUI로부터 수령확인 응답을 받는 콜백 함수
        응답을 TaskManager로 전달
        """
        try:
            # 응답 메시지 파싱: "업무ID|사용자ID|YES/NO"
            parts = msg.data.split('|')
            
            if len(parts) >= 3:
                order_id = parts[0]
                user_id = parts[1] 
                response = parts[2].upper()
                
                self.logger.info(f"📨 GUI 응답 수신: {user_id}님이 업무 {order_id}에 대해 '{response}' 응답")
                
                # TaskManager로 응답 전달
                if self.task_manager_callback:
                    confirmation_data = {
                        'order_id': order_id,
                        'user_id': user_id,
                        'response': response,
                        'timestamp': time.time()
                    }
                    self.task_manager_callback(confirmation_data)
                    
            else:
                self.logger.warn(f"⚠️ 잘못된 GUI 응답 형식: {msg.data}")
                
        except Exception as e:
            self.logger.error(f"❌ GUI 응답 처리 중 오류: {e}")

    def log_to_database(self, task_info, status):
        """
        DB에 배달 완료 정보를 기록하는 함수
        
        Args:
            task_info (dict): 업무 정보
            status (str): "수령완료" 또는 "수령거부"
        """
        try:
            # 시간 문자열 변환
            order_time_str = datetime.fromtimestamp(task_info['order_time']).strftime('%Y-%m-%d %H:%M:%S')
            pickup_time_str = datetime.fromtimestamp(task_info['pickup_time']).strftime('%Y-%m-%d %H:%M:%S') if task_info.get('pickup_time') else "미기록"
            delivery_time_str = datetime.fromtimestamp(task_info['delivery_time']).strftime('%Y-%m-%d %H:%M:%S') if task_info.get('delivery_time') else "미기록"
            confirmation_time_str = datetime.fromtimestamp(task_info['confirmation_time']).strftime('%Y-%m-%d %H:%M:%S') if task_info.get('confirmation_time') else "미기록"
            
            # DB 기록용 구조화된 메시지
            db_message = String()
            db_message.data = (f"DB_RECORD|"
                             f"ORDER_ID={task_info['order_id']}|"
                             f"USER_ID={task_info['user_id']}|"
                             f"ROBOT={task_info['robot_name']}|"
                             f"ITEM={task_info['item']}|"
                             f"DESTINATION={task_info['destination']}|"
                             f"ORDER_TIME={order_time_str}|"
                             f"PICKUP_TIME={pickup_time_str}|"
                             f"DELIVERY_TIME={delivery_time_str}|"
                             f"CONFIRMATION_TIME={confirmation_time_str}|"
                             f"STATUS={status}")
            
            self.db_log_pub.publish(db_message)
            
            # 사람이 읽기 쉬운 로그
            total_time = (task_info['confirmation_time'] - task_info['order_time']) if task_info.get('confirmation_time') and task_info.get('order_time') else 0
            self.logger.info(f"📊 DB 기록: {task_info['user_id']}님의 {task_info['item']} → {task_info['destination']} ({status}, 총 {total_time:.0f}초)")
            
        except Exception as e:
            self.logger.error(f"❌ DB 로그 기록 실패: {e}")

    def generate_gui_display_text(self, task_info):
        """
        GUI 창에 표시할 텍스트 생성
        팀원이 GUI 구현할 때 참고용
        
        Returns:
            dict: GUI 표시용 정보
        """
        return {
            'title': '배달 완료 확인',
            'message': f"{task_info['item']}을(를) 받았습니까?",
            'details': {
                '주문번호': task_info['order_id'],
                '배달지': task_info['destination'],
                '물품': task_info['item'],
                '로봇': task_info['robot_name'],
                '배달시간': datetime.fromtimestamp(task_info['delivery_time']).strftime('%H:%M:%S')
            },
            'buttons': ['YES', 'NO'],
            'timeout': 300  # 5분 타임아웃
        }

