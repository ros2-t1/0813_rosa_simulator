# GUI/DB 연동 가이드

## 📋 **개요**
팀원과의 GUI/DB 연동을 위해 수령확인 기능을 별도 모듈로 분리했습니다.

## 📁 **파일 구조**

```
src/0812_new/
├── delivery_interface.py     # GUI/DB 연동 전담 모듈 ⭐
├── gui_example.py           # GUI 구현 참고 예시 ⭐
├── task_manager.py          # 로봇 제어 (GUI 로직 제거됨)
├── GUI_연동_가이드.md       # 이 파일
└── ... (기타 파일들)
```

## 🔗 **연동 포인트**

### **1. ROS 토픽 기반 통신**

#### **📤 요청 토픽**: `/rosa/delivery_confirmation_request`
```
업무ID|사용자ID|로봇명|목적지|물품명|주문시간|배달완료시간
```
**예시:**
```
DP_03_1691901234|user123|DP_03|왼쪽방|물|1691901234|1691901534
```

#### **📥 응답 토픽**: `/rosa/delivery_confirmation_response`
```
업무ID|사용자ID|YES/NO
```
**예시:**
```
DP_03_1691901234|user123|YES
```

#### **📊 DB 로그 토픽**: `/rosa/db_log`
```
DB_RECORD|ORDER_ID=업무ID|USER_ID=사용자ID|ROBOT=로봇명|ITEM=물품|DESTINATION=배달지|ORDER_TIME=주문시간|PICKUP_TIME=픽업시간|DELIVERY_TIME=배달시간|CONFIRMATION_TIME=확인시간|STATUS=수령완료/수령거부
```

## 🎯 **GUI 구현 요구사항**

### **수령확인 창 내용**
```
🤖 배달 완료!

[물품]을(를) 받았습니까?

주문번호: DP_03_1691901234
배달지: 왼쪽방  
로봇: DP_03
배달시간: 14:35:42

[예 (Y)]  [아니오 (N)]
```

### **핵심 기능**
1. **사용자 식별**: `user_id`로 정확한 사용자에게만 창 표시
2. **YES 응답**: 수령 완료 → 로봇 충전소 복귀
3. **NO 응답**: 수령 거부 → 로봇 해당 위치 대기 → 관리자 알림 (선택사항)
4. **타임아웃**: 5분 후 자동 NO 처리

## 🔧 **연동 방법**

### **1. 기존 GUI 시스템에 추가**
```python
# 토픽 구독 추가
self.create_subscription(String, '/rosa/delivery_confirmation_request', 
                        self.confirmation_request_callback, 10)

# 토픽 발행 추가  
self.response_pub = self.create_publisher(String, '/rosa/delivery_confirmation_response', 10)
```

### **2. 메시지 처리**
```python
def confirmation_request_callback(self, msg):
    parts = msg.data.split('|')
    order_id, user_id, robot_name, destination, item = parts[0:5]
    
    # 현재 로그인된 사용자와 일치하는지 확인
    if user_id == self.current_user_id:
        self.show_confirmation_dialog(parts)

def send_response(self, order_id, user_id, response):
    response_msg = String()
    response_msg.data = f"{order_id}|{user_id}|{response}"
    self.response_pub.publish(response_msg)
```

### **3. DB 연동 (선택사항)**
```python
# DB 로그 구독
self.create_subscription(String, '/rosa/db_log', self.db_log_callback, 10)

def db_log_callback(self, msg):
    if msg.data.startswith("DB_RECORD|"):
        # DB에 기록 로직
        self.save_to_database(msg.data)
```

## 📝 **테스트 방법**

### **1. 시뮬레이션 모드로 테스트**
```bash
# 터미널 1: 메인 시스템
python3 main.py
# 모드 선택: 2 (시뮬레이션)

# 터미널 2: GUI 예시 실행  
python3 gui_example.py

# 터미널 3: 배달 명령 전송
# main.py에서 명령 입력: "3번 물 왼쪽방"
```

### **2. 토픽 모니터링**
```bash
# 요청 토픽 확인
ros2 topic echo /rosa/delivery_confirmation_request

# 응답 토픽 확인  
ros2 topic echo /rosa/delivery_confirmation_response

# DB 로그 확인
ros2 topic echo /rosa/db_log
```

## 🚀 **실제 연동 시 수정사항**

### **delivery_interface.py**
- 팀원의 DB 시스템에 맞게 `log_to_database()` 메서드 수정
- 필요시 추가 토픽이나 서비스 연동

### **GUI 시스템**  
- 기존 GUI 프레임워크에 맞게 `gui_example.py` 코드 적용
- 사용자 인증 시스템과 연동
- 디자인 및 UX 개선

### **관리자 알림 (추후 구현)**
- NO 응답 시 관리자 GUI에 알림창 표시
- 로봇 상태 모니터링 대시보드 연동

## 💡 **추가 기능 아이디어**

1. **배달 이력 조회**: 사용자별 배달 기록 확인
2. **실시간 알림**: 배달 진행 상황 푸시 알림
3. **평가 시스템**: 배달 서비스 만족도 평가
4. **예약 시스템**: 배달 시간 예약 기능

## 🔍 **문제 해결**

### **GUI 창이 안 뜨는 경우**
1. 사용자 ID 확인 (`user_id` 일치 여부)
2. ROS 토픽 연결 상태 확인
3. 로그 메시지 확인

### **응답이 전송 안 되는 경우**  
1. 토픽 발행자 초기화 확인
2. 메시지 형식 확인 (`업무ID|사용자ID|YES/NO`)
3. ROS 네트워크 연결 상태 확인

---

**연동 관련 문의사항이 있으면 언제든 연락주세요! 🤝**