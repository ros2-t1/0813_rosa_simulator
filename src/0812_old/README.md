# 🤖 ROSA (Robot Operation & Service Automation) 시스템

**다중 로봇 자동 배달 관제 시스템**

ROSA는 여러 대의 자율주행 로봇을 통합 관리하여 병원, 요양원, 호텔 등에서 자동 배달 서비스를 제공하는 ROS2 기반 시스템입니다.

---

## 📋 목차
- [시스템 개요](#-시스템-개요)
- [주요 기능](#-주요-기능)
- [시스템 아키텍처](#-시스템-아키텍처)
- [설치 및 설정](#-설치-및-설정)
- [사용법](#-사용법)
- [파일 구조](#-파일-구조)
- [설정 파일](#-설정-파일)
- [문제 해결](#-문제-해결)
- [개발자 가이드](#-개발자-가이드)

---

## 🎯 시스템 개요

### 주요 특징
- **다중 로봇 지원**: 최대 3대 로봇(DP_03, DP_08, DP_09) 동시 관리
- **이중 모드**: 시뮬레이션 모드와 실제 로봇 모드 지원
- **자연어 명령**: "3번 왼쪽방에 물 배달" 같은 직관적 명령어
- **실시간 모니터링**: 로봇 상태, 위치, 배터리 실시간 추적
- **장소 관리**: 충돌 방지를 위한 예약 시스템
- **비상 제어**: 비상정지, 재개, 강제복귀 기능

### 지원 환경
- **운영체제**: Ubuntu 20.04/22.04
- **ROS 버전**: ROS2 Humble/Foxy
- **Python**: 3.8+
- **네트워크**: DDS 도메인 브리지 지원

---

## ⭐ 주요 기능

### 1. 자동 배달 서비스
```bash
# 예시 명령어들
"3번 왼쪽방에 물 배달"     # 배달 업무
"8번 면회실 가"           # 이동 업무  
"9번 복귀해"             # 충전소 복귀
```

### 2. 실시간 로봇 관리
- **상태 추적**: 13개 세분화된 로봇 상태
- **위치 모니터링**: GPS 좌표 + 논리적 위치
- **배터리 관리**: 자동 저전력 복귀
- **작업 스케줄링**: 예약 시스템으로 충돌 방지

### 3. 안전 기능
- **비상정지**: 즉시 모든 동작 중단
- **타임아웃 감지**: 응답 없는 로봇 자동 감지
- **에러 복구**: 자동/수동 시스템 복구

### 4. 통합 모니터링
- **실시간 로그**: 모든 이벤트 시간순 출력
- **상태 대시보드**: 로봇별 현재 상황 확인
- **히스토리 추적**: 작업 완료 이력 관리

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   관제 PC       │    │   로봇 DP_03    │    │   로봇팔 시스템  │
│                 │    │                 │    │                 │
│ ┌─TaskManager   │◄──►│ ┌─PathExecutor  │    │ ┌─HANA Arm     │
│ ├─CommandParser │    │ ├─Nav2 Stack    │◄──►│ ├─ArUco 인식   │
│ ├─LocationMgr   │    │ ├─AMCL         │    │ └─픽업/배치     │
│ └─StatusLogger  │    │ └─Battery Mon   │    └─────────────────┘
└─────────────────┘    └─────────────────┘
         │                       │
         └───────────────────────┼─────────────────────────
                                 │
    ┌─────────────────┐    ┌─────────────────┐
    │   로봇 DP_08    │    │   로봇 DP_09    │
    │                 │    │                 │
    │ ┌─PathExecutor  │    │ ┌─PathExecutor  │
    │ ├─Nav2 Stack    │    │ ├─Nav2 Stack    │
    │ ├─AMCL         │    │ ├─AMCL         │
    │ └─Battery Mon   │    │ └─Battery Mon   │
    └─────────────────┘    └─────────────────┘
```

### DDS 도메인 구성
- **관제 PC**: Domain 20
- **DP_03**: Domain 13  
- **DP_08**: Domain 18
- **DP_09**: Domain 19
- **로봇팔**: Domain 14

---

## 🚀 설치 및 설정

### 1. 기본 요구사항
```bash
# ROS2 설치 (Ubuntu 22.04 기준)
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install python3-colcon-common-extensions

# 추가 패키지
sudo apt install python3-yaml python3-pip
pip3 install pyyaml
```

### 2. 워크스페이스 설정
```bash
# 워크스페이스 생성
mkdir -p ~/rosa_ws/src
cd ~/rosa_ws/src

# ROSA 시스템 복사
git clone <repository_url> rosa_system
cd ~/rosa_ws

# 빌드
colcon build
source install/setup.bash
```

### 3. 서비스 인터페이스 생성
```bash
# rosa_interfaces 패키지 생성
cd ~/rosa_ws/src
ros2 pkg create --build-type ament_cmake rosa_interfaces

# srv 파일들 복사
mkdir -p rosa_interfaces/srv
cp GetLocationStatus.srv rosa_interfaces/srv/
cp UpdateLocationStatus.srv rosa_interfaces/srv/

# CMakeLists.txt 및 package.xml 수정 후 빌드
cd ~/rosa_ws
colcon build --packages-select rosa_interfaces
source install/setup.bash
```

### 4. 환경 변수 설정
```bash
# ~/.bashrc에 추가
echo "export ROS_DOMAIN_ID=20" >> ~/.bashrc
echo "source ~/rosa_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 📖 사용법

### 1. 시스템 시작

#### A. 시뮬레이션 모드 (개발/테스트용)
```bash
cd ~/rosa_ws/src/rosa_system
python3 main.py
# 모드 선택: 2 (시뮬레이션)
```

#### B. 실제 로봇 모드 (운영용)
```bash
# 터미널 1: Location Manager 실행
python3 location_manager.py

# 터미널 2: Status Logger 실행 (선택사항)
python3 status_logger.py

# 터미널 3: 메인 시스템 실행
python3 main.py
# 모드 선택: 1 (실제 로봇)

# 각 로봇에서 PathExecutor 실행
# 로봇 DP_03에서:
export ROBOT_NAME=DP_03
python3 path_executor.py
```

### 2. 기본 명령어

#### 배달 업무
```bash
명령어 입력 > 3번 왼쪽방에 물 배달
명령어 입력 > 8번 오른쪽방에 식판 배달  
명령어 입력 > 9번 면회실에 영양제 배달
```

#### 이동 업무
```bash
명령어 입력 > 3번 면회실 가
명령어 입력 > 8번 출입구로 이동
명령어 입력 > 9번 픽업대 가서
```

#### 상태 확인
```bash
명령어 입력 > 3번 뭐해          # 현재 업무 확인
명령어 입력 > 8번 어디야        # 현재 위치 확인
명령어 입력 > 9번 상태          # 상태 확인
```

#### 제어 명령
```bash
명령어 입력 > 3번 멈춰          # 비상정지
명령어 입력 > 3번 계속해        # 업무 재개
명령어 입력 > 8번 복귀해        # 강제 충전소 복귀
명령어 입력 > 9번 새로고침      # 상태 리셋
```

### 3. 실시간 모니터링

시스템 실행 시 다음과 같은 실시간 정보를 확인할 수 있습니다:

```bash
[INFO] [timestamp] [task_manager]: 📝 새 배달 업무 할당: 'DP_03' -> '왼쪽방'에 '물' 배달
[INFO] [timestamp] [task_manager]: 🤖 [DP_03] 상태: MOVING_TO_PICKUP | 업무: 왼쪽방에 물 배달 중
[INFO] [timestamp] [task_manager]: ✅ [DP_03]의 이동 작업이 완료되었습니다.
```

---

## 📁 파일 구조

```
rosa_system/
├── main.py                 # 메인 실행 파일
├── task_manager.py         # 핵심 업무 관리 시스템
├── command_parser.py       # 자연어 명령어 해석기
├── config.py              # 시스템 설정 파일
├── simulation_test.py      # 시뮬레이션 모드 구현
├── location_manager.py     # 장소 상태 관리 서비스
├── path_executor.py        # 로봇 개별 경로 실행기
├── status_logger.py        # 실시간 이벤트 로거
├── waypoints.yaml         # 경로 및 목적지 정의
├── GetLocationStatus.srv   # 장소 상태 조회 서비스
├── UpdateLocationStatus.srv # 장소 상태 업데이트 서비스
└── 0812_domain.yaml       # DDS 도메인 브리지 설정
```

### 주요 파일 설명

#### 1. `task_manager.py` - 시스템 두뇌
- **역할**: 모든 로봇의 업무 할당, 상태 관리, 스케줄링
- **핵심 기능**:
  - 13개 로봇 상태 관리 (IDLE, MOVING, PICKING_UP 등)
  - 장소 예약 시스템으로 충돌 방지
  - 배터리 모니터링 및 자동 복귀
  - 타임아웃 감지 및 에러 처리
  - ArUco ID 기반 물품 구분

#### 2. `command_parser.py` - 자연어 처리기
- **역할**: 사용자 명령을 파싱하여 시스템 명령으로 변환
- **지원 패턴**:
  ```python
  # 배달: [로봇] [장소] [물품] 배달
  # 이동: [로봇] [장소] 가/이동
  # 제어: [로봇] 멈춰/계속해/복귀해
  # 조회: [로봇] 뭐해/어디야/상태
  ```

#### 3. `simulation_test.py` - 시뮬레이션 엔진
- **역할**: 실제 로봇 없이 시스템 테스트
- **시뮬레이션 대상**:
  - 로봇 이동 (3초)
  - 로봇팔 픽업 (3초)  
  - 배달 작업 (3초)
  - 사용자 확인 (3초)

#### 4. `config.py` - 설정 중앙화
- **포함 내용**:
  - 로봇 목록 및 충전소 매핑
  - 장소 좌표 정의
  - 물품별 ArUco ID 매핑
  - 배터리 임계값 등 시스템 파라미터

#### 5. `location_manager.py` - 장소 관리자
- **역할**: 모든 장소의 점유 상태 관리
- **상태 종류**:
  - `available`: 사용 가능
  - `reserved`: 예약됨 (로봇이 오는 중)
  - `busy`: 사용 중 (로봇이 있음)

#### 6. `path_executor.py` - 로봇별 실행기
- **역할**: 관제 PC의 경로 명령을 받아 Nav2로 실행
- **각 로봇에서 실행**: 환경변수 `ROBOT_NAME`으로 구분
- **기능**: 경로 수신 → Nav2 실행 → 결과 보고

#### 7. `waypoints.yaml` - 경로 정의
- **구성**:
  - `highway_up`: 상행선 (아래→위)
  - `highway_down`: 하행선 (위→아래)  
  - `destinations`: 최종 목적지들

---

## ⚙️ 설정 파일

### 1. `config.py` 주요 설정

```python
# 로봇 목록
ROBOT_NAMES = ['DP_03', 'DP_08', 'DP_09']

# 배터리 자동 복귀 임계값
BATTERY_THRESHOLD = 40.0

# 물품별 ArUco ID (로봇팔 인식용)
ITEM_ARUCO_MAP = {
    "식판": [4, 5],
    "물": [6, 7], 
    "영양제": [8, 9],
}

# 장소 좌표 (미터)
LOCATIONS = {
    '왼쪽방': (0.0, 0.9),
    '오른쪽방': (0.4, 0.9),
    '픽업대': (0.22, -0.22),
    # ...
}
```

### 2. 좌표 시스템

맵 좌표계 기준으로 모든 위치가 정의됩니다:
- **원점**: 맵의 중심점
- **단위**: 미터
- **방향**: X축(좌우), Y축(앞뒤)

### 3. 로봇 상태 정의

```python
class RobotState(Enum):
    CHARGING = auto()                    # 충전 중
    IDLE = auto()                       # 대기 중
    AWAITING_PICKUP_RESERVATION = auto() # 픽업대 예약 대기
    MOVING_TO_PICKUP = auto()           # 픽업대로 이동
    PICKING_UP = auto()                 # 픽업 작업 중
    AWAITING_DEST_RESERVATION = auto()   # 목적지 예약 대기
    MOVING_TO_DEST = auto()             # 목적지로 이동
    DELIVERING = auto()                 # 배달 중
    AWAITING_CONFIRMATION = auto()       # 확인 대기
    RETURNING = auto()                  # 충전소 복귀 중
    EMERGENCY_STOP = auto()             # 비상정지
    OFF_DUTY = auto()                   # 강제 복귀 중
    WAITING = auto()                    # 호출 대기
```

---

## 🔧 문제 해결

### 자주 발생하는 문제들

#### 1. 서비스 인터페이스 오류
```bash
# 에러: ModuleNotFoundError: No module named 'rosa_interfaces'
# 해결:
cd ~/rosa_ws
colcon build --packages-select rosa_interfaces
source install/setup.bash
```

#### 2. 로봇이 응답하지 않음
```bash
# 증상: TIMEOUT 메시지 출력
# 해결:
명령어 입력 > 3번 새로고침      # 상태 리셋
명령어 입력 > 3번 복귀해        # 강제 복귀
```

#### 3. 장소 예약 실패
```bash
# 증상: "픽업대가 사용 중입니다" 계속 출력
# 원인: 다른 로봇이 해당 장소 점유 중
# 해결: 해당 로봇 확인 후 필요시 복귀 명령
```

#### 4. 좌표 불일치 경고
```bash
# 증상: "실제로는 'XX' 근처" 메시지
# 원인: config.py와 실제 좌표 차이
# 해결: config.py 좌표 업데이트 필요
```

### 로그 해석

#### 정상 작업 흐름
```bash
✅ 배달 명령: 'DP_03' → '물'을(를) '왼쪽방'에 배달
📝 새 배달 업무 할당: 'DP_03' -> '왼쪽방'에 '물' 배달
🤖 [DP_03] 상태: AWAITING_PICKUP_RESERVATION → MOVING_TO_PICKUP
🤖 [DP_03] 로봇팔에 ArUco ID 6 픽업 명령 전송
🤖 [DP_03] 상태: PICKING_UP → AWAITING_DEST_RESERVATION  
🤖 [DP_03] 상태: MOVING_TO_DEST → DELIVERING
🏠 [DP_03] 복귀 완료. 충전 중입니다.
```

#### 에러 상황
```bash
🚨 [DP_03] 로봇이 1분 동안 응답하지 않습니다! # 타임아웃
⏳ [DP_03] 픽업대가 사용 중입니다. 대기 중...  # 예약 실패
❌ 로봇 'DP_03'을(를) 찾을 수 없습니다.      # 잘못된 로봇명
```

---

## 👨‍💻 개발자 가이드

### 1. 새로운 로봇 추가

```python
# config.py 수정
ROBOT_NAMES = ['DP_03', 'DP_08', 'DP_09', 'DP_10']  # 새 로봇 추가

ROBOT_CHARGE_STATIONS = {
    'DP_03': '3번 충전소',
    'DP_08': '8번 충전소', 
    'DP_09': '9번 충전소',
    'DP_10': '10번 충전소',  # 새 충전소 추가
}

LOCATIONS.update({
    '10번 충전소': (-0.2, 0.75),  # 새 충전소 좌표
})
```

```python
# command_parser.py 수정
self.robot_aliases = {
    "3번": "DP_03", "8번": "DP_08", "9번": "DP_09",
    "10번": "DP_10", "10번로봇": "DP_10",  # 새 별칭 추가
}
```

### 2. 새로운 물품 추가

```python
# config.py 수정
ITEM_ARUCO_MAP = {
    "식판": [4, 5],
    "물": [6, 7],
    "영양제": [8, 9],
    "약": [10, 11],  # 새 물품 추가
}
```

```python
# command_parser.py 수정
self.delivery_items = ["식판", "물", "영양제", "약"]  # 새 물품 추가
```

### 3. 새로운 장소 추가

```python
# config.py 수정
LOCATIONS = {
    '왼쪽방': (0.0, 0.9),
    '오른쪽방': (0.4, 0.9),
    '신규방': (0.8, 0.9),  # 새 장소 추가
    # ...
}
```

```yaml
# waypoints.yaml 수정
destinations:
  - name: "신규방"
    pose:
      position: {x: 0.8, y: 0.9, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

### 4. 시뮬레이션 타이밍 조정

```python
# simulation_test.py 수정
def __init__(self, task_manager):
    self.MOVE_TIME = 5.0      # 이동 시간 증가
    self.PICKUP_TIME = 2.0    # 픽업 시간 단축
    self.DELIVERY_TIME = 4.0  # 배달 시간 조정
    self.CONFIRM_TIME = 1.0   # 확인 시간 단축
```

### 5. 새로운 명령어 추가

```python
# command_parser.py의 parse_command() 수정
elif "점검해" in command or "진단" in command:
    print(f"🔧 진단 명령: '{found_robot}' 시스템 점검")
    self.task_manager.diagnose_robot(found_robot)
    return
```

```python
# task_manager.py에 새 메서드 추가
def diagnose_robot(self, robot_name: str):
    robot = self.robots.get(robot_name)
    if not robot:
        self.get_logger().warn(f"❌ 로봇 '{robot_name}'을(를) 찾을 수 없습니다.")
        return
    
    # 진단 로직 구현
    self.get_logger().info(f"🔧 [{robot_name}] 시스템 진단을 시작합니다...")
    # ...
```

### 6. 네트워크 설정

실제 다중 로봇 환경에서는 DDS 도메인 브리지가 필요합니다:

```bash
# 도메인 브리지 실행
ros2 run domain_bridge domain_bridge 0812_domain.yaml
```

---

## 📞 지원 및 문의

### 시스템 요구사항
- **ROS2**: Humble 또는 Foxy
- **Python**: 3.8 이상
- **메모리**: 최소 4GB RAM
- **네트워크**: 기가비트 이더넷 권장

### 성능 최적화
- **로봇 수**: 최대 10대까지 확장 가능
- **동시 작업**: 모든 로봇 병렬 작업 지원
- **응답 속도**: 평균 100ms 이내
- **안정성**: 24시간 연속 운영 가능

### 라이선스
이 프로젝트는 MIT 라이선스 하에 배포됩니다.

---

## 🔄 업데이트 로그

### v1.0.0 (현재)
- ✅ 기본 다중 로봇 관제 시스템
- ✅ 시뮬레이션/실제 모드 이중 지원
- ✅ 자연어 명령어 처리
- ✅ 실시간 상태 모니터링
- ✅ 비상정지/재개 기능
- ✅ ArUco ID 기반 물품 구분
- ✅ 장소 예약 시스템

### 향후 계획
- 🔄 웹 대시보드 개발
- 🔄 음성 명령 지원
- 🔄 AI 기반 경로 최적화
- 🔄 스마트폰 앱 연동

---

**🎉 ROSA 시스템을 사용해주셔서 감사합니다!**

시스템 사용 중 문제가 발생하거나 개선 사항이 있으시면 언제든 문의해주세요.