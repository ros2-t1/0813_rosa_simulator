# 🤖 ROSA 2대 로봇 버전 (DP_03, DP_09)

이 폴더는 **DP_03과 DP_09 로봇 2대만** 사용하는 최적화된 버전입니다.

## 주요 변경사항

### 1. **config.py**
```python
# 기존: ROBOT_NAMES = ['DP_03', 'DP_08', 'DP_09']
# 변경: ROBOT_NAMES = ['DP_03', 'DP_09']

# 충전소도 2개만
ROBOT_CHARGE_STATIONS = {
    'DP_03': '3번 충전소',
    'DP_09': '9번 충전소',
}
```

### 2. **tf_aggregator_2robots.py**
- DP_08 관련 TF 구독 제거
- DP_03 (도메인 13), DP_09 (도메인 19)만 처리
- 상태 추적도 2대만

### 3. **도메인 설정**
```bash
# 제어 스테이션: ROS_DOMAIN_ID=20
# DP_03: ROS_DOMAIN_ID=13  
# DP_09: ROS_DOMAIN_ID=19
```

## 실행 방법

### 제어 스테이션에서:
```bash
# 1. 환경 설정
export ROS_DOMAIN_ID=20

# 2. TF 중계기 실행
python3 tf_aggregator_2robots.py

# 3. ROSA 시스템 실행  
python3 main.py
```

### 로봇별 도메인:
- **DP_03**: 도메인 13
- **DP_09**: 도메인 19

## 주의사항
- 도메인 브릿지는 별도로 수정하여 DP_03, DP_09만 연결하도록 설정 필요
- 기존 3대 로봇 버전과 동시 실행 금지