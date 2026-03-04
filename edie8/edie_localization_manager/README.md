# alice_localization_manager

## 개요

`alice_localization_manager`는 Alice 로봇의 로컬라이제이션을 담당하는 핵심 패키지임.

본 패키지는 다양한 소스(Vision, Odometry 등)로부터 수신한 위치 정보를 융합하여 로봇의 최종 Pose를 추정하고, RoboCup 경기 상황에 맞는 위치 보정을 수행하는 기능을 제공함.

## 핵심 기능

- **다중 소스 융합 (Pose Fusion)**
  - 여러 센서로부터 입력된 위치 데이터를 융합하여 일관되고 안정적인 로봇의 위치를 계산함.
  - 이상치 제거, 가중 평균, 이동 평균 필터와 같은 융합 전략을 적용함.

- **상황 기반 위치 보정 (Pose Set)**
  - RoboCup Game Controller의 정보에 기반하여, 특정 게임 상태(예: 초기 상태, 페널티킥)에서 로봇의 위치를 사전에 정의된 값으로 강제 설정함.

- **데이터 평가 (Pose Evaluation)**
  - 각 위치 데이터 소스의 유효성과 신뢰도를 지속적으로 평가하여 융합 과정의 정확도를 높임.

## 실행 방법

```bash
# GUI 없이 실행
ros2 launch alice_localization_manager alice4_localization_gui_off.launch.py

# GUI와 함께 실행
ros2 launch alice_localization_manager alice4_localization.launch.py
```
