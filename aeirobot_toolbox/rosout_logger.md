# RosoutLogger

## 개요
`RosoutLogger`는 ROS2 시스템에서 `/rosout` 토픽을 통해 발행되는 모든 로그 메시지를 구독하여 파일 시스템에 체계적으로 저장하는 로깅 유틸리티임. 본 도구는 대규모 로봇 시스템에서 여러 노드의 로그를 효율적으로 관리하고 분석하기 위해 설계되었음.

## 주요 기능

### 1. 노드 그룹 관리
- YAML 설정 파일을 통해 노드를 논리적 그룹으로 분류함
- 각 그룹별로 별도의 디렉토리에 로그 파일을 생성하여 관리함

### 2. 로그 파일 구조화
- 워크스페이스/log/YYYYMMDD_HHMMSS 형식의 타임스탬프 디렉토리 구조 사용
- 그룹별 서브디렉토리 자동 생성
- 각 노드별로 개별 로그 파일 생성

### 3. 로그 필터링 기능
- 특정 노드만 선택적으로 로깅 가능
- 로그 레벨(DEBUG, INFO, WARN, ERROR, FATAL) 기반 필터링 지원

### 4. 로그 파일 롤오버
- 설정 가능한 시간 간격으로 자동 로그 파일 롤오버
- 시스템 장기 실행 시에도 로그 파일 크기 관리 용이

### 5. 색상 지원
- 로그 레벨별 색상 코드 지원 (선택적 활성화 가능)
- 콘솔 출력 시 가독성 향상

## 사용법

### 1. 파라미터 설정
```yaml
rosout_logger:
  ros__parameters:
    target_nodes: []  # 빈 배열 시 모든 노드 로깅, 특정 노드만 로깅하려면 노드 이름 입력
    log_level: 20     # 로그 레벨 필터 (10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL)
    log_dir: ""       # 빈 문자열 시 자동으로 워크스페이스/log 디렉토리 사용
    log_to_console: false  # 콘솔 출력 여부
    use_colored_log: true  # 색상 코드 사용 여부
    rollover_interval: 3600.0  # 로그 파일 롤오버 간격 (초)
    config_file_path: ""   # 노드 그룹 설정 YAML 파일 경로
```

### 2. 노드 그룹 설정 예시
```yaml
log_groups:
  navigation:
    nodes:
      - nav2_planner
      - nav2_controller
      - nav2_bt_navigator
  perception:
    nodes:
      - camera_node
      - lidar_node
  manipulation:
    nodes:
      - arm_controller
      - gripper_controller
```

### 3. 실행 방법
```bash
# 기본 설정으로 실행
ros2 run aeirobot_toolbox rosout_logger

# 설정 파일 지정하여 실행
ros2 run aeirobot_toolbox rosout_logger --ros-args -p config_file_path:=/path/to/config.yaml

# 파라미터 직접 지정하여 실행
ros2 run aeirobot_toolbox rosout_logger --ros-args -p log_level:=30 -p log_to_console:=true
```

## 기술적 특징

### 스레드 안전성
- `std::mutex`를 사용하여 여러 로그 메시지가 동시에 도착해도 안전하게 처리함

### 파일 시스템 관리
- `std::filesystem`을 활용하여 디렉토리 구조 자동 생성
- 파일 IO 오류에 대한 강건한 예외 처리

### 확장성
- 새로운 노드가 추가되어도 설정 파일만 업데이트하면 자동으로 로깅 구조 조정됨

## 주의사항
- 로그 디렉토리 용량이 충분한지 주기적 확인 필요함
- 롤오버 간격이 너무 짧으면 파일 수가 많아질 수 있으므로 시스템 부하 고려 필요함
- 대량의 로그 출력 시 시스템 성능에 영향을 미칠 수 있음
