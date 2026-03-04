# aeirobot_toolbox
aeirobot 도구상자 패키지 모음.

---

## 목차
1. [aeirobot_hand_msgs](#aeirobot_hand_msgs)
2. [aeirobot_math](#aeirobot_math)
   - [algebra](#algebra)
   - [optimizer](#optimizer)
   - [trajectory](#trajectory)
3. [aeirobot_msgs](#aeirobot_msgs)
4. [aeirobot_robotics](#aeirobot_robotics)
   - [kinematics](#kinematics)
   - [linear_rotary_converter](#linear_rotary_converter)
5. [aeirobot_state_msgs](#aeirobot_state_msgs)
6. [aeirobot_toolbox](#aeirobot_toolbox_module)

---

<details id="aeirobot_hand_msgs">
<summary><strong>1. aeirobot_hand_msgs</strong></summary>

ROS 2 손 그리퍼 통신을 위한 메시지·서비스 정의 모음.

### 주요 메시지
| `이름`           | 설명 |
| ---------------- | ---- |
| `GetAngleAct`    | -    |
| `GetAngleSet`    | -    |
| `GetCurrentAct`  | -    |
| `GetError`       | -    |
| `GetForceAct`    | -    |
| `GetForceSet`    | -    |
| `GetPosAct`      | -    |
| `GetPosSet`      | -    |
| `GetSpeedSet`    | -    |
| `GetTemp`        | -    |
| `HandTask`       | -    |
| `SetAngle`       | -    |
| `SetAngleResult` | -    |
| `SetForce`       | -    |
| `SetForceResult` | -    |
| `SetPos`         | -    |
| `SetPosResult`   | -    |
| `SetSpeed`       | -    |
| `SetSpeedResult` | -    |

### 주요 서비스
| `이름`          | 설명 |
| --------------- | ---- |
| `GetAngleAct`   | -    |
| `GetAngleSet`   | -    |
| `GetCurrentAct` | -    |
| `GetError`      | -    |
| `GetForceAct`   | -    |
| `GetForceSet`   | -    |
| `GetPosAct`     | -    |
| `GetPosSet`     | -    |
| `GetSpeedSet`   | -    |
| `GetTemp`       | -    |
| `SetAngle`      | -    |
| `SetForce`      | -    |
| `SetPos`        | -    |
| `SetSpeed`      | -    |

</details>

---

<details id="aeirobot_math">
<summary><strong>2. aeirobot_math</strong></summary>

다양한 수치 연산, 최적화, 궤적 생성 알고리즘을 모듈화한 C++ 헤더 라이브러리.

<!-- algebra -->
<details id="algebra">
<summary><strong>├─ algebra</strong></summary>

#### incremental_jacobian.hpp
> 헤더 설명:
>리그룹 기반 자코비안 계산에서 Propagate 방법으로 계산하는 함수가 들어있는 헤더 파일

| `함수`                | 설명                             |
| --------------------- | -------------------------------- |
| `PropagateKinematics` | Propagate 방법으로 자코비안 계산 |

#### lie_algebra.hpp
> 헤더 설명:
>리그룹에서 사용되는 skew symmetric(반대칭 연산) 관련 헤더

| `함수`                             | 설명                                           |
| ---------------------------------- | ---------------------------------------------- |
| `Ceil3DVectorOperator`             | 3x1 벡터 → 3x3 반대칭 행렬 변환                |
| `Floor3DVectorOperator`            | 3x3 반대칭 행렬 → 3x1 벡터 변환                |
| `Ceil6DVectorOperator`             | 6x1 벡터 → 4x4 반대칭 행렬 변환                |
| `Floor6DVectorOperator`            | 4x4 반대칭 행렬 → 6x1 벡터 변환                |
| `AdjointTransform`                 | 4x4 변환 행렬 → 6x6 Adjoint 행렬 변환          |
| `AdjointTransformInverseTranspose` | 4x4 변환 행렬 → 6x6 Adjoint 역행렬 전치행렬    |
| `AdjointTransformTranspose`        | 4x4 변환 행렬 → 6x6 Adjoint 전치행렬           |
| `CreateAdjointMatrix`              | 6x1 벡터 → 6x6 adjoint 행렬 변환               |
| `InverseAdjoint`                   | 4x4 변환 행렬 → 6x6 Adjoint 역행렬 변환        |
| `AdOperator`                       | adjoint operator 계산([v; omega] 입력)         |
| `CoAdOperator`                     | co adjoint (−1 × adjoint operator의 transpose) |

#### linear_algebra.hpp
> 헤더 설명:
>선형대수 관련 헤더

| `함수`                         | 설명                               |
| ------------------------------ | ---------------------------------- |
| `getTransitionXYZ`             | XYZ 좌표 → Eigen::Vector3d 변환    |
| `getTransformationXYZRPY`      | XYZ·RPY → 4×4 변환행렬 생성        |
| `getInverseTransformation`     | 변환행렬의 역행렬 계산             |
| `getInertiaXYZ`                | 관성 모멘트 → 3×3 관성행렬         |
| `getRotationX`                 | X축 회전행렬 생성                  |
| `getRotationY`                 | Y축 회전행렬 생성                  |
| `getRotationZ`                 | Z축 회전행렬 생성                  |
| `getRotation4d`                | RPY 회전 → 4×4 회전행렬 생성       |
| `getTranslation4D`             | XYZ 평행이동 4×4 행렬 생성         |
| `convertRotationToRPY`         | 회전행렬 → RPY 변환                |
| `convertRPYToRotation`         | RPY → 회전행렬 변환                |
| `convertRPYToQuaternion`       | RPY → Quaternion 변환              |
| `convertRotationToQuaternion`  | 회전행렬 → Quaternion 변환         |
| `convertQuaternionToRPY`       | Quaternion → RPY 변환              |
| `convertQuaternionToRotation`  | Quaternion → 회전행렬 변환         |
| `calcHatto`                    | 벡터 → hat(·) 반대칭 행렬 변환     |
| `calcRodrigues`                | Rodrigues 공식으로 회전행렬 계산   |
| `convertRotToOmega`            | 회전행렬 → 회전벡터 ω 변환         |
| `calcCross`                    | 두 벡터의 외적 계산                |
| `calcInner`                    | 두 벡터의 내적 계산                |
| `getPose3DfromTransformMatrix` | 변환행렬 → Pose3D 구조체 변환      |
| `GetPose3DfromTransformMatrix` | 변환행렬 → PoseXYZRPY 메시지 변환  |
| `RotationMatrixToRPY`          | 회전행렬 → RPY (특이점 처리)       |
| `GetTransformationMatrix`      | XYZ·RPY → 4×4 변환행렬 생성        |
| `SkewSymmetricMatrix`          | 벡터 → 스큐대칭 행렬 변환          |
| `DHMatrix`                     | 표준 DH 파라미터 변환행렬 생성     |
| `ModifiedDHMatrix`             | Modified‑DH 파라미터 변환행렬 생성 |
| `DampedLeastSquaresInverse`    | 댐핑 최소제곱 역행렬 계산          |

</details>

<!-- optimizer -->
<details id="optimizer">
<summary><strong>├─ optimizer</strong></summary>

#### riccati_solver.hpp
> 헤더 설명:
>riccati_solver 을 사용한 최적화 관련 헤더

| `함수`                      | 설명                                    |
| --------------------------- | --------------------------------------- |
| `SolveRiccatiIterationC`    | 연속체 모델 Riccati 방정식 반복 해법    |
| `SolveRiccatiIterationD`    | 이산 모델 Riccati 방정식 반복 해법      |
| `SolveRiccatiArimotoPotter` | Arimoto‑Potter 방식 연속체 Riccati 해법 |

</details>

<!-- trajectory -->
<details id="trajectory">
<summary><strong>└─ trajectory</strong></summary>

#### bezier_curve.hpp
> 헤더 설명:
> 베지어 커브 생성 헤더

| `함수`                   | 설명                                |
| ------------------------ | ----------------------------------- |
| `setBezierControlPoints` | 2D 제어점 설정해 Bézier 곡선 초기화 |
| `getPoint`               | 파라미터 t(0-1)에서 곡선상 점 계산  |

#### fifth_order_polynomial_trajectory.hpp
> 헤더 설명:
> 5차 다항식 트레젝토리 생성 헤더

| `함수`             | 설명                            |
| ------------------ | ------------------------------- |
| `changeTrajectory` | 목표 종료 조건 갱신·계수 재계산 |
| `getPosition`      | 위치 값 계산                    |
| `getVelocity`      | 속도 값 계산                    |
| `getAcceleration`  | 가속도 값 계산                  |
| `setTime`          | 내부 시각 설정 및 상태 업데이트 |

#### minimum_jerk_trajectory_with_via_point.hpp
> 헤더 설명:
>중간 지점과 minimum_jerk 를 고려한 트레젝토리 생성 헤더

| `함수`            | 설명                      |
| ----------------- | ------------------------- |
| `getPosition`     | 각 관절 위치(벡터) 계산   |
| `getVelocity`     | 각 관절 속도(벡터) 계산   |
| `getAcceleration` | 각 관절 가속도(벡터) 계산 |

#### minimum_jerk_trajectory.hpp
> 헤더 설명:
>minimum_jerk 를 고려한 트레젝토리 생성 헤더

| `함수`            | 설명                      |
| ----------------- | ------------------------- |
| `getPosition`     | 각 관절 위치(벡터) 계산   |
| `getVelocity`     | 각 관절 속도(벡터) 계산   |
| `getAcceleration` | 각 관절 가속도(벡터) 계산 |

#### ninth_order_polynomial_trajectory.hpp
> 헤더 설명:
>9차 다항식 트레젝토리 생성 헤더

| `함수`             | 설명                       |
| ------------------ | -------------------------- |
| `changeTrajectory` | 종료 조건 갱신·계수 재계산 |
| `getPosition`      | 위치 계산                  |
| `getVelocity`      | 속도 계산                  |
| `getAcceleration`  | 가속도 계산                |
| `getJerk`          | jerk 값 계산               |
| `getSnap`          | snap 값 계산               |
| `setTime`          | 내부 시각 및 상태 업데이트 |
| (get* 계열)        | 현재값 반환                |

#### seventh_order_polynomial_trajectory.hpp
> 헤더 설명:
>7차 다항식 트레젝토리 생성 헤더

| `함수`             | 설명                       |
| ------------------ | -------------------------- |
| `changeTrajectory` | 종료 조건 갱신·계수 재계산 |
| `getPosition`      | 위치 계산                  |
| `getVelocity`      | 속도 계산                  |
| `getAcceleration`  | 가속도 계산                |
| `getJerk`          | jerk 값 계산               |
| `setTime`          | 내부 시각 및 상태 업데이트 |
| (get* 계열)        | 현재값 반환                |

#### simple_trapezoidal_velocity_profile.hpp
> 헤더 설명:
>속도 프로파일을 사용한 트레젝토리 생성 헤더

| `함수`                      | 설명                       |
| --------------------------- | -------------------------- |
| `setVelocityBaseTrajectory` | 트레젝토리 프로파일 설정   |
| `setTimeBaseTrajectory`     | 시간기반 프로파일 설정     |
| `getPosition`               | 위치 계산                  |
| `getVelocity`               | 속도 계산                  |
| `getAcceleration`           | 가속도 계산                |
| `setTime`                   | 내부 시간 및 상태 업데이트 |
| (get* 계열)                 | 현재값 반환                |

#### spline.hpp
> 헤더 설명:
>스플라인 보간법 헤더

| `함수`                        | 설명                   |
| ----------------------------- | ---------------------- |
| `BSplineBasisRecursive`       | B‑spline 기저함수 계산 |
| `EvaluateClampedCubicBSpline` | 3차 B‑spline 평가      |

#### trajectory_calculator.hpp
> 헤더 설명:
>트레젝토리 생성 통합헤더

| `함수`                                    | 설명                       |
| ----------------------------------------- | -------------------------- |
| `calcMinimumJerkTra`                      | 최소 jerk 궤적 벡터 생성   |
| `calcMinimumJerkTraPlus`                  | 위치·속도·가속도 행렬 생성 |
| `calcMinimumJerkTraWithViaPoints`         | 다중 via‑point 궤적 생성   |
| `calcMinimumJerkTraWithViaPointsPosition` | 위치제약 기반 궤적 생성    |
| `calcArc3dTra`                            | 3D 원호 궤적 생성          |

</details>
</details>

---

<details id="aeirobot_msgs">
<summary><strong>3. aeirobot_msgs</strong></summary>

### 주요 액션
| `이름`    | 설명 |
| --------- | ---- |
| `Command` | -    |

### 주요 메시지
| `이름`              | 설명 |
| ------------------- | ---- |
| `Command`           | -    |
| `JointCommand`      | -    |
| `JointCommandArray` | -    |
| `PointArray`        | -    |
| `PoseXYZRPY`        | -    |

</details>

---

<details id="aeirobot_robotics">
<summary><strong>4. aeirobot_robotics</strong></summary>

로봇 기구학·변환 유틸리티를 모듈화한 C++ 헤더 라이브러리.

<!-- kinematics -->
<details id="kinematics">
<summary><strong>├─ kinematics</strong></summary>

#### 6dof_leg_kinematics.h
> 헤더 설명:
>6축 다리의 ik와 관련된 헤더

| `함수`                                | 설명                               |
| ------------------------------------- | ---------------------------------- |
| `ReadKinematicsYaml`                  | YAML에서 링크·조인트 파라미터 로드 |
| `CalcForwardKinematics`               | 전향 기구학 계산                   |
| `ComputeInverseKinematics`            | 다리 IK 계산                       |
| `ComputeInverseKinematicsForLeftLeg`  | 왼다리 반복 IK                     |
| `ComputeInverseKinematicsForRightLeg` | 오른다리 반복 IK                   |
| `KinematicsGraig`                     | DH 파라미터 초기화                 |
| `LoadRobotModel`                      | URDF 모델 Pinocchio로 로드         |

#### 7dof_arm_kinematics.h
> 헤더 설명:
>7축 팔의 ik와 관련된 헤더

| `함수`                  | 설명                        |
| ----------------------- | --------------------------- |
| `CalcInverseKinematics` | 7-DOF 팔 역기구학 계산      |
| `CalcForwardKinematics` | 각도로 엔드이펙터 Pose 계산 |

#### kinematics_define.h
> 헤더 설명:
>하체 ID 매크로 헤더

| `매크로`         | 값  | 설명                |
| ---------------- | --- | ------------------- |
| `ALL_JOINT_ID`   | 37  | 전체 조인트 수      |
| `ID_R_LEG_START` | 14  | 오른쪽 다리 시작 ID |
| `ID_L_LEG_START` | 13  | 왼쪽 다리 시작 ID   |

#### link_data.h
> 헤더 설명:
>LinkData 정의 헤더

| `함수`      | 설명                   |
| ----------- | ---------------------- |
| `LinkData`  | 링크 메타데이터 초기화 |
| `~LinkData` | 소멸자                 |

</details>

<!-- linear_rotary_converter -->
<details id="linear_rotary_converter">
<summary><strong>└─ linear_rotary_converter</strong></summary>

#### linear_rotary_converter.hpp
> 헤더 설명:
>리니어 모터를 가상조인트로 변환 해주는 헤더

| `함수`                    | 설명                              |
| ------------------------- | --------------------------------- |
| `CalculateKneeLinearMap`  | 선형 액추에이터 → 무릎 변환       |
| `CalculateHipLinearMap`   | 2개 액추에이터 → 골반 변환        |
| `CalculateAnkleLinearMap` | 2개 액추에이터 → 발목 변환        |
| `MapLinearPosKnee`        | 무릎 Pitch → 액추에이터 길이 변환 |
| `MapLinearPosPelvis`      | 골반 Pitch·Roll → 액추에이터 변환 |
| `MapLinearPosAnkle`       | 발목 Pitch·Roll → 액추에이터 변환 |
| `MapLinearEffortPelvis`   | 골반 토크 → 액추에이터 힘 변환    |
| `MapLinearEffortAnkle`    | 발목 토크 → 액추에이터 힘 변환    |
| `MapLinearEffortKnee`     | 무릎 토크 → 액추에이터 힘 변환    |

</details>
</details>

---

<details id="aeirobot_state_msgs">
<summary><strong>5. aeirobot_state_msgs</strong></summary>

### 주요 메시지
| `이름`           | 설명 |
| ---------------- | ---- |
| `EndpointState`  | -    |
| `FsrState`       | -    |
| `RobotState`     | -    |
| `Sensors`        | -    |
| `StateEstimator` | -    |
| `WholeBodyLog`   | -    |

</details>

---

<details id="aeirobot_toolbox_module">
<summary><strong>6. aeirobot_toolbox</strong></summary>

ROS 2 노드 개발에 유용한 헬퍼·유틸리티 헤더 모음.

#### basic_tools.hpp
> 헤더 설명:
>개발에 유용한 헬퍼·유틸리티 헤더

| `이름`                   | 설명                        |
| ------------------------ | --------------------------- |
| `IsNanOrInf`             | NaN/Inf 여부 검사           |
| `Clamp`                  | 값 범위 클램핑              |
| `SplitString`            | 구분자 문자열 분할          |
| `GetParameter`           | YAML 파라미터 읽기          |
| `PrintMessage`           | ROS2 컬러 메시지 출력       |
| `PRINT_COLOR`            | 색상 enum                   |
| `operator<<`             | 컬러 로그 연산자            |
| `ConvertStringToColor`   | 문자열 → PRINT_COLOR 변환   |
| `PelvisMode`             | 보행 모델 플래그            |
| `ROS_*_STREAM`           | 컬러 로그 매크로            |
| `PrintMatrixSizeAndType` | Matrix의 크기와 자료형 출력 |

#### command_protocol.hpp
> 헤더 설명:
>커맨드 명령 프로토콜 헤더

| `이름`             | 설명             |
| ------------------ | ---------------- |
| `FloatArrToString` | 벡터 → 문자열    |
| `CommandToString`  | 메시지 → 문자열  |
| `StyleToString`    | style → 문자열   |
| `ValueToString`    | value → 문자열   |
| `StringToCommand`  | 문자열 → Command |
| `StringToStyle`    | 문자열 → Style   |

#### aeirobot_pid_control.h
> 헤더 설명:
>PD, PID 제어기 헤더

| `이름`             | 설명                  |
| ------------------ | --------------------- |
| `PDController`     | PD 컨트롤러           |
| `~PDController`    | 소멸자                |
| `GetFeedBack`      | PD 제어 출력          |
| `PIDController`    | PID 컨트롤러          |
| `~PIDController`   | 소멸자                |
| `SetPidGains`      | PID 이득 런타임 설정  |
| `ResetPidIntegral` | 적분 오차 리셋        |
| `PidProcess`       | PID 연산 후 제어 출력 |

#### aeirobot_signal_processing.h
> 헤더 설명:
>필터링 관련 헤더

| `이름`                      | 설명                 |
| --------------------------- | -------------------- |
| `Initialize`                | 이동평균 버퍼 초기화 |
| `SetBufferSize`             | 버퍼 크기 변경       |
| `GetBufferSize`             | 윈도우 크기 반환     |
| `GetFilteredOutput`         | 필터 결과 반환       |
| `ApplyMovingAverageFilter`  | 이동평균 계산        |
| `LowPassFilter::initialize` | LPF 초기화           |
| `SetCutOffFrequency`        | 차단 주파수 갱신     |
| `GetCutOffFrequency`        | 차단 주파수 반환     |
| `GetPreviousOutput`         | 이전 필터 결과 반환   |
| `CalculateAlpha`            | LPF 내부 계수 계산   |
| `ScalarEKF::update`         | EKF 갱신             |
| `ScalarEKF::predict`        | EKF 예측             |
| `OneEuroFilter::Initialize` | 원 유로 필터 초기화     |
| `GetFilteredOutput`         | 원 유로 필터 결과 반환   |

#### qos_profiles.hpp
> 헤더 설명:
>QoS 설정 프로파일 헤더

| `이름`                  | 설명                 |
| ----------------------- | -------------------- |
| `qos_control_profile`   | 제어 루프 QoS        |
| `qos_sensor_profile`    | 센서 스트림 QoS      |
| `qos_topic_profile`     | 상태/명령 토픽 QoS   |
| `qos_service_profile`   | 서비스 요청·응답 QoS |
| `qos_action_profile`    | 액션 통신 QoS        |
| `qos_parameter_profile` | 파라미터 서버 QoS    |

#### ros_manager.hpp
> 헤더 설명:
>ROS 2 노드 개발에 유용한 헬퍼 헤더

| `이름`             | 설명                 |
| ------------------ | -------------------- |
| `Publish`          | 메시지 발행          |
| `AddPublisher`     | Publisher 생성       |
| `GetPublisher`     | Publisher 핸들 반환  |
| `RemovePublisher`  | Publisher 제거       |
| `AddSubscriber`    | Subscriber 생성      |
| `GetSubscriber`    | Subscriber 핸들 반환 |
| `RemoveSubscriber` | Subscriber 제거      |
| `AddService`       | 서비스 서버 생성     |
| `GetService`       | 서비스 핸들 반환     |
| `RemoveService`    | 서비스 서버 제거     |
| `AddClient`        | 클라이언트 생성      |
| `GetClient`        | 클라이언트 핸들 반환 |
| `RemoveClient`     | 클라이언트 제거      |
| `AddWallTimer`     | 타이머 생성          |
| `RemoveWallTimer`  | 타이머 제거          |
| `RosManager`       | ROS2 노드 초기화     |

#### vector3.hpp
> 헤더 설명:
>vector3 클래스 정의 헤더

| `이름`      | 설명                    |
| ----------- | ----------------------- |
| `Vector3`   | 생성자 (0,0,0 초기화)   |
| `Set`       | 값 갱신                 |
| `IsIn`      | 사각형 포함 검사        |
| `operator=` | 다양한 타입 대입 연산   |
| `operator+` | 벡터 덧셈               |
| `operator-` | 벡터 뺄셈               |
| `operator*` | 스칼라 곱셈             |
| `operator/` | 스칼라 나눗셈           |
| `Dot`       | 내적 계산               |
| `AngleTo`   | 방향각 반환             |
| `AngleAToB` | 두 점 방향각(프리 함수) |

</details>

<details id="rosout_logger">
<summary><strong>├─ rosout_logger</strong></summary>

> ROS 2 로깅 시스템에서 발생하는 모든 로그 메시지를 체계적으로 파일에 저장하는 유틸리티.

자세한 내용은 [rosout_logger.md](./rosout_logger.md) 문서를 참조하십시오.

</details>
