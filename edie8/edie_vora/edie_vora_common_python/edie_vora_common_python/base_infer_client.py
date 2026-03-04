#!/usr/bin/env python3
import threading
from typing import Optional, Callable
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Image
from edie_msgs.srv import SwitchVoraMode
from edie_msgs.action import HumanVoraInference
from edie_msgs.action import EmotionVoraInference

from edie_msgs.msg import HumanDetection2DArray, EmotionDetection2DArray
import json

class BaseInferClient(Node):
    """공통 추론 클라이언트 베이스
    - 이미지 구독
    - /vora/switch_mode, /vora/human_detection_infer, /vora/emotion_detection_infer 사용
    - inflight(중복 goal 방지), throttle(ns), feedback 로깅
    - 결과는 HandleHumanInferenceResult / HandleEmotionInferenceResult 훅으로 전달
    - 활성/비활성 가드(active 파라미터), image_topic 변경 시 동적 재구독
    - 매니저의 vora/model_state를 구독하여 모드 자동 동기화
    """

    IMAGE_TOPIC_PARAM = 'image_topic'
    MODEL_PARAM = 'model_path'

    def __init__(self, node_name: str, mode: str = 'human'):
        super().__init__(node_name)

        # ── params
        self.declare_parameter(self.IMAGE_TOPIC_PARAM, '/image_topic')
        self.declare_parameter(self.MODEL_PARAM, '/path/to/model.model')
        self.declare_parameter('active', True)
        # 목표 주파수(Hz)로 스로틀 자동 계산( >0이어야 우선)
        self.declare_parameter('infer_target_hz', 0.0)
        self.declare_parameter('infer_throttle_ms', 0.0)

        # ── mode (human | emotion)
        if mode not in ('human', 'emotion'):
            self.get_logger().warn(f"Unknown mode '{mode}', fallback to 'human'")
            mode = 'human'
        self._mode = mode

        # ── clients
        self._cli_switch_mode = self.create_client(SwitchVoraMode, 'vora/switch_mode')
        self._cli_action_human   = ActionClient(self, HumanVoraInference, 'vora/human_detection_infer')
        self._cli_action_emotion = ActionClient(self, EmotionVoraInference, 'vora/emotion_detection_infer')
        
        # ── subscription (동적 재구독 지원)
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self._img_qos = img_qos
        self._image_sub = None
        self._image_topic: str = self.get_parameter(self.IMAGE_TOPIC_PARAM).value
        # self.RecreateImageSubscription(self._image_topic)

        # 매니저 모델 상태 구독(모드 동기화)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        try:
            self.sub_mode_state = self.create_subscription(
                String,
                'vora/mode_state',
                self.ModeStateCallback,
                state_qos
            )
        except Exception:
            self.sub_mode_state = None

        # ── state
        self._goal_handle = None
        self._inprogress = False
        # 전환 중: 이미지 처리/goal 전송 차단
        self._switching = False
        # 활성 가드
        self._active: bool = bool(self.get_parameter('active').value)

        # ── 콜백 그룹 / 실행 동시성 
        self._cbg = ReentrantCallbackGroup()
        
        self._lock = threading.Lock()
        self._last_stamp_ns = 0
        self._throttle_ns = self.ComputeThrottleTime()
         # 최신 프레임 1-슬롯 파이프라인용 버퍼
        self._last_image: Optional[Image] = None

        # 파라미터 변경 콜백: image_topic/active 반영
        self.add_on_set_parameters_callback(self.ApplyParameterUpdates)

        # ── bootstrap: load model once ready
        self._bootstrap_timer = self.create_timer(0.1, self.BootStrapOnce)

        # ── 펌프 타이머 (작은 주기, 예: 1ms)
        #  - ImageCallback은 최신 프레임만 저장
        #  - 전송은 여기서 busy가 아닐 때만 수행
        self._pump_timer = self.create_timer(0.001, self.PumpOnce, callback_group=self._cbg)

        self.get_logger().info(f"{node_name} ready (mode={self._mode})")

    def ComputeThrottleTime(self) -> int:
        """infer_target_hz(>0) 우선, 그 외 infer_throttle_ms(ms) 사용"""
        try:
            target_hz = float(self.get_parameter('infer_target_hz').value)
            if target_hz and target_hz > 0.0:
                return int(1_000_000_000 / target_hz)
        except Exception:
            pass
        
        # fallback: ms 파라미터
        try:
            ms_val = float(self.get_parameter('infer_throttle_ms').value)
        except Exception:
            ms_val = 0.0
        if ms_val < 0.0:
            ms_val = 0.0
        return int(ms_val * 1_000_000)

    # ===== manager model_state sync =====
    def ModeStateCallback(self, msg: String):
        try:
            data = json.loads(msg.data)
            kind = str(data.get('kind', '')).strip()
            if kind not in ('human', 'emotion'):
                return

            if self._mode == kind:
                # 내 모드일 때 → 활성화
                self.set_parameters([
                    Parameter('active', value=True),
                    Parameter(self.IMAGE_TOPIC_PARAM, value='/edie8/vision/image_raw'),
                ])
                self.get_logger().info(f"[ModeStateCallback/if] MODE: {self._mode}. kind: {kind}. Active: {self._active}. Image Topic: {self._image_topic}")
            else:
                # 반대 모드일 때 → 비활성화
                self.set_parameters([
                    Parameter('active', value=False),
                    Parameter(self.IMAGE_TOPIC_PARAM, value='/unused'),
                ])
                self.get_logger().info(f"[ModeStateCallback/else] MODE: {self._mode}. kind: {kind}. Active: {self._active}. Image Topic: {self._image_topic}")

            # 진행 중 goal 있으면 cancel
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f'cancel failed: {e}')
                finally:
                    self._goal_handle = None
                    with self._lock:
                        self._inprogress = False  # or _inprogress

        except Exception as e:
            self.get_logger().warn(f"mode_state parse failed: {e}")

    # ===== parameter change handler =====
    def ApplyParameterUpdates(self, params: list[Parameter]) -> SetParametersResult:
        try:
            need_resub = False
            new_topic = self._image_topic

            for p in params:
                if p.name == self.IMAGE_TOPIC_PARAM and p.type_ in (Parameter.Type.STRING, Parameter.Type.NOT_SET):
                    if p.value and isinstance(p.value, str) and p.value != self._image_topic:
                        new_topic = p.value
                        need_resub = True
                elif p.name == 'active' and p.type_ in (Parameter.Type.BOOL, Parameter.Type.NOT_SET):
                    if p.value is not None:
                        self._active = bool(p.value)
                        self.get_logger().info(f"param 'active' -> {self._active}")
                elif p.name == 'infer_throttle_ms' and p.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE, Parameter.Type.NOT_SET):
                    self._throttle_ns = self.ComputeThrottleTime()
                    self.get_logger().info(f"param 'infer_throttle_ms' -> {self.get_parameter('infer_throttle_ms').value} (ns={self._throttle_ns})")
                elif p.name == 'infer_target_hz' and p.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE, Parameter.Type.NOT_SET):
                    self._throttle_ns = self.ComputeThrottleTime()
                    self.get_logger().info(f"param 'infer_target_hz' -> {self.get_parameter('infer_target_hz').value} (ns={self._throttle_ns})")
            
            if need_resub:
                self.get_logger().info(f"param '{self.IMAGE_TOPIC_PARAM}' -> {new_topic} (resubscribe)")
                self.RecreateImageSubscription(new_topic)

            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f'param change failed: {e}')

            return SetParametersResult(successful=False, reason=str(e))

    def RecreateImageSubscription(self, topic: str) -> None:
        try:
            if self._image_sub is not None:
                try:
                    self.destroy_subscription(self._image_sub)
                except Exception:
                    pass
            self._image_topic = topic
            self._image_sub = self.create_subscription(
                    Image, self._image_topic, self.ImageCallback, 
                    self._img_qos, callback_group=self._cbg
            )
        except Exception as e:
            self.get_logger().error(f'RecreateImageSubscription failed: {e}')

    # ===== bootstrap =====
    def BootStrapOnce(self):
        """노드 기동 직후 1회 정렬용.
        - 초기 active=False 이면 아무 것도 하지 않고 타이머 종료
        - active=True 이면 매니저의 /vora/switch_mode 로 '내 모드'를 요청
        """
        # 1) 이 노드가 비활성으로 시작하면, 매니저 신호를 기다리기만 하고 종료
        if not getattr(self, '_active', False):
            if hasattr(self, '_bootstrap_timer'):
                self._bootstrap_timer.cancel()
            return
            
        # 2) 스위치 서비스 준비 대기 (아직 안 떴으면 다음 타이머 틱에서 재시도)
        if not self._cli_switch_mode.service_is_ready():
            return
        if not self._cli_switch_mode.wait_for_service(timeout_sec=0.1):
            return

        # 3) 모드로 전환 요청 (예: 'human' 또는 'emotion')
        req = SwitchVoraMode.Request()
        req.target = str(self._mode)

        fut = self._cli_switch_mode.call_async(req)
        fut.add_done_callback(lambda f: self.HandleSwitchVoraMode(f, req.target))

        # 4) 1회만 실행 후 타이머 종료
        if hasattr(self, '_bootstrap_timer'):
            self._bootstrap_timer.cancel()

    def HandleSwitchVoraMode(self, fut, target: str):
        try:
            res = fut.result()
            self.get_logger().info(
                f"[bootstrap] requested {target}: ok={res.ok}, msg={res.message}"
            )
        except Exception as e:
            self.get_logger().error(f"[bootstrap] switch call failed: {e}")

    # ===== image callback → 최신 프레임 저장만 =====
    def ImageCallback(self, msg: Image):
        # 최신 이미지 저장(시각화 용도)
        self._last_image = msg

    # busy가 아닐 때만 최신 프레임을 잡아 액션 전송, 완료 시 busy 해제
    def PumpOnce(self):
        # 전환 중 및 비활성 상태에는 goal 전송 차단
        if self._switching or not self._active:
            return
        if self._last_image is None:
            return
        
        # 서버 준비 대기는 여기서(필요하면 유지)
        if not self._cli_action_human.server_is_ready() or not self._cli_action_emotion.server_is_ready():
            return

        # === 새 프레임 판단 & 스로틀 ===
        stamp_ns = (self._last_image.header.stamp.sec*1_000_000_000 
                    + self._last_image.header.stamp.nanosec)
        if self._throttle_ns > 0:
            if stamp_ns - self._last_stamp_ns < self._throttle_ns:
                return
        # 스로틀 0이면 같은 프레임 재전송 방지
        else:
            if stamp_ns <= self._last_stamp_ns:
                return
        
        with self._lock:
            if self._inprogress:
                return
            self._inprogress = True

        # 모드에 따른 goal/client/result_handler 선택
        if self._mode == 'human':
            goal = HumanVoraInference.Goal()
            goal.image = self._last_image
            client  = self._cli_action_human
            result_handler = self.HumanResultCallback
        else:
            goal = EmotionVoraInference.Goal()
            goal.image = self._last_image
            client  = self._cli_action_emotion
            result_handler = self.EmotionResultCallback

        # ── goal 전송
        send_future = client.send_goal_async(
            goal,
            feedback_callback=self.HandleActionFeedback
        )
        send_future.add_done_callback(
            partial(self.HandleActionGoalSent, mode=self._mode, result_cb=result_handler)
        )

        # 마지막으로 보낸 프레임의 stamp 갱신 (스로틀 0인 경우에도 중복 방지에 사용)
        self._last_stamp_ns = stamp_ns

    def HandleActionGoalSent(self, fut, mode: str, result_cb: Callable):
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().warn(f'Infer goal rejected ({mode})')
            with self._lock:
                self._inprogress = False
            return
        
        # 🔧 보완: 핸들을 저장해 두면 모드 전환 시 cancel 가능
        self._goal_handle = gh
        res_future = gh.get_result_async()
        res_future.add_done_callback(result_cb)

    def HandleActionFeedback(self, fb):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = getattr(self, '_last_feedback_log_ns', 0)
        if now_ns - last_ns >= 500_000_000:
            # self.get_logger().info(f"[infer] {fb.feedback.progress:.2f} {fb.feedback.stage}")
            self._last_feedback_log_ns = now_ns

    def HumanResultCallback(self, fut):
        try:
            res = fut.result().result
            self.HandleHumanInferenceResult(res.human_detections)
        except Exception as e:
            self.get_logger().error(f'infer result error (human): {e}')
        finally:
            # 어떤 상태로 끝나든 핸들/플래그 해제
            self._goal_handle = None
            with self._lock:
                self._inprogress = False

    def EmotionResultCallback(self, fut):
        try:
            res = fut.result().result
            self.HandleEmotionInferenceResult(res.emotion_detections)
        except Exception as e:
            self.get_logger().error(f'infer result error (emotion): {e}')
        finally:
            # 어떤 상태로 끝나든 핸들/플래그 해제
            self._goal_handle = None
            with self._lock:
                self._inprogress = False
    
    # ===== hooks =====
    def HandleHumanInferenceResult(self, dets: HumanDetection2DArray):
        """사람 검출 결과 훅. Human 모드 기본 구현은 HandleHumanInferenceResult 위임"""
        self.get_logger().info(f'[Base] detections: {len(dets.human_det)}')


    def HandleEmotionInferenceResult(self, dets: EmotionDetection2DArray):
        """감정 인식 결과 훅. 기본은 로깅만 수행"""
        try:
            n = len(getattr(dets, 'emotion_det', []))
        except Exception:
            n = 0
        self.get_logger().info(f'[Base] emotion_det: {n}')