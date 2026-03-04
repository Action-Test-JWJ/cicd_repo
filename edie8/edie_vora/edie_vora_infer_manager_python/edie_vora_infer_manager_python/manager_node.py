#!/usr/bin/env python3
from typing import List, Optional, Tuple
import json, os, cv2, math, time
import threading
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point32
from edie_msgs.srv import SwitchVoraMode
from edie_msgs.action import HumanVoraInference
from edie_msgs.action import EmotionVoraInference
from edie_msgs.msg import HumanDetection, HumanDetection2DArray
from edie_msgs.msg import EmotionDetection, EmotionDetection2DArray
from edie_msgs.msg import FaceDetection, FaceDetectionArray
from std_msgs.msg import String

from gtilib.native import module
from dlib import rectangle

# ----- 사람 인식
# detector.py: class SSDDetector(GtiModel) 기반 흐름 (GtiEvaluate + 14x14x30 후처리/NMS)
from edie_vora_infer_manager_python.detector import SSDDetector
from edie_vora_infer_manager_python.box import BoundingBox, IntersectBoundingBox
from edie_vora_infer_manager_python.gtilib import GtiModel, GtiTensor, GtiModelFactory, GtiDeviceException
from edie_vora_infer_manager_python.gyurinet.gtilib.util import convert_opencv2_img_to_gti_img
# ----- 감정 인식
from edie_vora_infer_manager_python.dlib_face_detector import DlibFaceDetector, DlibFaceDetectorResult

def BboxToMsg(bb: BoundingBox) -> HumanDetection:
    m = HumanDetection()
    m.label = bb.label
    m.score = float(bb.score)
    # ROI(픽셀) 변환
    x1, y1, x2, y2 = bb.x1, bb.y1, bb.x2, bb.y2
    if x2 < x1 or y2 < y1:
        m.roi.x_offset = m.roi.y_offset = m.roi.width = m.roi.height = 0
    else:
        m.roi.x_offset = max(0, int(x1))
        m.roi.y_offset = max(0, int(y1))
        m.roi.width    = max(0, int(x2 - x1))
        m.roi.height   = max(0, int(y2 - y1))
    m.roi.do_rectify = False

    return m

# 감정 추론용: dlib rectangle → RegionOfInterest 변환
def RectangleToRoi(r: rectangle) -> RegionOfInterest:
    roi = RegionOfInterest()
    x1 = int(max(0, r.left()))
    y1 = int(max(0, r.top()))
    x2 = int(max(0, r.right()))
    y2 = int(max(0, r.bottom()))
    roi.x_offset = x1
    roi.y_offset = y1
    roi.width = max(0, x2 - x1)
    roi.height = max(0, y2 - y1)
    roi.do_rectify = False
    return roi

class ManagerNode(Node):
    def __init__(self):
        super().__init__('vora_infer_manager_python_node')
        self._bridge = CvBridge()
        
        # 환경변수(원하면 파라미터화)
        os.environ.setdefault('GTISDKPATH', '/opt/gti-sdk')
        os.environ.setdefault('LD_LIBRARY_PATH', '/opt/gti-sdk/Lib/Linux/x86_64')
    
        gtisdk = os.environ.get('GTISDKPATH', '/opt/gti-sdk')
        self.emotion_path = os.path.join(gtisdk, 'models', 'emotion_detection', '2803_vgg16_full.model')
        self.human_path = os.path.join(gtisdk, 'models', 'human_detection', 'gti_gnetdet_2803.model')
    
        if not os.path.exists(self.emotion_path):
            self.get_logger().warn(f"[init] emotion_model_path not found: {self.emotion_path}")
        if not os.path.exists(self.human_path):
            self.get_logger().warn(f"[init] human_model_path not found: {self.human_path}")

        # 상태
        self._gti_model: GtiModel = None
        self._human_detector: SSDDetector = None            # 사람 인식 디텍터 -> GTI 모델에 종속됨
        self._face_detector = DlibFaceDetector(ratio=2)     # 얼굴 검출 디텍터_ dlib 사용 -> dlib 기반(CPU 라이브러리) 사용
        
        self._loaded_path: str = ""
        # 현재 로드된 모드 종류: 'human' or 'emotion'
        self._mode: str = ''
        # 전환 중 플래그
        self._switching: bool = False
        # 인퍼런스 동시성 제어 (전환 시 안전 언로드를 위해 in-flight 추적)
        self._inflight: int = 0
        self._inflight_lock = threading.Lock()
        self._gti_eval_lock = threading.Lock()
        # 인퍼런스 동시성 제어 (전환 시 안전 언로드를 위해 in-flight 추적)
        self._inflight: int = 0
        self._inflight_lock = threading.Lock()
        self._gti_eval_lock = threading.Lock()
        
        # 이미지 크기
        self._gti_w, self._gti_h = 224, 224
        
        # 퍼블리셔: 얼굴 검출 결과(랜드마크 포함)
        self.pub_faces = self.create_publisher(FaceDetectionArray, 'vora/face_detections', 10)
        # 퍼블리셔: 모드 상태
        state_qos = rclpy.qos.QoSProfile(depth=1)
        state_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        state_qos.durability  = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_mode_state = self.create_publisher(String, 'vora/mode_state', state_qos)
        
        # 서브스크라이버: 모드 전환 요청
        self._last_topic_call = 0.0           # 디바운스 최근 전환 시각
        self._switch_cooldown = 0.5           # 디바운스 쿨다운(sec)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.sub_mode_switch_request = self.create_subscription(String, 
                                            '/edie8/vora/switch_mode_request', self.SwitchModeRequestCallback, 10)

        # 서비스
        self._srv_switch = self.create_service(SwitchVoraMode, 'vora/switch_mode', self.SwitchModeSrvCallback)

        # 액션 서버 (인간 감지)
        self._action_human = ActionServer(
            self,
            HumanVoraInference,
            'vora/human_detection_infer',
            execute_callback=self.ExecuteHumanDetection,
            goal_callback=self.GoalHumanDetection,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
        )

        # 액션 서버 (감정 추론)
        self._action_emotion = ActionServer(
            self,
            EmotionVoraInference,
            'vora/emotion_detection_infer',
            execute_callback=self.ExecuteEmotionDetection,
            goal_callback=self.GoalEmotionDetection,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
        )

        self.declare_parameter('warmup_runs', 4)        # 2 -> 4
        self.declare_parameter('switch_wait_timeout_sec', 2.0)
        self.declare_parameter('switch_grace_sleep_ms', 100)
        self.declare_parameter('switch_wait_timeout_sec', 2.0)
        self.declare_parameter('switch_grace_sleep_ms', 100)

        # 프로파일 로그 on/off (true면 각 인퍼런스마다 단계별 소요시간(ms) 로그)
        self.declare_parameter('profile_infer', False)
        self._prof_enabled = bool(self.get_parameter('profile_infer').value)
        self.get_logger().info(f'[_init__] profile_infer: {self._prof_enabled}')

        self.get_logger().info('VORA manager ready')
    
    # ===== helpers =====
    def EnterInference(self) -> bool:
        """인퍼런스 시작 가드. 전환 중이면 거부, 아니면 in-flight 카운트 증가."""
        with self._inflight_lock:
            if self._switching:
                return False
            self._inflight += 1
            return True

    def LeaveInference(self) -> None:
        """인퍼런스 종료 시 in-flight 카운트 감소(하한 0)."""
        with self._inflight_lock:
            if self._inflight > 0:
                self._inflight -= 1
                
    def PublishModeState(self):
        msg = String()
        payload = {
            'kind': self._mode,
            'path': self._loaded_path
        }
        msg.data = json.dumps(payload)
        try:
            self.pub_mode_state.publish(msg)
        except Exception:
            pass

    def GoalHumanDetection(self, goal):
        if self._switching:
            self.get_logger().warn('reject human goal: switching in progress')
            return GoalResponse.REJECT
        if self._mode != 'human':
            self.get_logger().warn('reject human goal: current model is not human')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def GoalEmotionDetection(self, goal):
        if self._switching:
            self.get_logger().warn('reject emotion goal: switching in progress')
            return GoalResponse.REJECT
        if self._mode != 'emotion':
            self.get_logger().warn('reject emotion goal: current model is not emotion')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def SwitchModeRequestCallback(self, msg: String):
        target = (msg.data or '').strip().lower()
        
        # 대상 모델 경로 선택
        if target not in ('human','emotion'):
            self.get_logger().warn(f'invalid switch target: {target}')
            return
        elif target == 'human':
            path = self.human_path
        elif target == 'emotion':
            path = self.emotion_path

        # 1️⃣ 전환 중이면 무시
        if self._switching:
            self.get_logger().debug('switching in progress, ignore request')
            return

        # 2️⃣ 같은 모드 요청은 무시
        if target == self._mode:
            self.get_logger().debug(f'already in {target}, ignore duplicate request')
            return

        # 3️⃣ 전환 직후 쿨다운 내 요청은 무시
        # 디바운스
        now = self._steady_clock.now().nanoseconds / 1e9
        if now - self._last_topic_call < self._switch_cooldown:
            self.get_logger().debug(f'switch cooldown active, ignore')
            return
        self._last_topic_call = now

        # 4️⃣ 전환 실행
        model_path = path
        # 서비스 핸들러와 동일한 내부 루틴 호출(직접 or 공용 함수로 분리)
        ok, message = self.ExecuteSwitchMode(model_path, target)
        if ok:
            self._last_topic_call = now
            self.get_logger().info(f'[토픽 콜백함수] switched to {target}')
        else:
            self.get_logger().warn(f'[토픽 콜백함수] switch to {target} failed: {message}')

    # ===== services =====
    def ExecuteSwitchMode(self, model_path: str, target: str) -> Tuple[bool, str]:
        """
        실제 GTI 모델 언로드→ GTI 모델 로드→ 모드 세팅→ 웜업→ 모드 상태발행까지 수행.
        target: 'human' | 'emotion'
        """
        # 1) re-entrancy guard
        if self._switching:
            return False, 'switching in progress'
        self._switching = True

        # 2) 기존 모드에 할당된 GTI 모델 언로드 및 새로운 모드에 GTI 모듈 할당 & 로드
        try:
            # 전환 시작: 신규 인퍼런스 수락 금지, 진행 중 인퍼런스 drain 대기
            try:
                wait_timeout = float(self.get_parameter('switch_wait_timeout_sec').value)
            except Exception:
                wait_timeout = 2.0
            deadline = time.monotonic() + max(0.0, wait_timeout)

            while True:
                with self._inflight_lock:
                    curr = self._inflight
                if curr == 0:
                    break
                if time.monotonic() >= deadline:
                    self.get_logger().warn(f'[ExecuteSwitchMode] wait for inflight timeout (inflight={curr})')
                    break
                time.sleep(0.01)

            # 전환 시작: 신규 인퍼런스 수락 금지, 진행 중 인퍼런스 drain 대기
            try:
                wait_timeout = float(self.get_parameter('switch_wait_timeout_sec').value)
            except Exception:
                wait_timeout = 2.0
            deadline = time.monotonic() + max(0.0, wait_timeout)

            while True:
                with self._inflight_lock:
                    curr = self._inflight
                if curr == 0:
                    break
                if time.monotonic() >= deadline:
                    self.get_logger().warn(f'[ExecuteSwitchMode] wait for inflight timeout (inflight={curr})')
                    break
                time.sleep(0.01)

            # 멱득성(idempotence): 같은 입력(같은 모델 경로, 같은 모드)이 반복해서 들어왔을 때 불필요한 무거운 연산 수행 X
            if self._gti_model is not None and self._loaded_path == model_path and self._mode == target:
                self.get_logger().info(f'[ExecuteSwitchMode] already loaded: {model_path} (mode={target})')
                # 상태는 한 번 더 내보내 안전하게 동기화
                self.PublishModeState()
                return True, f'already {target} ({model_path})'

            # 1. 언로드
            # 1. 언로드
            if self._gti_model is not None:
                try:
                    self._gti_model.GtiDestroyModel()
                except AttributeError:
                    try:
                        module.GtiDestroyModel(self._gti_model.ptr)
                    except Exception:
                        pass
                except Exception:
                    pass


                self._gti_model = None
                self._human_detector = None  # 사람모드 디텍터는 모델에 종속되므로 같이 초기화
                self.get_logger().info(f'[ExecuteSwitchMode] 기존 Model unloaded: {self._loaded_path}')

                # 디바이스(USB) 해제 안정화를 위한 그레이스 슬립
                try:
                    grace_ms = int(self.get_parameter('switch_grace_sleep_ms').value)
                except Exception:
                    grace_ms = 100
                if grace_ms > 0:
                    time.sleep(grace_ms / 1000.0)

            # 2. 로드
                # 디바이스(USB) 해제 안정화를 위한 그레이스 슬립
                try:
                    grace_ms = int(self.get_parameter('switch_grace_sleep_ms').value)
                except Exception:
                    grace_ms = 100
                if grace_ms > 0:
                    time.sleep(grace_ms / 1000.0)

            # 2. 로드
            self._loaded_path = model_path
            self._gti_model = GtiModelFactory.create_by_file(self._loaded_path)
            self.get_logger().info(f'[ExecuteSwitchMode] 새로운 Model loaded: {self._loaded_path}')

            
            # 모드 세팅
            target = (target or '').strip().lower()
            if target not in ('human', 'emotion'):
                raise RuntimeError(f'invalid target: {target}')
            self._mode = target

        # 3) 웜업 추론
            try:
                warmup = int(self.get_parameter('warmup_runs').value)
            except Exception:
                warmup = 2
            self.get_logger().info(f'[LoadVoraModelCallback] warmup: {warmup} runs')

            if self._mode == 'emotion' and warmup > 0:
            # if warmup > 0:
            if self._mode == 'emotion' and warmup > 0:
            # if warmup > 0:
                dummy = np.zeros((self._gti_h, self._gti_w, 3), dtype=np.uint8)
                gti_img = convert_opencv2_img_to_gti_img(dummy, self._gti_w, self._gti_h)
                for _ in range(warmup):
                    with self._gti_eval_lock:
                        _ = self._gti_model.GtiImageEvaluate(gti_img, self._gti_w, self._gti_h, 3)
                    with self._gti_eval_lock:
                        _ = self._gti_model.GtiImageEvaluate(gti_img, self._gti_w, self._gti_h, 3)

            # 사람 모드일 때만 디텍터 준비
            if self._mode == 'human':
                self._human_detector = SSDDetector(self._gti_model)
            else:
                self._human_detector = None
            
        # 4) 상태 발행 (현재 코드는 JSON String으로 PublishModeState 구현됨)
            self.PublishModeState()

            ok = True
            message = f'loaded: {self._loaded_path} (kind={self._mode})'
            return ok, message

        except Exception as e:
            self.get_logger().error(f'[ExecuteSwitchMode] error: {e}')
            self.get_logger().error(f'[ExecuteSwitchMode] error: {e}')
            # 실패 시 내부 상태를 최대한 일관되게
            self._gti_model = None
            self._human_detector = None
            self._mode = ''
            self.PublishModeState()

            ok = False
            message = f'load failed: {e}'
            return ok, message

        finally:
            self._switching = False
    
    def SwitchModeSrvCallback(self, req: SwitchVoraMode.Request, res: SwitchVoraMode.Response):
        target = (req.target or '').strip().lower()
        
        # 대상 모델 경로 선택
        if target == 'human':
            path = self.human_path
        elif target == 'emotion':
            path = self.emotion_path
        else:
            res.ok = False
            res.message = f'invalid target={req.target}'
            return res

        # 현재와 동일 인식 모드면 NOP로 처리(빠른 OK)
        if self._mode == target and self._gti_model is not None:
            self.get_logger().info(f'[서비스 콜백함수] already in {target}')
            self.PublishModeState()
            res.ok = True
            res.message = f'already {target}'
            return res

        model_path = path
        ok, msg = self.ExecuteSwitchMode(model_path, target)
        if ok:
            self.get_logger().info(f'[서비스 콜백함수] switched to {target}')
        else:
            self.get_logger().warn(f'[서비스 콜백함수] switch to {target} failed: {msg}')

        res.ok = ok
        res.message = msg
        return res
    
    # ===== action =====
    ## 반환값(res)는 액션 클라이언트 내 '_on_result_done'로 넘김
    async def ExecuteHumanDetection(self, goal_handle):
        fb = HumanVoraInference.Feedback()
        res = HumanVoraInference.Result()
        msg: Image = goal_handle.request.image
        
        # 전환 중 가드
        if self._switching:
            self.get_logger().warn('switching in progress; abort human')
            res.human_detections = HumanDetection2DArray()
            res.human_detections.header = msg.header
            goal_handle.abort()
            return res
        
        # 모델 종류 체크: 사람 모델이 아니면 빈 결과
        if self._mode != 'human':
            self.get_logger().warn('current model is not human; skip detection')
            res.human_detections = HumanDetection2DArray()
            res.human_detections.header = msg.header
            goal_handle.abort()
            return res
        # 정상 동작 확인 완.
        # self.get_logger().info(f'[ExecuteHumanDetection] Image received: {msg.width}x{msg.height}')
        # self.get_logger().info(f'[ExecuteHumanDetection] msg.header: {msg.header}')

        if self._human_detector is None:
            self.get_logger().warn('detector not loaded')
            res.human_detections = HumanDetection2DArray()
            res.human_detections.header = msg.header
            goal_handle.abort()
            return res
        # else:
        #     self.get_logger().info('detector loaded')

        # 인퍼런스 진입 가드 (전환 중 동시에 들어오는 경합 방지)
        if not self.EnterInference():
            self.get_logger().warn('switching in progress; reject human inference')
            res.human_detections = HumanDetection2DArray()
            res.human_detections.header = msg.header
            goal_handle.abort()
            return res

        try:
            fb.progress = 0.1; fb.stage = 'preprocess'; goal_handle.publish_feedback(fb)

            # 프로파일 시작 시간
            t0 = time.perf_counter() if self._prof_enabled else 0.0
            t_cv = t_prep = t_infer = t_post = 0.0

            # GTI 입력 생성
            ## 1) ROS Image → OpenCV
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if self._prof_enabled:
                    t_cv = time.perf_counter()
            except Exception as e:
                self.get_logger().error(f"cv_bridge convert failed: {e}")
                res.human_detections = HumanDetection2DArray()
                res.human_detections.header = msg.header
                goal_handle.abort()
                return res
                
            ## 2) OpenCV → GTI 이미지 버퍼
            gti_img: bytes = convert_opencv2_img_to_gti_img(frame, self._gti_w, self._gti_h)
            if self._prof_enabled:
                t_prep = time.perf_counter()
            
            # 추론 및 후처리(14x14x30 reshape & NMS) 후 BoundingBox 리스트 반환
            ## 3) 디텍터 호출 (GTI 버퍼, 원본 W/H)
            with self._gti_eval_lock:
                bb_list: List[BoundingBox] = self._human_detector.detect(
                    gti_img, msg.width, msg.height)
            with self._gti_eval_lock:
                bb_list: List[BoundingBox] = self._human_detector.detect(
                    gti_img, msg.width, msg.height)
            if self._prof_enabled:
                t_infer = time.perf_counter()

            fb.progress = 0.9; fb.stage = 'postprocess'; goal_handle.publish_feedback(fb)

            out = HumanDetection2DArray()
            out.header = msg.header
            # ------------------------------- 주의!! -------------------------------
            # bb_list에는 person 클래스 뿐만 아니라 detecting한 모든 클래스 들어있다.
            if bb_list:
                out.human_det = [BboxToMsg(bb) for bb in bb_list]

            res.human_detections = out

            # 프로파일 종료 시간
            if self._prof_enabled:
                t_post = time.perf_counter()
                total_ms = (t_post - t0) * 1000.0
                cv_ms    = (t_cv - t0) * 1000.0 if t_cv else 0.0
                prep_ms  = (t_prep - (t_cv if t_cv else t0)) * 1000.0 if t_prep else 0.0
                infer_ms = (t_infer - (t_prep if t_prep else (t_cv if t_cv else t0))) * 1000.0 if t_infer else 0.0
                post_ms  = (t_post - (t_infer if t_infer else (t_prep if t_prep else (t_cv if t_cv else t0)))) * 1000.0
                self.get_logger().info(
                    f"[prof human] total={total_ms:.2f}ms cv={cv_ms:.2f}ms prep={prep_ms:.2f}ms infer={infer_ms:.2f}ms post={post_ms:.2f}ms")

            fb.progress = 1.0; fb.stage = 'done'; goal_handle.publish_feedback(fb)
            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f'execute error: {e}')
            res.human_detections = HumanDetection2DArray()
            res.human_detections.header = msg.header
            goal_handle.abort()
        finally:
            self.LeaveInference()

        return res

    # 감정 추론 액션
    async def ExecuteEmotionDetection(self, goal_handle):
        fb = EmotionVoraInference.Feedback()
        res = EmotionVoraInference.Result()
        msg: Image = goal_handle.request.image

        # 전환 중 가드
        if self._switching:
            self.get_logger().warn('switching in progress; abort emotion')
            out = EmotionDetection2DArray(); out.header = msg.header
            res.emotion_detections = out
            goal_handle.abort()
            return res
        
        # 모델 종류 체크: 감정 모델이 아니면 빈 결과
        if self._mode != 'emotion':
            self.get_logger().warn('current model is not emotion; skip emotion inference')
            out = EmotionDetection2DArray()
            out.header = msg.header
            res.emotion_detections = out
            goal_handle.abort()
            return res
        
        if self._gti_model is None:
            self.get_logger().warn('gti model not loaded')
            out = EmotionDetection2DArray()
            out.header = msg.header
            res.emotion_detections = out
            goal_handle.abort()
            return res

        # 인퍼런스 진입 가드
        if not self.EnterInference():
            self.get_logger().warn('switching in progress; reject emotion inference')
            out = EmotionDetection2DArray(); out.header = msg.header
            res.emotion_detections = out
            goal_handle.abort()
            return res

        try:
            fb.progress = 0.1; fb.stage = 'preprocess'; goal_handle.publish_feedback(fb)
        
            # 프로파일 시작 시간
            t0 = time.perf_counter() if self._prof_enabled else 0.0
            t_cv = t_prep = t_infer = t_post = 0.0

            # GTI 입력 생성
            ## 1) ROS Image → OpenCV
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                if self._prof_enabled:
                    t_cv = time.perf_counter()
            except Exception as e:
                self.get_logger().error(f"cv_bridge convert failed: {e}")
                out = EmotionDetection2DArray(); out.header = msg.header
                res.emotion_detections = out
                goal_handle.abort()
                return res
                
            # 기존 DlibFaceDetector.detect 재사용
            # 얼굴 검출
            fd: Optional[DlibFaceDetectorResult] = self._face_detector.detect(frame, 0)
            # 얼굴 없음
            if not fd or not fd.locations:
                out = EmotionDetection2DArray()
                out.header = msg.header
                res.emotion_detections = out
                goal_handle.succeed()
                return res

            # 가장 큰 얼굴 선택 (기존 demo.py와 동일 로직)
            locations = fd.locations
            landmark_list = fd.lm_list
            pick: Optional[rectangle] = None  # 가장 큰 얼굴.   # dlib.rectangle 타입
            largest_face_index = -1           # 위 얼굴 인덱스  # fd.lm_list에 대응하는 랜드마크 꺼내기 위함
            max_width = -1
            for idx, loc in enumerate(locations):
                width = abs(loc.right() - loc.left())
                if width > max_width:
                    max_width = width
                    pick = loc
                    largest_face_index = idx

            if pick is None:
                out = EmotionDetection2DArray(); out.header = msg.header
                res.emotion_detections = out
                goal_handle.succeed()
                return res

            x1, y1 = max(0, int(pick.left())), max(0, int(pick.top()))
            x2, y2 = int(pick.right()), int(pick.bottom())
            x2 = min(x2, frame.shape[1])
            y2 = min(y2, frame.shape[0])
            
            if x2 <= x1 or y2 <= y1:
                out = EmotionDetection2DArray(); out.header = msg.header
                res.emotion_detections = out
                goal_handle.succeed()
                return res

            # crop 후 GTI 입력 생성
            cropped = frame[y1:y2, x1:x2]
            if cropped.size == 0:
                out = EmotionDetection2DArray(); out.header = msg.header
                res.emotion_detections = out
                goal_handle.succeed()
                return res
            
            ## 2) OpenCV → GTI 이미지 버퍼
            gti_img: bytes = convert_opencv2_img_to_gti_img(cropped, self._gti_w, self._gti_h)
            if self._prof_enabled:
                t_prep = time.perf_counter()

            ## 3) 추론
            fb.progress = 0.6; fb.stage = 'inference'
            with self._gti_eval_lock:
                inference_bytes = self._gti_model.GtiImageEvaluate(gti_img, self._gti_w, self._gti_h, 3)
            with self._gti_eval_lock:
                inference_bytes = self._gti_model.GtiImageEvaluate(gti_img, self._gti_w, self._gti_h, 3)
            if self._prof_enabled:
                t_infer = time.perf_counter()
            goal_handle.publish_feedback(fb)

            ## 4) 후처리 → (label, emotion_data)
            fb.progress = 0.9; fb.stage = 'postprocess'
            label, score = self.PostProcessEmotion(inference_bytes)

            # 프로파일 종료 시간
            if self._prof_enabled:
                t_post = time.perf_counter()
                total_ms = (t_post - t0) * 1000.0
                cv_ms    = (t_cv - t0) * 1000.0 if t_cv else 0.0
                prep_ms  = (t_prep - (t_cv if t_cv else t0)) * 1000.0 if t_prep else 0.0
                infer_ms = (t_infer - (t_prep if t_prep else (t_cv if t_cv else t0))) * 1000.0 if t_infer else 0.0
                post_ms  = (t_post - (t_infer if t_infer else (t_prep if t_prep else (t_cv if t_cv else t0)))) * 1000.0
                self.get_logger().info(
                    f"[prof emotion] total={total_ms:.2f}ms cv={cv_ms:.2f}ms prep={prep_ms:.2f}ms infer={infer_ms:.2f}ms post={post_ms:.2f}ms")

            goal_handle.publish_feedback(fb)

            # # 프로파일 종료 시간
            # if self._prof_enabled:
            #     total_ms = (t_post - t0) * 1000.0
            #     cv_ms    = (t_cv - t0) * 1000.0 if t_cv else 0.0
            #     prep_ms  = (t_prep - (t_cv if t_cv else t0)) * 1000.0 if t_prep else 0.0
            #     infer_ms = (t_infer - (t_prep if t_prep else (t_cv if t_cv else t0))) * 1000.0 if t_infer else 0.0
            #     post_ms  = (t_post - (t_infer if t_infer else (t_prep if t_prep else (t_cv if t_cv else t0)))) * 1000.0
            #     self.get_logger().info(
            #         f"[prof emotion] total={total_ms:.2f}ms cv={cv_ms:.2f}ms prep={prep_ms:.2f}ms infer={infer_ms:.2f}ms post={post_ms:.2f}ms")

            # 얼굴 검출 결과 퍼블리시(FaceDetectionArray) - 최대 얼굴 1개만
            try:
                faces_msg = FaceDetectionArray()
                faces_msg.header = msg.header
                f = FaceDetection()
                f.roi = RectangleToRoi(pick)
                pts = []
                if landmark_list and 0 <= largest_face_index < len(landmark_list):
                    for (x, y) in landmark_list[largest_face_index]:
                        p = Point32() 
                        p.x = float(x); p.y = float(y); p.z = 0.0
                        pts.append(p)

                f.landmarks = pts
                f.score = 0.0
                faces_msg.faces = [f]
                self.pub_faces.publish(faces_msg)
            except Exception:
                pass

            out = EmotionDetection2DArray()
            out.header = msg.header
            det = EmotionDetection()
            det.label = label
            det.score = float(score)
            det.roi = RectangleToRoi(pick)
            out.emotion_det = [det]
            res.emotion_detections = out

            fb.progress = 1.0; fb.stage = 'done'; goal_handle.publish_feedback(fb)
            goal_handle.succeed()
        except Exception as e:
            self.get_logger().error(f'emotion execute error: {e}')
            out = EmotionDetection2DArray(); out.header = msg.header
            res.emotion_detections = out
            goal_handle.abort()
        finally:
            self.LeaveInference()

        return res

    def PostProcessEmotion(self, inference_result_bytes: bytes) -> Tuple[str, float]:
        """
        inference_result_bytes(JSON string bytes) -> (label, arousal_prob)
        """
        inference_results = json.loads(inference_result_bytes)
        items = [
            it for it in inference_results.get('result', [])
            if it.get('label') not in ['Arousal']
        ]
        if not items:
            return "Unknown", 0.0

        items_sorted = sorted(items, key=lambda x: x.get('probability', 0), reverse=True)
        label = items_sorted[0]['label']

        # Arousal 추출
        arousal_prob = next(
            (it['probability'] for it in inference_results['result'] if it['label'] == 'Arousal'),
            0.0
        )
        arousal_prob = round(math.tanh(arousal_prob), 3)

        # emotion_data = np.zeros(7, dtype=np.float64)
        # if label in self.EMOTION_LABELS:
        #     idx = self.EMOTION_LABELS.index(label)
        #     emotion_data[idx] = arousal_prob

        return label, arousal_prob

def main():
    rclpy.init()
    node = ManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()