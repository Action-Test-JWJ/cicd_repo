#!venv/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray, String, Bool
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point32
from edie_msgs.msg import EmotionDetection, EmotionDetection2DArray, FaceDetection, FaceDetectionArray
from ml_edie_emotion_detection.emotion_predictor import EmotionPredictor

import sys, os, cv2, errno, time
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ml_edie_emotion_detection.emotional_mesh_detection import EmotionalMeshDetection
    
def draw_fps(frame: np.ndarray, fps: float) -> None:
    fps_text = 'FPS = {:.1f}'.format(fps)
    cv2.putText(frame, fps_text, (20, 24), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)


def draw_bounding_box(frame: np.ndarray, bounding_box, label: str) -> None:
    cv2.rectangle(frame, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (255, 255, 0), 2)
    (w, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(frame, (bounding_box[0], bounding_box[1] - 20), (bounding_box[0] + w, bounding_box[1]), (255, 255, 0), -1)
    cv2.putText(frame, label, (bounding_box[0], bounding_box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

class EmotionDetectionNode(Node):
    def __init__(self):
        super().__init__('edie_emotion_detection_node')

        # Declare parameters with defaults
        self.declare_parameter('algorithm', 'KNN')
        self.declare_parameter('max_num_faces', 1)
        self.declare_parameter('profile_infer', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('draw_mesh', False)
        # Neutral 판정 임계치(%) 파라미터
        self.declare_parameter('neutral_min_conf', 60.0)   # [%]
        self.declare_parameter('neutral_margin_th', 15.0)  # [%]

        self._algorithm = str(self.get_parameter('algorithm').value)
        self._max_num_faces = int(self.get_parameter('max_num_faces').value)
        self._prof_enabled = bool(self.get_parameter('profile_infer').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._draw_mesh = bool(self.get_parameter('draw_mesh').value)
        # Neutral 파라미터 로드(%) → 내부에서 0~1 스케일로 전달
        _neutral_min_conf = float(self.get_parameter('neutral_min_conf').value)
        _neutral_margin_th = float(self.get_parameter('neutral_margin_th').value)
        self.get_logger().info(f'[_init__] algorithm: {self._algorithm}, neutral_min_conf: {_neutral_min_conf}, neutral_margin_th: {_neutral_margin_th}, max_num_faces: {self._max_num_faces}, profile_infer: {self._prof_enabled}')

        self.bridge = CvBridge()
        self.emotion_label_buffer = deque()
        self.label = None
        self.last_published_label = None
        self.emotion_observation_time = 1.0 #0.5  # EMO_WIN [sec]
        self.max_valid_time = 0.4            # TTL [sec]

        # Emotion predictor
        self.emotion_predictor = EmotionPredictor(
            model_algorithm=self._algorithm,
            max_num_faces=self._max_num_faces,
            neutral_min_conf=max(0.0, min(1.0, _neutral_min_conf / 100.0)),
            neutral_margin_th=max(0.0, min(1.0, _neutral_margin_th / 100.0)),
        )

        # Optional: mesh visualizer (별도 처리 경량 오버레이)
        self._mesh_vis = None
        if self._draw_mesh:
            try:
                self._mesh_vis = EmotionalMeshDetection(max_num_faces=self._max_num_faces, refine=True)
            except Exception:
                self._mesh_vis = None

        # ===== QoS 설정 =====
        # (2) result_qos: 1회성 이미지 전달, 늦게 구독해도 전달 / 오래된 건 폐기
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # STT가 늦게 붙어도 최근 1장 전달
            history=HistoryPolicy.KEEP_LAST,
            lifespan=Duration(seconds=5)  # 너무 오래된 이미지가 재전송되지 않도록
        )
        # 저지연 상태 퍼블리셔용 QoS (backpressure 최소화)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher
        self.pub_emption_state = self.create_publisher(String, '/edie8/vision/emotion_state', state_qos)
        # FaceDetectionArray (edie.cpp 파이프라인 연계용)
        self.pub_faces = self.create_publisher(FaceDetectionArray, '/edie8/vision/face_detections', 10)

        # Subscriberce
        self.sub_img = self.create_subscription(Image, '/edie8/vision/image_raw', self.ImageCallback, 10)

        self.last_image_header = None   # 원본 이미지 헤더 보관
        self.frame = None

        # FPS calculation
        self.counter: int = 0
        self.fps_avg_frame_count: int = 10
        self.fps: float = 0.0
        self.start_time: float = time.time()

    def ImageCallback(self, msg: Image):   
        self.last_image_header = msg.header  # stamp, frame_id 같이 보관
        # 프로파일 시작 시간
        t_init = time.perf_counter() if self._prof_enabled else 0.0
        t_cv = 0.0
        now = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self._prof_enabled:
                t_cv = time.perf_counter()
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        # Predict emotions and publish bounding boxes <- emotion_predictor.py 내 detect 함수에서 가장 큰 얼굴 1개 선택 로직 확인
        emo_det_array_msg = EmotionDetection2DArray()
        emo_det_array_msg.header = msg.header
        self.frame = frame
        
        emotions = self.emotion_predictor.predict(self.frame)
        frame_h, frame_w = self.frame.shape[:2]
        for emotion in emotions:
            # 안전 클램핑
            try:
                x1 = int(emotion.bounding_box[0])
                y1 = int(emotion.bounding_box[1])
                x2 = int(emotion.bounding_box[2])
                y2 = int(emotion.bounding_box[3])
            except Exception:
                continue

            x1 = max(0, min(x1, max(0, frame_w - 1)))
            y1 = max(0, min(y1, max(0, frame_h - 1)))
            x2 = max(0, min(x2, max(0, frame_w - 1)))
            y2 = max(0, min(y2, max(0, frame_h - 1)))

            # 정규화: 좌상단/우하단 보장
            if x2 < x1:
                x1, x2 = x2, x1
            if y2 < y1:
                y1, y2 = y2, y1

            # 유효성 검사(영역이 0이면 스킵 가능; 메시지는 0도 허용)
            safe_bbox = (x1, y1, x2, y2)
            draw_bounding_box(self.frame, safe_bbox, getattr(emotion, 'label', getattr(emotion, 'class_name', 'Unknown')))

            emo_det_msg = EmotionDetection()
            emo_det_msg.label = getattr(emotion, 'class_name', getattr(emotion, 'label', 'Unknown'))
            emo_det_msg.score = float(getattr(emotion, 'probability', 0.0))

            roi = RegionOfInterest()
            roi.x_offset = x1
            roi.y_offset = y1
            roi.width = max(0, x2 - x1)
            roi.height = max(0, y2 - y1)
            roi.do_rectify = False
            emo_det_msg.roi = roi

            emo_det_array_msg.emotion_det.append(emo_det_msg)

        # FaceDetectionArray 퍼블리시: predict()가 이미 가장 큰 얼굴 1개만 반환 → 첫 요소 사용
        try:
            if emo_det_array_msg.emotion_det:
                best = emo_det_array_msg.emotion_det[0]
                
                faces_msg = FaceDetectionArray()
                faces_msg.header = msg.header
                f = FaceDetection()
                f.roi = best.roi
                # 27개 랜드마크(rcoordinates)를 Point32 배열로 변환하여 채움
                try:
                    meshes = self.emotion_predictor.emotional_mesh_detection.get_emotional_meshes()
                    lm_pts = []
                    if meshes:
                        # max_num_faces=1 기본이므로 0번째 사용
                        mesh = meshes[0]
                        for (px, py) in getattr(mesh, 'rcoordinates', []) or []:
                            p = Point32() 
                            p.x = float(px) 
                            p.y = float(py) 
                            p.z = 0.0
                            lm_pts.append(p)
                            
                    f.landmarks = lm_pts
                except Exception:
                    f.landmarks = []
                # EmotionDetection2DArray의 score(%)를 그대로 사용
                f.score = float(best.score)
                faces_msg.faces = [f]
                self.pub_faces.publish(faces_msg)
        except Exception:
            pass

        # ===== 시간 기반 스무딩(다수결) =====
        # 1) 윈도우에서 오래된 항목 제거
        try:
            cutoff = now - self.emotion_observation_time
            while self.emotion_label_buffer and self.emotion_label_buffer[0][1] < cutoff:
                self.emotion_label_buffer.popleft()
        except Exception:
            pass

        # 2) 이번 프레임 라벨을 버퍼에 추가(있을 때만)
        if emo_det_array_msg.emotion_det:
            try:
                curr_label = str(emo_det_array_msg.emotion_det[0].label or 'Neutral')
            except Exception:
                curr_label = 'Neutral'
            self.emotion_label_buffer.append((curr_label, now))
        else:
            self.PubEmotionState('Unknown')
            
        # 3) 다수결 집계 후 변경 시에만 발행
        if self.emotion_label_buffer:
            try:
                labels = [l for (l, t) in self.emotion_label_buffer]
                most_common = max(set(labels), key=labels.count)
                
                if most_common != self.last_published_label:
                    self.PubEmotionState(most_common)
                    self.last_published_label = most_common
            except Exception:
                pass

        # Update FPS
        self.UpdateFPS()
        draw_fps(self.frame, self.fps)

        if self._show_window:
            # 선택적 Emotional Mesh 오버레이
            if self._draw_mesh and self._mesh_vis is not None:
                try:
                    self._mesh_vis.process(self.frame)
                    for m in self._mesh_vis.get_emotional_meshes():
                        m.draw(self.frame)
                except Exception:
                    pass
            cv2.imshow('ml_edie_emotion_detection', self.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                try:
                    cv2.destroyWindow('ml_edie_emotion_detection')
                except Exception:
                    try:
                        cv2.destroyAllWindows()
                    except Exception:
                        pass
                self._show_window = False
    
    def UpdateFPS(self) -> None:
        self.counter += 1
        if self.counter % self.fps_avg_frame_count == 0:
            end_time = time.time()
            self.fps = self.fps_avg_frame_count / (end_time - self.start_time)
            self.start_time = time.time()

    def PubEmotionState(self, emotion_state):
        msg = String()
        msg.data = emotion_state
        self.pub_emption_state.publish(msg)

def main(argv=None):
    if argv is None:     # new   
        argv = sys.argv  # argv가 None이면 sys.argv로 설정
    
    rclpy.init(args=argv)
    node = EmotionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 