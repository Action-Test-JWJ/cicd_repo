#!/usr/bin/env python3
from typing import List, Tuple, Optional
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from collections import deque
import time
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, String, Bool
from sensor_msgs.msg import RegionOfInterest, Image
from edie_msgs.msg import EmotionDetection2DArray, FaceDetectionArray
from edie_vora_common_python.base_infer_client import BaseInferClient
from ament_index_python.packages import get_package_share_directory

class EmotionDetectionClient(BaseInferClient):
    # 감정 라벨 (주의: "Happiness " 공백 포함)
    EMOTION_LABELS = ["Neutral", "Anger", "Happiness ", "Surprise", "Disgust", "Sadness", "Fear"]

    @staticmethod
    def LoadFaceLandmarks(config_path: str) -> np.ndarray:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, 'r') as file:
            landmarks_list = eval(file.read())  # Use eval to parse the list from the config file.
        return np.array(landmarks_list, dtype=np.float64)

    # Locate the config file using ament_index_python
    CONFIG_FILE_PATH = os.path.join(
        get_package_share_directory('edie_emotion_detection_python'),
        'config',
        'landmarks.config'
    )

    # Initialize LANDMARKS by loading the config file
    LANDMARKS: np.ndarray = LoadFaceLandmarks(CONFIG_FILE_PATH)

    def __init__(self):
        super().__init__('emotion_detection_client', mode='emotion')       
        # ===== QoS 설정 =====
        # (1) voice_heard: 이벤트 유실 방지용
        voice_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        # (2) result_qos: 1회성 이미지 전달, 늦게 구독해도 전달 / 오래된 건 폐기
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # STT가 늦게 붙어도 최근 1장 전달
            history=HistoryPolicy.KEEP_LAST,
            lifespan=Duration(seconds=5)
        )
        # 저지연 상태 퍼블리셔용 QoS (backpressure 최소화)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher
        # self.pub_emotion_array = self.create_publisher(Float64MultiArray, '/edie8/vision/normalized_emotion_depth_array', result_qos)     
        self.pub_emption_state = self.create_publisher(String, '/edie8/vision/emotion_state', state_qos)
        self.pub_emotion_result_image  = self.create_publisher(Image, '/edie8/vision/face_img', result_qos)

        # Subscriber
        self.sub_voice_heard = self.create_subscription(Bool, '/edie8/llm/voice_heard', self.VoiceCallback, voice_qos)
        self.sub_face_detections = self.create_subscription(FaceDetectionArray, 'vora/face_detections', self.FaceDetectionCallback, 10)
        
        # Parameters / State
        self.emotion_data = np.array([0.0]*7, dtype=np.float64) 
        self.oneshot_flag = False
        self.last_image_header = None
        self.last_published_label = None
        self.emotion_label_buffer = deque()
        self.emotion_observation_time = 0.5  # EMO_WIN [sec]
        self.max_valid_time = 0.4            # TTL [sec]

        self.bridge = CvBridge()
        """ 윈도우 창 초기화 """
        self.window_name = 'EmotionDetection'
        self.window_width = 800
        self.window_height = 600
        # 창 표시 여부 파라미터 선언
        self.declare_parameter('show_window', False)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.get_logger().info(f"[EmotionDetectionClient.__init__] show_window: {self.show_window}")
        """ 키 입력 딜레이 """
        self.wait_key_delay = 1  # space로 0/1 토글

        """ OpenCV HighGUI 초기화 """
        if self.show_window:
            self.InitOpenCVWindow()

        self.get_logger().info('EmotionDetectionClient ready')

    def InitOpenCVWindow(self) -> None:
        """
        OpenCV HighGUI 초기화
        Returns:
        """
        # cv2.WINDOW_AUTOSIZE: 영상 크기에 맞도록 자동으로 윈도우 크기 조정 (사용자가 크기 조정 불가)
        # cv2.WINDOW_GUI_NORMAL: 상태나 도구바 없는 윈도우로 출력
        # cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE + cv2.WINDOW_GUI_NORMAL)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        # 지정한 _window_name을 갖는 윈도우 크기를 window_width X window_height 크기로 변경
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        # 지정한 _window_name을 갖는 윈도우 위치를 (400, 100) 위치로 이동
        cv2.moveWindow(self.window_name, 400, 100)

    def ConvertPt(self, point: np.ndarray) -> Tuple[int, int]:
        return tuple(np.round(point).astype(np.int32).tolist())

    # voice 이벤트(원샷 발행 트리거)
    def VoiceCallback(self, msg: Bool):
        if msg.data:
            if not self.oneshot_flag:
                self.oneshot_flag = True
                self.get_logger().info("voice_heard=True 수신 → face_img 원샷 발행 대기 상태로 전환")

    # FaceDetectionArray 수신 → 시각화(박스, 랜드마크, 헤드포즈)
    def FaceDetectionCallback(self, msg: FaceDetectionArray):
        try:
            last_img: Image = getattr(self, '_last_image', None)
            if last_img is None or not msg.faces:
                return
            frame = self.bridge.imgmsg_to_cv2(last_img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        green = (0, 255, 0)
        font_face = cv2.FONT_HERSHEY_SIMPLEX
        
        # 시각화: 얼굴 박스
        face = msg.faces[0]
        x, y, w, h = face.roi.x_offset, face.roi.y_offset, face.roi.width, face.roi.height
        try:
            cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), green, 2)
        except Exception:
            pass

        # 간단 head-pose 축 표시(옵션: 원본 코드 유지)
        # 1) landmarks(Point32[]) → Nx2 float64
        pts2d = np.array([[float(p.x), float(p.y)] for p in face.landmarks], dtype=np.float64)
        if pts2d.shape[0] < 6:
            return

        # 2) object points (Nx3) 길이 맞추기
        obj3d = self.LANDMARKS
        if obj3d.shape[0] != pts2d.shape[0]:
            # self.get_logger().warn(f'landmarks mismatch: {obj3d.shape[0]} != {pts2d.shape[0]}')
            # self.get_logger().warn(f'nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn')
            n = min(obj3d.shape[0], pts2d.shape[0])
            obj3d = obj3d[:n]
            pts2d = pts2d[:n]
        # 일단 else 구문만 출력 확인.
        # else:
        #     self.get_logger().info(f'landmarks matched: {obj3d.shape[0]} == {pts2d.shape[0]}')
        #     self.get_logger().info(f'!')

        # 3) 카메라 파라미터 (데모와 동일: 윈도우 기준)
        w = float(self.window_width)
        h = float(self.window_height)
        K = np.array([w, 0., w/2., 0., w, h/2., 0., 0., 1.], dtype=np.float64).reshape(3, 3)
        dist = np.zeros((5, 1), dtype=np.float64)

        # 4) 초기 추정치
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.array([0., 0., 1.], dtype=np.float64)

        # 5) PnP
        ok, rvec, tvec = cv2.solvePnP(
            obj3d, pts2d, K, dist, rvec, tvec,
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return

        # 6) 축 투영 및 그리기 (데모와 동일)
        rot = Rotation.from_rotvec(rvec)
        length = 0.05
        axes3d = np.eye(3, dtype=np.float64) @ Rotation.from_euler('XYZ', [0, np.pi, 0]).as_matrix()
        axes3d = axes3d * length
        axes2d, _ = cv2.projectPoints(axes3d, rot.as_rotvec(), tvec, K, dist)
        axes2d = np.squeeze(axes2d)

        # 코(랜드마크 30) 기준으로 Z축 그리기
        nose_index = pts2d[30]  # (x,y)
        center = self.ConvertPt(nose_index)
        tip = self.ConvertPt(axes2d[2])
        cv2.arrowedLine(frame, (center[0]*2, center[1]*2), (tip[0]*2, tip[1]*2),
                        (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.5)

        if self.show_window:
            scaled = cv2.resize(frame, (480, 360))
            # Pause 오버레이
            if self.wait_key_delay == 0:
                font_face2 = cv2.FONT_HERSHEY_COMPLEX
                font_scale2 = 1
                thickness2 = 1
                fs, base_line = cv2.getTextSize("Pause", font_face2, font_scale2, thickness2)
                text_w, text_h = fs  # (width, height)
                lft = scaled.shape[1] - text_w - 10
                tp = scaled.shape[0] - text_h
                cv2.putText(scaled, "Pause", (lft, tp), font_face2, font_scale2, (0, 0, 255), thickness2)

            cv2.imshow(self.window_name, scaled)

            key = cv2.waitKey(self.wait_key_delay) & 0xFF
            if key == ord(' '):
                self.wait_key_delay = 0 if self.wait_key_delay != 0 else 1
            elif key == ord('q'):
                cv2.destroyWindow(self.window_name)

        if self.oneshot_flag:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = last_img.header
                self.pub_emotion_result_image.publish(img_msg)
            except Exception:
                pass
            self.oneshot_flag = False

    # Base hook (emotion)
    def HandleEmotionInferenceResult(self, dets: EmotionDetection2DArray):
        if not dets.emotion_det:
            return
        # else:
        #     self.get_logger().info(f'len: {len(dets.emotion_det)}')
        now = time.time()

        label = dets.emotion_det[0].label
        score = dets.emotion_det[0].score
        emotion_data = np.zeros(7, dtype=np.float64)
        if label in self.EMOTION_LABELS:
            idx = self.EMOTION_LABELS.index(label)
            emotion_data[idx] = score
        
        # if (label == "Happiness ") or (label == "Neutral"):
        if label == "Happiness ":
            label = "Happiness"

        # ------------------------ 여기에 스무딩 처리 추가 -------------------
        #  버퍼에 (label, t) 저장
        self.emotion_label_buffer.append((label, now))
        # self.get_logger().info(f'[HandleEmotionInferenceResult] len(emotion_label_buffer): {len(self.emotion_label_buffer)}')
        # self.get_logger().info(f'[HandleEmotionInferenceResult] -----------------------------------------------------------')

        #  버퍼에서 오래된 항목 제거
        cut_off_time = now - self.emotion_observation_time
        while self.emotion_label_buffer and self.emotion_label_buffer[0][1] < cut_off_time:
            self.emotion_label_buffer.popleft()

        # 얉은 TTL: 최근 표본이 너무 오래됐으면 발행 중단 (스트림 끊김 시 잔상 방지)
        if not self.emotion_label_buffer or (now - self.emotion_label_buffer[-1][1]) > self.max_valid_time:
            return

        # 다수결
        labels = [l for l, t in self.emotion_label_buffer]
        most_common = max(set(labels), key=labels.count)

        # 중복시 발행 중단
        if most_common == self.last_published_label:
            return
        self.last_published_label = most_common

        # 감정 라벨 발행
        s = String()
        s.data = most_common
        self.pub_emption_state.publish(s)
        
        # 감정 데이터 배열 발행
        # msg = Float64MultiArray() 
        # msg.data = emotion_data.tolist()
        # self.pub_emotion_array.publish(msg)


def main():
    rclpy.init()
    node = EmotionDetectionClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()