#!venv/bin/python3

from typing import List, Tuple, Sequence, Optional
import json, time, cv2, os, math, errno
import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from gtilib import GtiModelFactory, GtiModel, GtiDeviceException
from gyurinet.gtilib import VisionReader, VisionTypeEnum, VideoCaptureVisionReader, create_vision_reader
from gyurinet.opencv2 import Color, Point, Size
from _dlib_pybind11 import rectangle
from gyurinet.gtilib.util import convert_opencv2_img_to_gti_img
from dlib_face_detector import DlibFaceDetector, DlibFaceDetectorResult

class Demo:
    """
    Classification Demo Class. 2083 추론 내용을 OpenCV HighGUI를 이용하여 보여줌
    """
    # 감정 라벨 (주의: "Happiness " 공백 포함)
    EMOTION_LABELS = ["Neutral", "Anger", "Happiness ", "Surprise", "Disgust", "Sadness", "Fear"]

    @staticmethod
    def load_landmarks_from_config(config_path: str) -> np.ndarray:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, 'r') as file:
            landmarks_list = eval(file.read())  # Use eval to parse the list from the config file.
        return np.array(landmarks_list, dtype=np.float64)

    # Locate the config file using ament_index_python
    CONFIG_FILE_PATH = os.path.join(
        get_package_share_directory('edie_emotion_detection'),
        'config',
        'landmarks.config'
    )

    # Initialize LANDMARKS by loading the config file
    LANDMARKS: np.ndarray = load_landmarks_from_config(CONFIG_FILE_PATH)

    # 화면 사이즈 상수
    window_min_width = 800
    window_min_height = 600

    # 2803에 적재된 GtiModel
    gti_model: GtiModel

    # 윈도우 사이즈는 정수로만 받아오게 함.
    window_width: int
    window_height: int

    SAVE_BTN_TEXT_LIST: Sequence[str] = (
        'Save (frontal face)',
        'Save (left-turn face)',
        'Save (right-turn face)',
        'Save (looking-up face)',
        'Save (looking-down face)',
    )

    device_list: Sequence[Tuple[VisionTypeEnum, str]]
    device_cnt: int

    #얼굴인식
    def __init__(
            self, 
            model_path: str, 
            window_name: str = 'Emotion Detection', 
            window_width: int = 800, 
            window_height: int = 600,
            show_window: bool = True,
            gti_img_width: int = 224,
            gti_img_height: int = 224,
    ) -> None:
        """
        Demo Class 생성시 실행되는 생성자.
        윈도우 초기화 및 2803에 모델 적재
        이 Demo Class 는 Classification 추론 결과를 OpenCV HighGUI로 보여준다.

        Args:
            model_path: 2803에 적재할 모델 파일 경로
            window_name: OpenCV HighGui Window Name(id)
            window_width: OpenCV HighGui Window 너비
            window_height:  OpenCV HighGui Window 눂이
        """
        
        try:
        # GtiModelFactory.create_by_file(model_path): 2803에 적재시킬 모델을 파일로부터 불러 오고 적재 시킨후 2803 모델 객체 만듦.
            self.gti_model = GtiModelFactory.create_by_file(model_path)
        # 2803의 적재 실패 시 에러 메시지가 뜨게 함.
        except GtiDeviceException:
            raise RuntimeError(f"{model_path} Model Not Loaded")

        """ 윈도우 창 초기화 """
        # 윈도우 창 title = Demo
        self.window_name = window_name
        # 윈도우 창 너비 = 800
        self.window_width = window_width
        # 윈도우 창 높이 = 600
        self.window_height = window_height
        # 윈도우 창 표시 여부
        self.show_window = show_window

        """ 2803 이미지 너비, 높이 """
        self._gti_w = gti_img_width
        self._gti_h = gti_img_height

        """ 키 입력 딜레이 """
        self.wait_key_delay = 1  # space로 0/1 토글

        """ OpenCV HighGUI 초기화 """
        if self.show_window:
            self._init_opencv2_window()

        """ 얼굴 인식 init"""
        self.face_detect_results = [None]
        self.face_detect_results: List[Optional[DlibFaceDetectorResult]]
        self.face_detector = DlibFaceDetector(ratio=2)
        self.frame_original_list = [None]
       
    def _init_opencv2_window(self) -> None:
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

    def __del__(self):
        """
        Demo Class 객체 삭제시 실행됨
        Returns:
        """
        # OpenCV HighGUI로 생성된 모든 윈도우를 닫음
        cv2.destroyAllWindows()

    def ConvertPt(self, point: np.ndarray) -> Tuple[int, int]:
        return tuple(np.round(point).astype(np.int32).tolist())
    
    def PerformChipInference(self, gti_img) -> Tuple[Optional[bytes], float, float]:
        """
        추론 칩을 수행하여 bytes, float, float 객체로 값을 전달함.
        Args:
            gti_img: 2803에서 처리할 수 있도록 사전 처리된 이미지 파일
            gti_img_width: GTI2803에서 처리할 이미지 너비
            gti_img_height: GTI2803에서 처리할 이미지 높이

        Returns:
            (추론 결과(json string bytes), 추론 시작 시간, 추론 종료 시간)
        """
        # chip_start = 추론 시작 시간
        chip_start = time.time()
        try:
            inference_result = self.gti_model.GtiImageEvaluate(gti_img, self._gti_w, self._gti_h, 3)
        except TypeError:
            # 예외 처리 예제. 실행되는 경우는 없음
            # GtiImageEvaluate 에서 맨 마지막 인자인 return_type이 잘못된 경우 RuntimeError만 떨어짐
            raise RuntimeError("GtiImageEvaluate Failed")
        # chip_end = 추론 종료 시간
        chip_end = time.time()

        return inference_result, chip_start, chip_end

    def PostProcessEmtion(self, inference_result_bytes: bytes) -> Tuple[str, np.ndarray]:
        """
        inference_result_bytes(JSON string bytes) -> (label, emotion_data[7])
        emotion_data에는 'Arousal'만 하이퍼볼릭 탄젠트 후 해당 label 인덱스에 할당
        """
        inference_results = json.loads(inference_result_bytes)
        items = [
            it for it in inference_results.get('result', [])
            if it.get('label') not in ['Arousal']
        ]
        if not items:
            return "Unknown", np.zeros(7, dtype=np.float64)

        items_sorted = sorted(items, key=lambda x: x.get('probability', 0), reverse=True)
        label = items_sorted[0]['label']

        # Arousal 추출
        arousal_prob = next(
            (it['probability'] for it in inference_results['result'] if it['label'] == 'Arousal'),
            0.0
        )
        arousal_prob = round(math.tanh(arousal_prob), 3)

        emotion_data = np.zeros(7, dtype=np.float64)
        if label in self.EMOTION_LABELS:
            idx = self.EMOTION_LABELS.index(label)
            emotion_data[idx] = arousal_prob

        # if (label == "Happiness ") or (label == "Neutral"):
        if label == "Happiness ":
            label = "Happiness"

        return label, emotion_data

    def ProcessFrame(self, frame: np.ndarray, profile_infer: bool = False, t_init: float = 0.0, t_cv: float = 0.0) -> Optional[Tuple[str, np.ndarray, np.ndarray]]:
        """
        한 프레임 처리:
        - 얼굴 검출 → 가장 큰 얼굴 선택 → crop → GTI 추론 → 라벨/배열 반환
        - 화면에도 결과 표시
        반환: (label: str, emotion_data: np.ndarray(7,), vis_frame: np.ndarray) 또는 None
        """
        # 프로파일 시작 시간
        t_prep = t_infer = t_post = 0.0

        t0 = time.time()
        vis = frame.copy()
        green = Color.GREEN
        font_face = cv2.FONT_HERSHEY_SIMPLEX

        # 얼굴 검출
        fd: Optional[DlibFaceDetectorResult] = self.face_detector.detect(frame, 0)
        if not fd or not fd.locations:
            # 얼굴 없음: 표시만 하고 None 반환
            if self.show_window:
                fps = 1.0 / max(1e-6, (time.time() - t0))
                cv2.putText(vis, f"{fps:.3f} fps", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.GREEN)
                cv2.imshow(self.window_name, vis)
                key = cv2.waitKey(self.wait_key_delay) & 0xFF
                if key == ord(' '):
                    self.wait_key_delay = 0 if self.wait_key_delay != 0 else 1
                elif key == ord('q'):
                    cv2.destroyWindow(self.window_name)
                # 감정 결과는 없음
            return None

        locations = fd.locations
        landmarks_list = fd.lm_list

        # 가장 큰 얼굴 선택
        largest_face_index = None
        max_width = 0
        for idx, loc in enumerate(locations):
            width = abs(loc.right() - loc.left())
            if width > max_width:
                max_width = width
                largest_face_index = idx
        
        # No valid face found, exit early
        if largest_face_index is None:
            return None

        loc = locations[largest_face_index]
        lm = landmarks_list[largest_face_index]

        left, top = max(0, loc.left()), max(0, loc.top())
        right, bottom = loc.right(), loc.bottom()

        # 시각화: 얼굴 박스
        cv2.rectangle(vis, (left, top), (right, bottom), green, 2)

        # 간단 head-pose 축 표시(옵션: 원본 코드 유지)
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.array([0, 0, 1], dtype=np.float64)

        w = float(self.window_width) # 800
        h = float(self.window_height) # 600
        _, rvec, tvec = cv2.solvePnP(
            self.LANDMARKS, lm,
            # np.array([800., 0., 400., 0., 800., 300., 0., 0., 1.]).reshape(3, 3),
            np.array([w, 0., w/2., 0., w, h/2., 0., 0., 1.]).reshape(3, 3),
            np.array([0., 0., 0., 0., 0.]).reshape(-1, 1),
            rvec, tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE
        )
        rot = Rotation.from_rotvec(rvec)
        length = 0.05
        axes3d = np.eye(3, dtype=np.float64) @ Rotation.from_euler('XYZ', [0, np.pi, 0]).as_matrix()
        axes3d = axes3d * length
        axes2d, _ = cv2.projectPoints(
            axes3d, rot.as_rotvec(), tvec,
            # np.array([800., 0., 400., 0., 800., 300., 0., 0., 1.]).reshape(3, 3),
            np.array([w, 0., w/2., 0., w, h/2., 0., 0., 1.]).reshape(3, 3),
            np.array([0., 0., 0., 0., 0.]).reshape(-1, 1)
        )
        axes2d = np.squeeze(axes2d)
        nose_index = lm[30]
        center = self.ConvertPt(nose_index)   # 코
        tip = self.ConvertPt(axes2d[2])
        cv2.arrowedLine(vis, (center[0]*2, center[1]*2), (tip[0]*2, tip[1]*2),
                        (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.5)

        # crop 후 GTI 입력 생성
        cropped = vis[top:bottom, left:right]
        if cropped.size == 0:
            return None
        gti_img = convert_opencv2_img_to_gti_img(cropped, self._gti_w, self._gti_h)
        if profile_infer:
            t_prep = time.perf_counter()

        # 추론
        inference_bytes, chip_start, chip_end = self.PerformChipInference(gti_img)
        if inference_bytes is None:
            return None
        if profile_infer:
            t_infer = time.perf_counter()

        # 후처리 → (label, emotion_data)
        label, emotion_data = self.PostProcessEmtion(inference_bytes)
        if profile_infer:
            t_post = time.perf_counter()
            total_ms = (t_post - t_init) * 1000.0
            cv_ms = (t_cv - t_init) * 1000.0 if t_cv else 0.0
            prep_ms  = (t_prep - (t_cv if t_cv else t_init)) * 1000.0 if t_prep else 0.0
            infer_ms = (t_infer - (t_prep if t_prep else (t_cv if t_cv else t_init))) * 1000.0 if t_infer else 0.0
            post_ms  = (t_post - (t_infer if t_infer else (t_prep if t_prep else (t_cv if t_cv else t_init)))) * 1000.0
            # print(f"[prof emotion] t_init={t_init * 1000:.2f}ms t_cv={t_cv * 1000:.2f}ms t_prep={t_prep * 1000:.2f}ms t_infer={t_infer * 1000:.2f}ms t_post={t_post * 1000:.2f}ms")
            print(f"[prof emotion] total={total_ms:.2f}ms cv={cv_ms:.2f}ms prep={prep_ms:.2f}ms infer={infer_ms:.2f}ms post={post_ms:.2f}ms")
            print("---------------------------------------------------------------------------")

        # 표시
        fps = 1.0 / max(1e-6, (time.time() - t0))
        cv2.putText(vis, f"chip: {1.0 / max(1e-6, (chip_end - chip_start)):.3f} fps",
                    Point(10, 35), cv2.FONT_HERSHEY_COMPLEX, 0.6, Color.RED, 1)
        cv2.putText(vis, f"{fps:.3f} fps", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.GREEN)
        cv2.putText(vis, label, (left, max(10, top - 20)), font_face, 1.0, green, 2, 4)

        if self.show_window:
            scaled = cv2.resize(vis, (480, 360))
            # Pause 오버레이
            if self.wait_key_delay == 0:
                font_face2 = cv2.FONT_HERSHEY_COMPLEX
                font_scale2 = 1
                thickness2 = 1
                fs, base_line = cv2.getTextSize("Pause", font_face2, font_scale2, thickness2)
                fs = Size(*fs)
                lft = scaled.shape[1] - fs.width - 10
                tp = scaled.shape[0] - fs.height
                cv2.putText(scaled, "Pause", Point(lft, tp), font_face2, font_scale2, Color.RED, thickness2)

            cv2.imshow(self.window_name, scaled)

            key = cv2.waitKey(self.wait_key_delay) & 0xFF
            if key == ord(' '):
                self.wait_key_delay = 0 if self.wait_key_delay != 0 else 1
            elif key == ord('q'):
                cv2.destroyWindow(self.window_name)

        return label, emotion_data, cropped