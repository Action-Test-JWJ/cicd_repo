#!venv/bin/python3

import sys
from typing import List, Tuple, Sequence, Optional
import errno
import json
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import math

from gtilib import GtiModelFactory, GtiModel, GtiDeviceException
from gyurinet.gtilib import VisionReader, VideoCaptureVisionReader, VisionTypeEnum, create_vision_reader
from gyurinet.opencv2 import Color, Point
from _dlib_pybind11 import rectangles, rectangle

from dlib_face_detector import DlibFaceDetector, DlibFaceDetectorResult
import os
os.environ['GTISDKPATH'] = '/media/vimlab/A054896254893C52/fire/2803SDK_v5.0.4.0_AI_v1.2.0'

class Demo:
    """
    Classification Demo Class. 2083 추론 내용을 OpenCV HighGUI를 이용하여 보여줌
    """
    LANDMARKS: np.ndarray = np.array([
        [-0.07141807, -0.02827123, 0.08114384],
        [-0.07067417, -0.00961522, 0.08035654],
        [-0.06844646, 0.00895837, 0.08046731],
        [-0.06474301, 0.02708319, 0.08045689],
        [-0.05778475, 0.04384917, 0.07802191],
        [-0.04673809, 0.05812865, 0.07192291],
        [-0.03293922, 0.06962711, 0.06106274],
        [-0.01744018, 0.07850638, 0.04752971],
        [0., 0.08105961, 0.0425195],
        [0.01744018, 0.07850638, 0.04752971],
        [0.03293922, 0.06962711, 0.06106274],
        [0.04673809, 0.05812865, 0.07192291],
        [0.05778475, 0.04384917, 0.07802191],
        [0.06474301, 0.02708319, 0.08045689],
        [0.06844646, 0.00895837, 0.08046731],
        [0.07067417, -0.00961522, 0.08035654],
        [0.07141807, -0.02827123, 0.08114384],
        [-0.05977758, -0.0447858, 0.04562813],
        [-0.05055506, -0.05334294, 0.03834846],
        [-0.0375633, -0.05609241, 0.03158344],
        [-0.02423648, -0.05463779, 0.02510117],
        [-0.01168798, -0.04986641, 0.02050337],
        [0.01168798, -0.04986641, 0.02050337],
        [0.02423648, -0.05463779, 0.02510117],
        [0.0375633, -0.05609241, 0.03158344],
        [0.05055506, -0.05334294, 0.03834846],
        [0.05977758, -0.0447858, 0.04562813],
        [0., -0.03515768, 0.02038099],
        [0., -0.02350421, 0.01366667],
        [0., -0.01196914, 0.00658284],
        [0., 0., 0.],
        [-0.01479319, 0.00949072, 0.01708772],
        [-0.00762319, 0.01179908, 0.01419133],
        [0., 0.01381676, 0.01205559],
        [0.00762319, 0.01179908, 0.01419133],
        [0.01479319, 0.00949072, 0.01708772],
        [-0.045, -0.032415, 0.03976718],
        [-0.0370546, -0.0371723, 0.03579593],
        [-0.0275166, -0.03714814, 0.03425518],
        [-0.01919724, -0.03101962, 0.03359268],
        [-0.02813814, -0.0294397, 0.03345652],
        [-0.03763013, -0.02948442, 0.03497732],
        [0.01919724, -0.03101962, 0.03359268],
        [0.0275166, -0.03714814, 0.03425518],
        [0.0370546, -0.0371723, 0.03579593],
        [0.045, -0.032415, 0.03976718],
        [0.03763013, -0.02948442, 0.03497732],
        [0.02813814, -0.0294397, 0.03345652],
        [-0.02847002, 0.03331642, 0.03667993],
        [-0.01796181, 0.02843251, 0.02335485],
        [-0.00742947, 0.0258057, 0.01630812],
        [0., 0.0275555, 0.01538404],
        [0.00742947, 0.0258057, 0.01630812],
        [0.01796181, 0.02843251, 0.02335485],
        [0.02847002, 0.03331642, 0.03667993],
        [0.0183606, 0.0423393, 0.02523355],
        [0.00808323, 0.04614537, 0.01820142],
        [0., 0.04688623, 0.01716318],
        [-0.00808323, 0.04614537, 0.01820142],
        [-0.0183606, 0.0423393, 0.02523355],
        [-0.02409981, 0.03367606, 0.03421466],
        [-0.00756874, 0.03192644, 0.01851247],
        [0., 0.03263345, 0.01732347],
        [0.00756874, 0.03192644, 0.01851247],
        [0.02409981, 0.03367606, 0.03421466],
        [0.00771924, 0.03711846, 0.01940396],
        [0., 0.03791103, 0.0180805],
        [-0.00771924, 0.03711846, 0.01940396],
    ], dtype=np.float64)
    # 화면 사이즈 상수
    WINDOW_MIN_WIDTH = 800
    WINDOW_MIN_HEIGHT = 600

    # 2803에 적재된 GtiModel
    _gti_model: GtiModel

    # 윈도우 사이즈는 정수로만 받아오게 함.
    _window_width: int
    _window_height: int

    SAVE_BTN_TEXT_LIST: Sequence[str] = (
        'Save (frontal face)',
        'Save (left-turn face)',
        'Save (right-turn face)',
        'Save (looking-up face)',
        'Save (looking-down face)',
    )

    _device_list: Sequence[Tuple[VisionTypeEnum, str]]
    _device_cnt: int

    #얼굴인식

    def __init__(
            self, model_path: str, window_name: str = 'Demo', window_width: int = 800, window_height: int = 600
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
            #GtiModelFactory.create_by_file(model_path) = 2803에 적재시킬 모델을 파일로 부터 불러 오고 적재 시킨후 2803 모델 객체를
            #                                              만들어준다.
            self._gti_model = GtiModelFactory.create_by_file(model_path)
            # 윈도우 창 title = Demo
            self._window_name = window_name
            # 윈도우 창 너비 = 800
            self._window_width = window_width
            # 윈도우 창 높이 = 600
            self._window_height = window_height
            # OpenCV HighGUI 초기화
            self._init_opencv2_window()
        # 2803의 적재 실패 시 에러 메시지가 뜨게 함.
        except GtiDeviceException:
            raise RuntimeError(f"{model_path} Model Not Loaded")

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
        cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE + cv2.WINDOW_GUI_NORMAL)
        # 지정한 _window_name을 갖는 윈도우 크기를 _window_width X _window_height 크기로 변경
        cv2.resizeWindow(self._window_name, self._window_width, self._window_height)
        # 지정한 _window_name을 갖는 윈도우 위치를 (400, 100) 위치로 이동
        cv2.moveWindow(self._window_name, 400, 100)

    def __del__(self):
        """
        Demo Class 객체 삭제시 실행됨
        Returns:
        """

        # OpenCV HighGUI로 생성된 모든 윈도우를 닫음
        cv2.destroyAllWindows()

    def Run(
            self, vision_type: VisionTypeEnum, path_pattern: str, gti_img_width: int = 224, gti_img_height: int = 224
    ):
        """
        Classification 추론을 실행하고 결과를 OpenCV HighGUI를 이용하여 보여줌
        Demo Class 실행 시 실행됨.
        Args:
            vision_type: 추론 대상 영상 형식(캠, 비디오, 이미지, RAW 이미지, 이미지 디렉토리)
            path_pattern: 영상 파일 경로 패턴(이미지인 경우 와일드 카드 사용 가능 나머지는 단일만 지정 가능)

        Returns:

        """

        # vision_reader 영상을 읽어 오는 객체
        vision_reader = create_vision_reader(vision_type, path_pattern)
        if isinstance(vision_reader, VideoCaptureVisionReader):
            # reader가 Video 타입(비디오파일, 캠)이라면 원본 소스에 맞춰 윈도우 사이즈 조정
            vision_reader.resize_from_video_capture_source(self._window_name, self.WINDOW_MIN_WIDTH,
                                                           self.WINDOW_MIN_HEIGHT)

        # 종료될 때까지 무한 반복
        wait_key_delay = 1
        while True:
            # 영상 Reader 객체가 가지고 있는 이미지 Index 만큼 반복
            for idx in vision_reader:
                
                self.PerformInferenceAndDraw(vision_reader, idx, gti_img_width, gti_img_height, self.face_detector)

                # 키 입력 대기 시간 = 10ms. 이 코드가 없으면 OpenCV HighGUI의 이미지가 갱신 되지 않음.
                # key_code = 대기중 입력받은 ASCII 키 값
                key_code = cv2.waitKey(wait_key_delay)
                if key_code == -1:
                    # 입력 대기 시간 동안 아무 키도 눌리지 않았다면 다음 코드를 실행
                    continue

                # chr = ascii 키보드를 문자로 변경
                if 'q' == chr(key_code):

                    # 입력된 값이 q라면 종료
                    return

                if ' ' == chr(key_code):  # 입력된 값이 spacebar라면 dealy 값을 변경
                    if wait_key_delay == 0:
                        wait_key_delay = 10
                    else:
                        wait_key_delay = 0  # dealy 가 0 이면 일시정지

    def PerformInferenceAndDraw(
            self, vision_reader: VisionReader, idx: int, gti_img_width: int, gti_img_height: int,  face_detector: DlibFaceDetector,
    ):
        """
        영상의 Classification 추론을 수행하고 결과를 OpenCV HighGUI에 출력

        Args:
            gti_img_width:
            vision_reader: 영상 Reader 객체
            idx: 읽을 대상의 영상 index를 정수로 받아옴.

        Returns:

        """
        left =0
        person_info_y =0
        font_face = 0
        # gti_img = 영상 Reader 객체에서 해당 Index의 영상을 GTI용 입력 데이터 가져온다.
        self.frame_original_list = vision_reader.get_opencv_img(idx)
        frame_recognition = self.frame_original_list.copy()
        chip_start, chip_end = 5, 1

        self.face_detect_results[0] = face_detector.detect(self.frame_original_list, 0)
        face_detect_result = self.face_detect_results[0]
        inference_result_label = 'label not found'
        if face_detect_result is None:
            pass
        else:
            locations = face_detect_result.locations
            landmarks = face_detect_result.lm_list
            green = Color.GREEN
            font_face = cv2.FONT_HERSHEY_SIMPLEX

            # for different cameras
            loc: rectangle
            
            # locations[0]

            for loc, lm in zip(locations,landmarks):
                right, bottom = [loc.right(), loc.bottom()]
                left = max(0, loc.left())
                top = max(0, loc.top())

                cv2.rectangle(frame_recognition, (left, top), (right, bottom), green, 2)
                
                rvec = np.zeros(3, dtype=np.float64)
                tvec = np.array([0, 0, 1], dtype=np.float64)
                _, rvec, tvec = cv2.solvePnP(self.LANDMARKS, lm, np.array([800., 0., 400.,
                                             0., 800., 300.,0., 0., 1.]).reshape(3, 3),
                                             np.array([0., 0., 0., 0., 0.]).reshape(-1, 1), 
                                             rvec, tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
                rot = Rotation.from_rotvec(rvec)
                right, bottom = [loc.right(), loc.bottom()]
                left = max(0, loc.left())
                top = max(0, loc.top())
                length = 0.05
                axes3d = np.eye(3, dtype=np.float64) @ Rotation.from_euler('XYZ', [0, np.pi, 0]).as_matrix()
                axes3d = axes3d * length
                if rvec is None:
                    rvec = np.zeros(3, dtype=np.float)
                if tvec is None:
                    tvec = np.zeros(3, dtype=np.float)
                axes2d, _ = cv2.projectPoints(axes3d, rot.as_rotvec(), tvec,
                                                np.array([800., 0., 400., 0., 800., 300.,0., 0., 1.]).reshape(3, 3),
                                                np.array([0., 0., 0., 0., 0.]).reshape(-1, 1))
                #axes2d = self._camera.project_points(axes3d,rot.as_rotvec(),tvec)
                axes2d = np.squeeze(axes2d)
                center = lm[30] # nose index
                center = self.ConvertPt(center)
                pt = self.ConvertPt(axes2d[2])
                cv2.arrowedLine(frame_recognition, (center[0]*2,center[1]*2), (pt[0]*2,pt[1]*2), (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.5)
                
                

                person_info_y = np.max((10, top - 20))
            # inference_result_bytes = 영상의 추론 결과 json string bytes

            cropped_region = frame_recognition[top:bottom, left:right]
            
            gti_img = vision_reader.get_gti_img_intermediate(cropped_region)
            
            inference_result_bytes, chip_start, chip_end = self.PerformChipInference(
                gti_img, gti_img_width, gti_img_height
            )

            inference_results = json.loads(inference_result_bytes)
            #print(inference_results)
            # inference_result_label = 첫번째 추론 결과값(라벨)
            arousal_prob = next(item['probability'] for item in inference_results['result'] if item['label'] == 'Arousal')
            arousal_prob = round(math.tanh(arousal_prob), 3)
            # sorted_items = sorted(
            #     (item for item in inference_results['result'] if item['label'] != 'Arousal'),
            #     key=lambda x: x['probability'],
            #     reverse=True
            # )
            
            sorted_items = sorted(
                (item for item in inference_results.get('result', []) 
                if item.get('label') not in ['Arousal']),
                key=lambda x: x.get('probability', 0),
                reverse=True
            )
            
            #inference_result_label = [item for item in inference_results['result'] if item['label'].strip() == 'Happiness']
            

            inference_result_label = sorted_items[0]['label']
            #if 'label' in inference_results["result"][0]:
            #    inference_result_label = inference_results["result"][0]["label"]

            cv2.putText(frame_recognition, inference_result_label+str('   ')+str(arousal_prob),(left, person_info_y), font_face, 1.0, green, 2, 4)

        scaled_raw_img = cv2.resize(frame_recognition, (800, 600))

        # duration = 추론하는데 소비된 시간
        # frame_info = 시간을 fps로 변경 Frame Per Seconds. 초당 몇장의 추론을 진행하였는지.
        # 윈도우 화면에 텍스트를 출력
        # 이미지, fps 값, 텍스트 좌표, 폰트 종류, 폰트 크기, 폰트 색상, 폰트 두께
        duration = chip_end - chip_start
        
        frame_info = "chip: {0:.3f} fps".format(1 / duration)
        if face_detect_result:
            cv2.putText(scaled_raw_img, frame_info, Point(10, 35), cv2.FONT_HERSHEY_COMPLEX, 0.6, Color.RED, 1)

        # 이미지를 윈도우 화면에 출력
        # 윈도우 title = _window_name
        # 이미지 파일 = scaled_raw_img
        cv2.imshow(self._window_name, scaled_raw_img)
        
    def ConvertPt(self, point: np.ndarray) -> Tuple[int, int]:
        return tuple(np.round(point).astype(np.int32).tolist())

    # Done
    def PerformChipInference(
            self, gti_img, gti_img_width: int = 224, gti_img_height: int = 224
    ) -> Tuple[Optional[bytes], float, float]:
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
            inference_result = self._gti_model.GtiImageEvaluate(gti_img, gti_img_width, gti_img_height, 3)
                
        except TypeError as e:
            # 예외 처리 예제. 실행되는 경우는 없음
            # GtiImageEvaluate 에서 맨 마지막 인자인 return_type이 잘못된 경우 RuntimeError만 떨어짐
            raise RuntimeError("GtiImageEvaluate Failed")

        # chip_start 추론 종료 시간
        chip_end = time.time()

        return inference_result, chip_start, chip_end


def print_usage(argv: List[str]):
    """
    demo.py 예제 코드 실행에 필요한 인자 안내
    Args:
        argv: 예제 코드 실행시 제공된 인자
              argv[0] demo.py
              argv[1] 영상 타입
              argv[2] 모델 파일 경로
              argv[3] 영상 파일 경로 패턴
    Returns:

    """

    # model_path = model 경로
    model_path = '../../assets/models/2803/2803_vgg16_FER.model'
    # s = 실행 방법을 입력
    s = ""
    s += f"Usage: python3 {argv[0]}  command    model_file {' ' * 40}[image/video/dir/...]\n"
    s += f"   Ex: python3 {argv[0]}  raw-image {model_path} ../../assets/data/Image_lite/panda_c1000.bin\n"
    s += f"       python3 {argv[0]}  image     {model_path} ../../assets/data/Image_bmp_c1000/panda.bmp\n"
    s += f"       python3 {argv[0]}  video     {model_path} ../../assets/data/Image_mp4/video_1000class.mp4\n"
    s += f"       python3 {argv[0]}  video     {model_path} ../../assets/data/Image_mp4/video_20class.mp4\n"
    s += f"       python3 {argv[0]}  camera    {model_path} 0\n"
    s += f"       python3 {argv[0]}  slideshow {model_path} ../../assets/data/Image_bmp_c1000/\n"

    # 입력한 내용들 출력
    print(s)

    # Shell에게 명령을 허용하지 않는 다는 알림을 주고 파이썬 모듈 종료. sys.exit(1)과 같은 뜻
    sys.exit(errno.EPERM)


def main(argv: List[str]):
    """
    명령행 인자값을 받아 Demo Class 실행함
    Args:
        argv: 예제 코드 실행시 제공된 인자
              argv[0] demo.py
              argv[1] 영상 타입
              argv[2] 모델 파일 경로
              argv[3] 영상 파일 경로 패턴
    Returns:

    """
    if len(argv) < 4:
        # 만약 인자 값의 개수가 4개 미만이라면 print_usage 실행
        print_usage(argv)

    # Demo Class 생성
    demo = Demo(argv[2])

    # slideshow는 변칙적인 녀석이기에 아래 처럼 처리
    vision_type: VisionTypeEnum
    path: str
    if argv[1] == 'slideshow':
        vision_type = VisionTypeEnum.IMAGE
        path = argv[3] + "/*.bmp"
    else:
        vision_type = VisionTypeEnum(argv[1])
        path = argv[3]

    # Demo Class 실행
    # VisionTypeEnum(argv[1]) demo.py 실행시 제공 받은 argv[1] 인자를 VisionTypeEnum 형태로 변환
    demo.Run(vision_type, path)


if __name__ == "__main__":
    main(sys.argv)
