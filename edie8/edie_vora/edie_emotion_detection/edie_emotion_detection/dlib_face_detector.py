from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import os
import cv2
from dlib import fhog_object_detector, rectangles, rectangle, get_frontal_face_detector, shape_predictor
from numpy import ndarray

from gyurinet.opencv2 import Point


@dataclass
class DlibFaceDetectorResult:
    locations: rectangles
    face_img_list: List[ndarray]
    lm_list: List[ndarray]


class DlibFaceDetector:
    _ratio: float
    _detector: fhog_object_detector

    def __init__(self, ratio: float):
        """
        얼굴 영역을 탐색 하는 객체
        Args:
            ratio: 원본 이미지의 사이즈를 줄여서 얼굴 탐색 속도를 증가 시킴.(정확도는 떨어짐)
        """
        """https://github.com/davisking/dlib-models"""
        """https://debuggercafe.com/face-detection-with-dlib-using-cnn/"""

        self._detector = get_frontal_face_detector()
        
        #shape_predictor_68_face_landmarks.dat 경로
        home_dir = os.path.expanduser("~")
        predictor_path = os.path.join(home_dir, 'ros2_ws/src/edie8/edie_vora/edie_emotion_detection/edie_emotion_detection/shape_predictor_68_face_landmarks.dat')
        self.predictor = shape_predictor(predictor_path)

        self._ratio = max(1.0, ratio)

    # dlib detect
    def detect(self, org_frame: ndarray, threshold: float) -> Optional[DlibFaceDetectorResult]:
        """
        얼굴을 인식

        Args:
            org_frame:
            threshold:

        Returns:
        """

        ratio = self._ratio

        detector_frame = cv2.resize(
            org_frame,
            Point(int(org_frame.shape[1] / ratio), int(org_frame.shape[0] / ratio)),
            interpolation=cv2.INTER_CUBIC
        )

        faces: List[ndarray] = []
        lm: List[ndarray] = []
        gray_frame: ndarray = cv2.cvtColor(detector_frame, cv2.COLOR_BGR2GRAY)

        scores: List[float]
        locations: rectangles
        indexes: List[int]

        locations, scores, indexes = self._detector.run(gray_frame, 1, threshold)

        org_frame_height = org_frame.shape[0]
        org_frame_width = org_frame.shape[1]

        resized_locations = rectangles()

        has_face = False
        for loc in locations:
            predictions = self.predictor(gray_frame, loc)            
            landmarks = np.array([(pt.x, pt.y) for pt in predictions.parts()],
                                 dtype=np.float64)
            has_face = True

            top = int(self.clamp(loc.top() * ratio, 0, org_frame_height))
            bottom = int(self.clamp(loc.bottom() * ratio, 0, org_frame_height))

            left = int(self.clamp(loc.left() * ratio, 0, org_frame_width))
            right = int(self.clamp(loc.right() * ratio, 0, org_frame_width))

            resized_locations.append(rectangle(left, top, right, bottom))

            faces.append(org_frame[top:bottom, left:right])
            lm.append(landmarks)
        # if __debug__:
        #     print('detect faces: ', len(faces))

        if has_face:
            return DlibFaceDetectorResult(resized_locations, faces,lm)

        return None

        # fast cropping

    @staticmethod
    def clamp(val, min_val, max_val):
        return max(min(max_val, val), min_val)

    # cell_size: minimum face size, cell_size x cell_size
    # num_h: horizontal divisions 
    # num_v: vertical divisions 
    @staticmethod
    def fast_crop(frame: ndarray, cell_size, num_h, num_v):
        # get image_width, image_height
        (image_height, image_width) = frame.shape[:2]
        print('h = ', image_height, ' w = ', image_width)

        locs: List[rectangle] = []
        faces = []

        block_width = (image_width - cell_size) / num_h + cell_size
        block_height = (image_height - cell_size) / num_v + cell_size
        for i in range(num_h):
            for j in range(num_v):
                top = (block_height - cell_size) * j
                bottom = (block_height - cell_size) * j + block_height
                left = (block_width - cell_size) * i
                right = (block_width - cell_size) * i + block_width
                locs.append(rectangle(left, top, right, bottom))
                faces.append(frame[top:bottom, left:right])
        has_face = True
        return has_face, locs, faces