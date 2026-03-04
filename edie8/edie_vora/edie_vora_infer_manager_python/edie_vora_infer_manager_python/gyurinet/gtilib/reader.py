from abc import abstractmethod, ABC
from enum import Enum
from glob import glob
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import time
import cv2
from numpy import ndarray, concatenate, split, fromfile, uint8

from gyurinet.gtilib.exception import CameraVisionReaderException, CameraVisionReaderReadException
from gyurinet.gtilib.util import convert_opencv2_img_to_gti_img, convert_gti_img_to_opencv


class VisionTypeEnum(Enum):
    CAMERA = "camera"
    VIDEO = "video"
    IMAGE = "image"
    RAW_IMAGE = "raw-image"


class VisionReader(ABC):
    _current: int
    _prev_idx: int
    _prev_img: Optional[ndarray]

    _gti_img_width: int  # 224, 448, 320 640
    _gti_img_height: int  # 224, 448, 320 640

    def __init__(self, gti_img_width: int, gti_img_height: int) -> None:
        self._current = 0
        self._prev_idx = -1
        self._prev_img = None
        self._gti_img_width = gti_img_width
        self._gti_img_height = gti_img_height

    def __iter__(self):
        return self

    def __next__(self) -> int:
        if self._is_end(self._current):
            self._current = 0
            raise StopIteration

        prev = self._current
        self._current = self._current + 1

        return prev

    @abstractmethod
    def _is_end(self, idx: int):
        pass

    @abstractmethod
    def get_opencv_img(self, idx: int) -> ndarray:
        pass
    
    def get_gti_img(self, idx: int) -> bytes:
        return convert_opencv2_img_to_gti_img(self.get_opencv_img(idx), self._gti_img_width, self._gti_img_height)


class ImageVisionReader(VisionReader):
    _img_path_list: List[str]
    _img_count: int

    def __init__(self, path_pattern: str, gti_img_width: int, gti_img_height: int) -> None:
        super().__init__(gti_img_width, gti_img_height)
        self._img_path_list = []

        for filepath in glob(path_pattern):
            self._img_path_list.append(filepath)

        self._img_count = len(self._img_path_list)
        if self._img_count == 0:
            raise FileNotFoundError(f"could not find {path_pattern} as a media file/dir")

    def _is_end(self, idx: int):
        return idx >= self._img_count

    def get_opencv_img(self, idx: int) -> ndarray:
        return cv2.imread(self._img_path_list[idx])

    def get_gti_img(self, idx: int) -> bytes:
        try:
            return convert_opencv2_img_to_gti_img(self.get_opencv_img(idx), self._gti_img_width, self._gti_img_height)
        except (Exception, ) as e:
            raise RuntimeError(e, self._img_path_list[idx])


class VideoCaptureVisionReader(VisionReader, ABC):
    _video_capture: Optional[cv2.VideoCapture]

    def __init__(self, gti_img_width: int, gti_img_height: int) -> None:
        super().__init__(gti_img_width, gti_img_height)
        self._video_capture = None

    @property
    def video_capture(self) -> cv2.VideoCapture:
        return self._video_capture

    @property
    def is_opened(self) -> bool:
        return self._video_capture.isOpened()

    @property
    def width(self) -> int:
        return int(self._video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))

    @property
    def height(self) -> int:
        return int(self._video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def resize_from_video_capture_source(self, window_name: str, window_min_width: int, window_min_height):
        new_width = min(float(window_min_width), self._video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        new_height = min(float(window_min_height), self._video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cv2.resizeWindow(window_name, int(new_width), int(new_height))

    def set_size(self, width: int, height: int) -> bool:
        # https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d

        return self._video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width) and \
            self._video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


class VideoFileVisionReader(VideoCaptureVisionReader):
    """
    이 클래스는 Video 파일, 캠용으로 사용한다.
    Video 파일이 원본인 경우 _from_cam 이 false
    """
    _frame_count: float
    _video_capture: Optional[cv2.VideoCapture]

    def __init__(self, path: str, gti_img_width: int, gti_img_height: int) -> None:
        super().__init__(gti_img_width, gti_img_height)

        self._video_capture = None

        self._open(path)

    def _open(self, path: str):
        self._video_capture = cv2.VideoCapture(path)
        self._frame_count = self._video_capture.get(cv2.CAP_PROP_FRAME_COUNT)

        if not self._video_capture.isOpened():
            raise RuntimeError(f"could not open Video File: {path}")

    def __del__(self):
        self._video_capture.release()

    def _is_end(self, idx: int):
        return idx >= self._frame_count

    def get_opencv_img(self, idx: int) -> ndarray:
        if idx < self._prev_idx:
            # self._video_capture.get(cv2.CAP_PROP_POS_FRAMES, idx)
            self._video_capture.set(cv2.CAP_PROP_POS_FRAMES, idx)

        if self._prev_idx != idx:
            read_success, img = self._video_capture.read()
            if not read_success:
                raise RuntimeError("Video Capture Read error")

            self._prev_idx = idx
            self._prev_img = img

        return self._prev_img

class CameraVisionReader(VideoCaptureVisionReader):
    """
    이 클래스는 Video 파일, 캠용으로 사용한다.
    캠은 끝없이 영상이 재생되지만 VisionReader 에서 정의한 Generator 가 사용하는 Int값이 4바이트를 넘지 않는 것이 성능상 유리
    파이썬은 이러한 제약이 없으면 무한으로 수를 확장시킨다.

    Video 파일이 원본인 경우 _from_cam 이 false
    """
    _video_capture: Optional[cv2.VideoCapture]
    _MAX = 2 << 31

    def __init__(self, path: str, gti_img_width: int, gti_img_height: int) -> None:
        super().__init__(gti_img_width, gti_img_height)
        
        src = int(path) if isinstance(path, str) and path.isdigit() else path
        self._video_capture = cv2.VideoCapture(src)
        if not self._video_capture.isOpened():
            raise RuntimeError(f"could not open camera: {path}")
        
        self._prev_idx = -1
        self._prev_img = None
        self._current = 0    

    def __del__(self):
        try:
            if self._video_capture is not None:
                self._video_capture.release()
        except Exception:
            pass

    def _is_end(self, idx: int):
        return idx > self._MAX or not self.is_opened

    def get_opencv_img(self, idx: int) -> Optional[ndarray]:
        if self._prev_idx != idx:
            ok, img = self._video_capture.read()
            if not ok or img is None:
                raise RuntimeError("Camera Read error")
            self._prev_img = img
            self._prev_idx = idx
        return self._prev_img
    
    def __next__(self) -> int:
        if self._is_end(self._current):
            self._current = 0
            raise StopIteration
        prev = self._current
        self._current += 1
        return prev

class GTIRawImageVisionReader(ImageVisionReader):
    def __init__(self, path_pattern: str, gti_img_width: int, gti_img_height: int) -> None:
        super().__init__(path_pattern, gti_img_width, gti_img_height)

    def get_opencv_img(self, idx: int) -> ndarray:
        if idx != self._prev_idx:
            self._prev_img = convert_gti_img_to_opencv(self._img_path_list[idx], self._gti_img_width,
                                                       self._gti_img_height)

        return self._prev_img

    def get_gti_img(self, idx: int) -> bytes:
        raw_img = fromfile(self._img_path_list[idx], dtype=uint8)
        return concatenate(split(raw_img, 3)).tobytes()


def create_vision_reader(
        vision_type: VisionTypeEnum, path_pattern: str, gti_img_width: int = 224, gti_img_height: int = 224
) -> VisionReader:
    if vision_type == VisionTypeEnum.CAMERA:
        return CameraVisionReader(path_pattern, gti_img_width, gti_img_height)
    if vision_type == VisionTypeEnum.VIDEO:
        return VideoFileVisionReader(path_pattern, gti_img_width, gti_img_height)
    if vision_type == VisionTypeEnum.IMAGE:
        return ImageVisionReader(path_pattern, gti_img_width, gti_img_height)
    if vision_type == VisionTypeEnum.RAW_IMAGE:
        return GTIRawImageVisionReader(path_pattern, gti_img_width, gti_img_height)

    raise RuntimeError(f"iterator of {vision_type} is not implemented")