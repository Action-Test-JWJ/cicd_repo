#!/usr/bin/env python3

from typing import List, Optional, Tuple
from numpy import ndarray
from gyurinet.gtilib.util import convert_opencv2_img_to_gti_img
from box import BoundingBox
from detector import SSDDetector
from gtilib import GtiModelFactory
from box_util import BoxDrawer
from gyurinet.opencv2 import Point, Color, Size
import cv2, os, time, sys

class Demo:
    def __init__(
            self,
            model_path: str = "~/ros2_ws/src/edie8/edie_vora/edie_human_detection/src/gti_gnetdet_2803.model",
            window_name: str = "SSDDetection",
            gti_img_width: int = 224, # 224, 448, 320, 640
            gti_img_height: int = 224, # 224, 448, 320, 640
            show_window: bool = True,
    ):
        model_path = os.path.expanduser(model_path)
        self.detector = SSDDetector(GtiModelFactory.create_by_file(model_path))
        self.box_drawer = BoxDrawer()
        self._gti_w, self._gti_h = gti_img_width, gti_img_height
        self.window_name = window_name
        self.show_window = show_window
        self.wait_key_delay = 1  # <== 기본 delay (1ms)

        if self.show_window:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE + cv2.WINDOW_GUI_NORMAL)

    def ProcessFrame(self, frame: ndarray) -> Optional[Tuple[int, int, int, int]]:
        """한 장 처리해서 (x, y, w, h) 또는 None을 반환."""
        t0 = time.time()

        gti_img: bytes = convert_opencv2_img_to_gti_img(frame, self._gti_w, self._gti_h)
        boxes: List[BoundingBox] = self.detector.detect(gti_img, frame.shape[1], frame.shape[0])

        center_point, max_w, max_h = self.box_drawer.draw(frame, boxes)

        if self.show_window:
            # 좌측 상단 FPS 정보
            fps = 1.0 / max(1e-6, (time.time() - t0))
            cv2.putText(frame, f"{fps:.3f} fps", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color.GREEN)

            # 키 입력 처리
            if self.wait_key_delay == 0:
                font_face = cv2.FONT_HERSHEY_COMPLEX
                font_scale = 1
                thickness = 1
                font_shape, base_line = cv2.getTextSize("Pause", font_face, font_scale, thickness)
                font_shape = Size(*font_shape)
                left = frame.shape[1] - font_shape.width - 10
                top = frame.shape[0] - font_shape.height
                cv2.putText(frame, "Pause", Point(left, top),
                            font_face, font_scale, Color.RED, thickness)

            # 화면 표시
            cv2.imshow(self.window_name, frame)

            # 키 입력 처리
            key = cv2.waitKey(self.wait_key_delay) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                print("Shutting down...")
                sys.exit(0)
            elif key == ord(' '):
                # 토글 pause
                if self.wait_key_delay == 0:
                    self.wait_key_delay = 1
                else:
                    self.wait_key_delay = 0

        if center_point is None:
            return None

        return center_point.x, center_point.y, max_w, max_h