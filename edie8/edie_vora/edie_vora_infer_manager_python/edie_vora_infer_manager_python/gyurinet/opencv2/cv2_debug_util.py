from typing import Dict

import cv2
from numpy import empty, ndarray

_debug_titles: Dict[int, str] = {}


def imshow(mat: ndarray, idx: int, prefix: str = 'debug'):
    cv2.imshow(get_debug_window_title(idx, prefix), mat)


def get_debug_window_title(idx: int, prefix: str = 'debug'):
    if idx not in _debug_titles:
        title = f"{prefix}{idx}"
        cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
        _debug_titles[idx] = title

    return _debug_titles[idx]


def create_empty_mat_24bit(width: int, height: int, color: int) -> ndarray:
    mat = empty((height, width, 3), dtype="uint8")
    mat[::, ::] = color
    return mat
