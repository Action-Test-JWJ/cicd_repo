from pathlib import Path
from typing import Union, BinaryIO

import cv2
from numpy import ndarray, concatenate, fromfile, uint8, split


def convert_opencv2_img_to_gti_img(opencv2_img: ndarray, width: int, height: int) -> bytes:

    resized_img = cv2.resize(opencv2_img, (width, height))
    gti_img = concatenate(cv2.split(resized_img)).tobytes()
    return gti_img


def convert_gti_img_to_opencv(file: Union[str, BinaryIO, Path], width: int, height: int):
    """
    sdk/Tools/imageTool 로 bin 파일을 만들수 있는데 이를 역으로 만들면 아래 코드가 나온다.
    이는 GTI가 해석하기 좋은 이미지 파일을 다시 OpenCV가 해석 가능한 형태로 변경하는 코드이다.

    Args:
        file: gti tensor input buffer에 바로 넣을수 있는 파일 경로
        width: gti 이미지 파일 width
        height: gti 이미지 파일 height

    Returns:
    """

    buff = fromfile(file, dtype=uint8)

    ch_b: ndarray
    ch_g: ndarray
    ch_r: ndarray

    ch_b, ch_g, ch_r = split(buff, 3)
    ch_b = ch_b.reshape((height, width))
    ch_g = ch_g.reshape((height, width))
    ch_r = ch_r.reshape((height, width))

    return cv2.merge((ch_b, ch_g, ch_r))

