from typing import List, Optional

import numpy as np
from numpy import ndarray

from box import BoundingBox, IntersectBoundingBox
from gtilib import GtiModel, GtiTensor

MAXBUFS = 3


class SSDDetector:
    """
    C++ 코드와 비슷. 성능이 낮음
    """

    _gti_model: GtiModel
    INPUT_IMAGE_H: int = 224
    INPUT_IMAGE_W: int = 224
    INPUT_IMAGE_C: int = 3
    CHIPOUT_SCALE: float = 1.167193412

    aChipOutputBuffer: ndarray

    def __init__(self, gti_model: GtiModel) -> None:
        super().__init__()
        self._gti_model = gti_model
        self.aChipOutputBuffer = ndarray(
            shape=(MAXBUFS, 14 * 14 * 30),
            dtype=np.float32
        )

    def detect(self, gti_img: bytes, src_frame_width: int, src_frame_height: int) -> Optional[List[BoundingBox]]:
        self._forward_gti(gti_img, self.aChipOutputBuffer[0])
        return self._forward_sw(self.aChipOutputBuffer[0], src_frame_width, src_frame_height)

    def _forward_gti(self, gti_img: bytes, chip_output_buffer: ndarray):
        out_tensor = self._gti_model.GtiEvaluate(GtiTensor.create_from_img(gti_img, 224, 224, 3))

        # OutTensor에서 만들어진 배열의 축을 바꾸어줍니다.

        out_buff: ndarray = np.array(out_tensor.buff_byte_array, dtype=np.byte)

        out_buff = out_buff.reshape((out_tensor.depth, out_tensor.height, out_tensor.width))
        out_buff = out_buff.transpose(self._create_permute_axes("CHW", "HWC"))
        chip_output_buffer = chip_output_buffer.reshape((14, 14, 30))
        chip_output_buffer[:, :, :] = out_buff[:14, :14, :30] * self.CHIPOUT_SCALE / 31.0

        # 위아 와래는 같은 처리 방법.

        # out_buffer = out_tensor.buff_byte_array
        #
        # idx = 0
        # for i in range(14):
        #     for j in range(14):
        #         for k in range(30):
        #             val = float(out_buffer[14 * 14 * k + i * 14 + j]) * self.CHIPOUT_SCALE / 31.0
        #             chip_output_buffer[idx] = val
        #             idx = idx + 1

    @staticmethod
    def _create_permute_axes(from_axes: str, to_axes: str) -> List[int]:
        """
        사람이 이해할 수 있는 축명칭을 컴퓨터가 이해할 수 있는 형태로 변경합니다.
        ex: get_permute_axes('xy', 'yx') => [1,0]
        """

        return list(map(lambda x: from_axes.find(x), to_axes))

    def _forward_sw(
            self, chip_output_buffer: ndarray, src_frame_width: int, src_frame_height: int
    ) -> Optional[List[BoundingBox]]:
        bounding_boxes: List[BoundingBox] = []

        contain = ndarray(shape=(14, 14, 2), dtype=np.float32)
        cell_size: float = 1.0 / 14.0

        idx = 0

        pre = ndarray(shape=(14, 14, 30), dtype=np.float32)
        for i in range(14):
            for j in range(14):
                for k in range(30):
                    pre[i][j][k] = chip_output_buffer[idx]
                    idx = idx + 1

        for i in range(14):
            for j in range(14):
                contain[i][j][0] = pre[i][j][4]
                contain[i][j][1] = pre[i][j][9]

        max_contain: float = 0.0

        for i in range(14):
            for j in range(14):
                for k in range(2):
                    if contain[i][j][k] > max_contain:
                        max_contain = contain[i][j][k]

        mask = ndarray(shape=(14, 14, 2), dtype=np.int32)
        for i in range(14):
            for j in range(14):
                for k in range(2):
                    if contain[i][j][k] > 0.1:
                        val = 1
                    else:
                        val = 0
                    val += contain[i][j][k] == max_contain

                    if val > 0:
                        mask[i][j][k] = 1
                    else:
                        mask[i][j][k] = 0

        xy = ndarray(shape=(2,), dtype=np.float32)
        box = ndarray(shape=(4,), dtype=np.float32)
        box_xy = ndarray(shape=(4,), dtype=np.float32)

        for i in range(14):
            xy[1] = i * cell_size

            for j in range(14):
                xy[0] = j * cell_size

                for k in range(2):
                    if mask[i][j][k]:
                        for m in range(4):
                            if m < 2:
                                box[m] = pre[i][j][k * 5 + m] * cell_size + xy[m]
                            else:
                                box[m] = pre[i][j][k * 5 + m]

                        contain_probability: float = pre[i][j][k * 5 + 4]

                        for m in range(4):
                            if m < 2:
                                box_xy[m] = box[m] - 0.5 * box[2 + m]
                            else:
                                box_xy[m] = box[m - 2] + 0.5 * box[m]

                        max_probability: float = 0.0
                        cls_index: int = 10

                        for v in range(10, 30):
                            if pre[i][j][v] > max_probability:
                                max_probability = pre[i][j][v]
                                cls_index = v - 10

                        if contain_probability * max_probability > 0.2:
                            bounding_boxes.append(
                                BoundingBox(
                                    x1=box_xy[0] * src_frame_width, y1=box_xy[1] * src_frame_height,
                                    x2=box_xy[2] * src_frame_width, y2=box_xy[3] * src_frame_height,
                                    label=cls_index, score=contain_probability * max_probability
                                )
                            )

        if len(bounding_boxes) < 1:
            return None

        return self._nms(bounding_boxes, 0.5, 1)

    def _nms(self, inferred_bounding_boxes: List[BoundingBox], thresh: float, flag: int) -> List[BoundingBox]:
        """
        Non max suppression

        :param inferred_bounding_boxes:
        :param thresh:
        :param flag:
        :return:
        """

        bounding_boxes: List[BoundingBox] = []
        self._quick_sort(inferred_bounding_boxes, 0, len(inferred_bounding_boxes) - 1, len(inferred_bounding_boxes))

        while len(inferred_bounding_boxes) > 0:
            keep: bool = True

            for bounding_box in bounding_boxes:
                if keep:
                    overlap: float = self._jaccard_overlap(inferred_bounding_boxes[0], bounding_box, flag)
                    keep = overlap <= thresh
                else:
                    break

            if keep:
                bounding_boxes.append(inferred_bounding_boxes[0])

            del inferred_bounding_boxes[0]

        return bounding_boxes

    def _quick_sort(self, inferred_bounding_boxes: List[BoundingBox], left: int, right: int, bboxes_len: int):
        left_bounding_box: BoundingBox = inferred_bounding_boxes[left]
        right_bounding_box: BoundingBox

        initial: int = left
        end: int = right

        if left > right:
            return

        while left != right:
            while inferred_bounding_boxes[right].score <= left_bounding_box.score and left < right:
                right -= 1

            while inferred_bounding_boxes[left].score >= left_bounding_box.score and left < right:
                left += 1

            if left < right:
                right_bounding_box = inferred_bounding_boxes[right]
                inferred_bounding_boxes[right] = inferred_bounding_boxes[left]
                inferred_bounding_boxes[left] = right_bounding_box

        inferred_bounding_boxes[initial] = inferred_bounding_boxes[left]
        inferred_bounding_boxes[left] = left_bounding_box

        self._quick_sort(inferred_bounding_boxes, initial, left - 1, bboxes_len)

        if bboxes_len > left + 1:
            self._quick_sort(inferred_bounding_boxes, left + 1, end, bboxes_len)

    @staticmethod
    def _jaccard_overlap(inferred_bounding_boxes: BoundingBox, bounding_box: BoundingBox, flag: int) -> float:
        """
        Jaccard overlap
        :param inferred_bounding_boxes:
        :param bounding_box:
        :param flag:
        :return:
        """

        intersect_bounding_box = IntersectBoundingBox(inferred_bounding_boxes, bounding_box)

        if flag < 1 or flag > 2:
            raise RuntimeError("Invalid Flag.")

        if intersect_bounding_box.width > 0 and intersect_bounding_box.height > 0:
            intersect_size: float = intersect_bounding_box.size()

            overlap: float
            if flag == 1:
                overlap = intersect_size / (inferred_bounding_boxes.size + bounding_box.size - intersect_size)
            else:
                min_size: float = min(inferred_bounding_boxes.size, bounding_box.size)
                overlap = intersect_size / min_size

            return overlap
        else:
            return .0
