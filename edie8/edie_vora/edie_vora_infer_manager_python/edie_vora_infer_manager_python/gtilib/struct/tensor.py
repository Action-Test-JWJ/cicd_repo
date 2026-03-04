from ctypes import Structure, c_int, c_char_p, c_void_p, POINTER, cast, c_byte, c_float, sizeof
from typing import Optional, Union, List

from numpy import ndarray

from gtilib import TENSOR_FORMAT
from gtilib.util import GtiGetSDKVersion


class GtiTensor(Structure):
    width: int
    height: int
    depth: int
    stride: int
    ptr_buffer: c_void_p
    ptr_customer_buffer: any
    size: int
    tensor_format: TENSOR_FORMAT
    tag: Optional[bytes]
    next_tensor: Optional['GtiTensor']

    def __init__(
            self, width: int, height: int, depth: int, stride: int,
            buffer: Union[bytes, ndarray], customer_buffer: any, size: int,
            tensor_format: TENSOR_FORMAT, tag: Optional[bytes] = None, next_tensor: Optional['GtiTensor'] = None
    ) -> None:
        if isinstance(buffer, bytes):
            ptr_buffer = cast(c_char_p(buffer), c_void_p)
        elif isinstance(buffer, ndarray):
            ptr_buffer = buffer.ctypes.data
        else:
            raise RuntimeError("Unsupported Buffer Type")

        if GtiGetSDKVersion().major >= 5:
            super().__init__(
                c_int(width),
                c_int(height),
                c_int(depth),
                c_int(stride),  # stride, unused
                ptr_buffer,
                c_char_p(customer_buffer),
                c_int(size),
                c_int(tensor_format.value),
                c_char_p(tag),
                cast(c_void_p(next_tensor), POINTER(GtiTensor)),  # internal
                c_void_p(None),  # internal
                c_void_p(None)  # internal
            )
        else:
            super().__init__(
                c_int(width),
                c_int(height),
                c_int(depth),
                c_int(0),  # stride, unused
                ptr_buffer,
                c_int(size),
                c_int(tensor_format.value)
            )

    @property
    def buffer_string(self) -> str:
        return cast(self.ptr_buffer, c_char_p).value.decode() if self.ptr_buffer is not None else None

    @property
    def buffer_bytes(self) -> bytes:
        return bytes(cast(self.ptr_buffer, POINTER(c_byte * self.size)).contents) \
            if self.ptr_buffer is not None else None

    @property
    def buff_byte_array(self) -> bytearray:
        return bytearray(cast(self.ptr_buffer, POINTER(c_byte * self.size)).contents) \
            if self.ptr_buffer is not None else None

    @property
    def buff_float_array(self) -> List[float]:
        return list(cast(self.ptr_buffer, POINTER(c_float * int(self.size / sizeof(c_float)))).contents) \
            if self.ptr_buffer is not None else None

    @classmethod
    def create_from_img(
            cls, img: Union[bytes, ndarray], img_width: int, img_height: int, color_channel: int
    ) -> 'GtiTensor':
        return cls(
            img_width, img_height, color_channel, 0, img, None, len(img), TENSOR_FORMAT.TENSOR_FORMAT_BINARY, None
        )


if GtiGetSDKVersion().major >= 5:
    GtiTensor._fields_ = [("width", c_int),
                          ("height", c_int),
                          ("depth", c_int),
                          ("stride", c_int),
                          ("ptr_buffer", c_void_p),
                          ("customerBuffer", c_char_p),
                          ("size", c_int),  # buffer size
                          ("format", c_int),  # tensor format
                          ("tag", c_char_p),
                          ("next", POINTER(GtiTensor)),  # gtilib internal_only
                          ("reserved", c_void_p),
                          ("privateData", c_void_p)]
else:
    GtiTensor._fields_ = [
        ("width", c_int),
        ("height", c_int),
        ("depth", c_int),
        ("stride", c_int),
        ("ptr_buffer", c_void_p),
        ("size", c_int),  # buffer size
        ("format", c_int)  # tensor format
    ]
