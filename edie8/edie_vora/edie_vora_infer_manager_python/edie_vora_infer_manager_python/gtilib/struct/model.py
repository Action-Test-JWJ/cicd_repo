# ***********************************************************************
#
# Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
# See LICENSE file in the project root for full license information.
#
# ***********************************************************************

"""
 * \file GTILib.h
 * \brief GTI header file includes the public functions of GTI SDK library.
"""
import json
from ctypes import (
    c_int, c_void_p, c_char_p,
    Structure, POINTER, pointer
)
from typing import Optional

from gtilib.enum import GtiDeviceExceptionCause, GtiModelExceptionCause
from gtilib.exception import GtiDeviceException, GtiModelException
from gtilib.native import module
from gtilib.struct.tensor import GtiTensor


class GtiModel(Structure):
    """
    GTI Model Python Wrapper
    Create an instance with GtiModelFactory.
    """
    ptr: Optional[POINTER(c_void_p)]

    _fields_ = [("ptr", POINTER(c_void_p))]

    def __init__(self, ptr: POINTER(c_void_p)) -> None:
        super().__init__(ptr)

        if not ptr:
            raise GtiDeviceException(GtiDeviceExceptionCause.NO_AVAILABLE_DEVICES_FOUND)

    def __del__(self):
        if self.ptr is not None:
            module.GtiDestroyModel(self.ptr)

    # noinspection PyPep8Naming
    def GtiEvaluate(self, tensor: GtiTensor) -> Optional[GtiTensor]:
        module.GtiEvaluate.restype = POINTER(GtiTensor)

        result = module.GtiEvaluate(self.ptr, pointer(tensor))
        if result:
            return result.contents

        raise GtiModelException(GtiModelExceptionCause.MODEL_NOT_READY)

    # noinspection PyPep8Naming
    def GtiImageEvaluate(self, image: bytes, width: int, height: int, depth) -> Optional[str]:
        result = module.GtiImageEvaluate(self.ptr, image, width, height, depth)
        if result:
            return str(bytes(result).decode())

        raise GtiModelException(GtiModelExceptionCause.MODEL_NOT_READY)

    # noinspection PyPep8Naming
    def GtiEvaluateWithBuffer(self, input_tensor: GtiTensor, customer_buffer: Optional[bytes]) -> Optional[GtiTensor]:
        if customer_buffer is None:
            result = module.GtiEvaluateWithBuffer(
                self.ptr, pointer(input_tensor), customer_buffer, 0
            )
        else:
            result = module.GtiEvaluateWithBuffer(
                self.ptr, pointer(input_tensor), customer_buffer, len(customer_buffer)
            )

        if result:
            return result.contents

        raise GtiModelException(GtiModelExceptionCause.MODEL_NOT_READY)

    # noinspection PyPep8Naming
    def GtiDestroyModel(self):
        if self.ptr is not None:
            try:
                return module.GtiDestroyModel(self.ptr)
            finally:
                self.ptr = None

        raise GtiDeviceException(GtiDeviceExceptionCause.MODEL_IS_ALREADY_DESTROYED)

    # noinspection PyPep8Naming
    def GtiQueryModelLayerInfo(
            self, layer_index: int, input_tensor: GtiTensor, output_tensor: GtiTensor
    ) -> int:
        return module.GtiQueryModelLayerInfo(self.ptr, layer_index, pointer(input_tensor), pointer(output_tensor))

    # noinspection PyPep8Naming
    @staticmethod
    def GtiGetLayerInputTensorInfo(model_buffer: bytes, layer_index: int, tensor: GtiTensor) -> int:
        return module.GtiGetLayerInputTensorInfo(model_buffer, layer_index, pointer(tensor))

    # noinspection PyPep8Naming
    @staticmethod
    def GtiGetLayerOutputTensorInfo(model_buffer: bytes, layer_index: int, tensor: GtiTensor) -> int:
        return module.GtiGetLayerOutputTensorInfo(model_buffer, layer_index, pointer(tensor))

    # noinspection PyPep8Naming
    @staticmethod
    def GetModelJsonConfigurationFromFilePath(model_file_path: str):
        with open(model_file_path, "rb") as fp:
            json_length = 0
            while True:
                ret = int.from_bytes(fp.read(1), 'big')

                if ret == 0:
                    break
                json_length += 1
            fp.seek(0)

            json_bytes = fp.read(json_length)

            json_dict = json.loads(json_bytes)
            json_dict['json size'] = json_length
            return json_dict

    # noinspection PyPep8Naming
    @staticmethod
    def GetLabelByGtiModelLayerConfiguration(model_file_path: str, layer: dict) -> str:
        if layer['operation'] == 'LABEL':
            data_size = layer['data size']
            data_offset = layer['data offset']

            with open(model_file_path, "rb") as fp:
                fp.seek(data_offset)
                buff = fp.read(data_size)
                return buff.decode()
        else:
            raise GtiModelException(GtiModelExceptionCause.NOT_LABEL_LAYER)


module.GtiCreateModelFromBuffer.argtypes = [c_void_p, c_int]
module.GtiCreateModelFromBuffer.restype = c_void_p

module.GtiCreateModel.argtypes = [c_char_p]
module.GtiCreateModel.restype = c_void_p

module.GtiDestroyModel.argtypes = [c_void_p]
module.GtiDestroyModel.restype = c_int

module.GtiImageEvaluate.restype = c_char_p
module.GtiImageEvaluate.argtypes = [c_void_p, c_char_p, c_int, c_int, c_int]

module.GtiEvaluate.argtypes = [c_void_p, c_void_p]
module.GtiEvaluate.restype = POINTER(GtiTensor)

module.GtiEvaluateWithBuffer.argtypes = [c_void_p, c_void_p, c_char_p, c_int]
module.GtiEvaluateWithBuffer.restype = POINTER(GtiTensor)

module.GtiGetLayerInputTensorInfo.argtypes = [c_char_p, c_int, c_void_p]
module.GtiGetLayerInputTensorInfo.restype = c_int

module.GtiGetLayerOutputTensorInfo.argtypes = [c_char_p, c_int, c_void_p]
module.GtiGetLayerOutputTensorInfo.restype = c_int

module.GtiQueryModelLayerInfo.argtypes = [c_void_p, c_int, c_void_p, c_void_p]
module.GtiQueryModelLayerInfo.restype = c_int
