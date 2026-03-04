from ctypes import cast, POINTER, c_void_p

from gtilib import GtiModel
from gtilib.native import module


class GtiModelFactory:
    @staticmethod
    def create_by_file(model_file_path: str) -> GtiModel:
        return GtiModel(
            cast(module.GtiCreateModel(model_file_path.encode('ascii')), POINTER(c_void_p))
        )

    @staticmethod
    def create_by_buffer(model_buffer: bytes) -> GtiModel:
        return GtiModel(
            cast(module.GtiCreateModelFromBuffer(model_buffer, len(model_buffer)), POINTER(c_void_p))
        )
