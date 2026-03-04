from ctypes import c_char_p, Structure, c_uint, POINTER
from dataclasses import dataclass, field

from packaging import version
from packaging.version import Version

from gtilib.enum import GTI_RESPONSE
from gtilib.native import module

__module_version = None


# noinspection PyPep8Naming
def GtiGetSDKVersion() -> Version:
    global __module_version
    if __module_version is None:
        __module_version = version.parse(bytes.decode(module.GtiGetSDKVersion(), "ascii")[len("GTK SDK "):])
    return __module_version


module.GtiGetSDKVersion.argtypes = []
module.GtiGetSDKVersion.restype = c_char_p


@dataclass
class ProcessTime(Structure):
    write: int
    read: int
    success: GTI_RESPONSE = field(default=GTI_RESPONSE.GTI_OK)

    _fields_ = [
        ('write', c_uint),
        ('read', c_uint)
    ]

    def __init__(self, write: int = 0, read: int = 0, success: GTI_RESPONSE = GTI_RESPONSE.GTI_OK) -> None:
        super().__init__(
            c_uint(write),
            c_uint(read),
        )
        self.success = success


# noinspection PyPep8Naming
def GtiGetGtiProcessTime() -> ProcessTime:
    process_time = ProcessTime()
    success = module.GtiGetGtiProcessTime(process_time)
    process_time.success = GTI_RESPONSE(success)

    return process_time


module.GtiGetGtiProcessTime.argtypes = [POINTER(ProcessTime)]
module.GtiGetGtiProcessTime.restype = GTI_RESPONSE
