# OS_TYPE = "Linux"  # os is Windows or Linux
# CPU_ARCH = "x86_64"  # cpu is x86_64 or armv7l or aarch64
from gtilib.util import GtiGetSDKVersion, GtiGetGtiProcessTime, ProcessTime
from gtilib.enum import (
    GTI_RESPONSE, GTI_DEVICE_STATUS, GTI_DEVICE_TYPE, GTI_CHIP_MODE, TENSOR_FORMAT, GtiImageColorFormat,
    GtiDeviceExceptionCause, GtiModelExceptionCause
)
from gtilib.exception import GtiException, GtiDeviceException, GtiModelException
from gtilib.struct.model import GtiModel
from gtilib.struct.tensor import GtiTensor
from gtilib.factory.factory import GtiModelFactory
from gtilib.struct.device import GtiDevice

__all__ = [
    # function
    GtiGetSDKVersion, GtiGetGtiProcessTime,

    # gtilib struct
    ProcessTime, GtiModel, GtiTensor, GtiDevice,

    # gtilib enum
    GTI_RESPONSE, GTI_DEVICE_STATUS, GTI_DEVICE_TYPE, GTI_CHIP_MODE, TENSOR_FORMAT, GtiImageColorFormat,

    # factory class
    GtiModelFactory,

    # exception cause
    GtiDeviceExceptionCause, GtiModelExceptionCause,

    # exception
    GtiException, GtiDeviceException, GtiModelException
]
