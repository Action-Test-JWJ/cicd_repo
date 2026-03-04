from enum import Enum


class GtiModelExceptionCause(Enum):
    MODEL_NOT_READY = "device not ready"
    NOT_LABEL_LAYER = "not label layer"


class GtiDeviceExceptionCause(Enum):
    INVALID_DEVICE = "Invalid Device"
    NO_AVAILABLE_DEVICES_FOUND = "No available devices found"
    MODEL_IS_ALREADY_DESTROYED = "model is already destroyed"


# noinspection PyPep8Naming
class GTI_RESPONSE(Enum):
    GTI_OK = 0
    GTI_INVALID_PARAM = 1
    GTI_NOT_IMPLEMENTED = 2


# noinspection PyPep8Naming
class GTI_DEVICE_TYPE(Enum):
    GTI_DEVICE_TYPE_ALL = 0
    GTI_DEVICE_USB_FTDI = 1
    GTI_DEVICE_USB_EUSB = 2
    GTI_DEVICE_PCIE = 3
    GTI_DEVICE_VIRTUAL = 4
    GTI_DEVICE_USB_NATIVE = 5
    GTI_DEVICE_TYPE_UNKNOWN = 6
    GTI_DEVICE_TYPE_BUT = 7


# noinspection PyPep8Naming
class TENSOR_FORMAT(Enum):
    TENSOR_FORMAT_BINARY = 0
    TENSOR_FORMAT_BINARY_INTEGER = 1
    TENSOR_FORMAT_BINARY_FLOAT = 2
    TENSOR_FORMAT_TEXT = 3
    TENSOR_FORMAT_JSON = 4
    TENSOR_FORMAT_UNDEFINED = 5
    TENSOR_FORMAT_BUT = 6


# noinspection PyPep8Naming
class GTI_DEVICE_STATUS(Enum):
    GTI_DEVICE_STATUS_ERROR = 0
    GTI_DEVICE_STATUS_ADDED = 1
    GTI_DEVICE_STATUS_REMOVED = 2
    GTI_DEVICE_STATUS_IDLE = 3
    GTI_DEVICE_STATUS_LOCKED = 4
    GTI_DEVICE_STATUS_RUNNING = 5
    GTI_DEVICE_STATUS_PENDING = 6
    GTI_DEVICE_STATUS_UNKNOWN = 7
    GTI_DEVICE_STATUS_BUT = 8


# noinspection PyPep8Naming
class GTI_CHIP_MODE(Enum):
    FC_MODE = 0
    LEARN_MODE = 1
    SINGLE_MODE = 2
    SUBLAST_MODE = 3
    LASTMAJOR_MODE = 4
    LAST7x7OUT_MODE = 5
    SUM7x7OUT_MODE = 6
    GTI_CHIP_MODE_BUT = 7


# noinspection PyPep8Naming
class GtiImageColorFormat(Enum):
    GTI_CF_BGR24_PLANAR = 0
    GTI_CF_RGB24_PLANAR = 1
    GTI_CF_UNDEFINED = 2
    GTI_CF_BUT = 3
