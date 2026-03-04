from ctypes import cast, POINTER, c_void_p, c_char_p, c_int
from typing import Optional, TYPE_CHECKING, List

from gtilib import GTI_RESPONSE, GTI_DEVICE_TYPE, GtiDeviceException
from gtilib.enum import GtiDeviceExceptionCause
from gtilib.native import module

if TYPE_CHECKING:
    from pyudev import Context


class GtiDevice:
    _ptr: Optional[POINTER(c_void_p)]
    _device_name: str

    def __init__(self, device_name: str) -> None:
        super().__init__()
        self._ptr = None
        self._device_name = device_name

    # noinspection PyPep8Naming
    def GtiDeviceOpen(self):
        self._ptr = cast(module.GtiDeviceOpen(self._device_name.encode('ascii')), POINTER(c_void_p))

        if self._ptr is None:
            raise GtiDeviceException(GtiDeviceExceptionCause.NO_AVAILABLE_DEVICES_FOUND)

    # noinspection PyPep8Naming
    def GtiDeviceClose(self) -> GTI_RESPONSE:
        ret = module.GtiDeviceClose(self._ptr)
        if ret == -1:
            return GTI_RESPONSE.GTI_INVALID_PARAM

        return GTI_RESPONSE(ret)

    # noinspection PyPep8Naming
    def GtiGetDeviceType(self) -> GTI_DEVICE_TYPE:
        ret = module.GtiGetDeviceType(self._ptr)
        if ret >= 0:
            return GTI_DEVICE_TYPE(ret)

        # GTI_RESPONSE.GTI_INVALID_PARAM
        raise GtiDeviceException(GtiDeviceExceptionCause.INVALID_DEVICE)

    @classmethod
    def find_all(cls) -> List[str]:
        """
        이 메소드를 사용하기 위해선 pyudev가 설치되어 있어야합니다.
        수동으로 창치는 찾는법
           $ demsg
           $ usb 2-9.4: Manufacturer: FTDI
           $ udevadm info /sys/bus/usb/devices/2-9.4 -qname
           $ bus/usb/002/012
           디바이스노드=/dev/bus/usb/002/012

        :return:
        """
        from pyudev import Context

        founds = []

        ctx = Context()

        # VORA USB 2803 판매 버전
        for device in ctx.list_devices(subsystem='usb'):
            if 'ID_VENDOR' not in device.properties or 'ID_MODEL_ID' not in device.properties:
                continue

            vendor = device.properties['ID_VENDOR']
            model = device.properties['ID_MODEL_ID']

            if vendor == "FTDI" and device.device_type == 'usb_device' and model == '601f':
                founds.append(device.device_node)

        # VORA USB 2803 초기버전
        for device in ctx.list_devices(subsystem='scsi_generic'):
            if 'usb' in device.device_path:
                target_device = device.parent.parent.parent.parent.parent

                if 'ID_VENDOR' not in target_device.properties or 'ID_MODEL_ID' not in target_device.properties:
                    continue

                vendor = target_device.properties['ID_VENDOR']
                model = target_device.properties['ID_MODEL_ID']

                if vendor == 'Generic' and model == '0769':
                    founds.append(device.device_node)

        return founds


module.GtiDeviceOpen.argtypes = [c_char_p]
module.GtiDeviceOpen.restype = c_void_p

module.GtiDeviceClose.argtypes = [c_void_p]
module.GtiDeviceClose.restype = c_int

module.GtiGetDeviceType.argtypes = [c_void_p]
module.GtiGetDeviceType.restype = c_int
