import os
import platform
from ctypes import CDLL, sizeof, c_void_p, RTLD_GLOBAL

CPU_ARCH: str = platform.uname()[4]
OS_TYPE: str = platform.system()
GTI_SDK_PATH: str = os.environ['GTISDKPATH']
CPU_ARCH: str = CPU_ARCH.replace("AMD64", "x86_64")
GIT_LIB_DIR: str = os.path.join(GTI_SDK_PATH, 'Lib', OS_TYPE, CPU_ARCH)

if OS_TYPE.lower() == "windows":
    LIBNAME: str = "GTILibrary.dll"
    os.environ['PATH'] += f";{GIT_LIB_DIR}"
else:
    LIBNAME: str = "libGTILibrary.so"

GIT_LIB_MODULE_PATH = os.path.join(GIT_LIB_DIR, LIBNAME)

try:
    module: CDLL = CDLL(GIT_LIB_MODULE_PATH, mode=RTLD_GLOBAL)

    ARCH_SIZE = sizeof(c_void_p)
    if ARCH_SIZE == 4:
        print("Running on a 32-bit machine")
    elif ARCH_SIZE == 8:
        print("Running on a 64-bit machine")
    else:
        raise Exception('Cannot determine machine type')
except Exception as e:
    print("Could not find %s in the current package directory" % LIBNAME)
    print(type(e), e.args)
    print("Try to find %s in the system path" % LIBNAME)
    raise e
