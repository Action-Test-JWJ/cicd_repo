import os
import platform
from ctypes import CDLL, sizeof, c_void_p, RTLD_GLOBAL

CPU_ARCH: str = platform.uname()[4]
OS_TYPE: str = platform.system()
GTI_SDK_PATH: str = os.environ['GTISDKPATH']
CPU_ARCH: str = CPU_ARCH.replace("AMD64", "x86_64")
GIT_LIB_DIR: str = os.path.join(GTI_SDK_PATH, 'Lib', OS_TYPE, CPU_ARCH)

# 홈 디렉토리 기반 경로
home_dir = os.path.expanduser('~')
base_lib_path = os.path.join(home_dir, 'ros2_ws', 'src', 'edie8', 'edie_vora', 
                            'edie_emotion_detection', 'edie_emotion_detection')

# 가능한 라이브러리 위치들
possible_lib_dirs = [
    home_dir,  # 홈 디렉토리 (현재 사용 중)
    os.path.join(base_lib_path, 'Drivers', 'Linux', 'pcie_drv'),  # 드라이버 폴더
    os.path.join(base_lib_path, 'Lib', OS_TYPE, CPU_ARCH)  # SDK 라이브러리 폴더
]

# 라이브러리 파일 찾기 함수
def find_library(lib_name):
    for dir_path in possible_lib_dirs:
        lib_path = os.path.join(dir_path, lib_name)
        if os.path.exists(lib_path):
            print(f"{lib_name} 발견: {lib_path}")
            return lib_path
    print(f"경고: {lib_name} 파일을 찾을 수 없습니다.")
    return None

# 라이브러리 경로 설정
reader_lib_path = find_library('libimagereader.so')
fc_lib_path = find_library('libfc.so')
label_lib_path = find_library('liblabel.so')

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
