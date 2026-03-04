#!/bin/bash
set -e

# === [자동으로 ROS2 배포판 이름 설정] ===
if [ -n "$ROS_DISTRO" ]; then
    distro="$ROS_DISTRO"
    echo -e "\033[1;32m[INFO] Detected ROS_DISTRO from environment: $distro\033[0m"
else
    echo -e "\033[1;31m[ERROR] Cannot determine ROS 2 distro. Please source ROS 2 setup or set ROS_DISTRO manually.\033[0m"
    exit 1
fi

echo "==== [EDIE8 의존성 자동 설치 스크립트] ===="

# 0. ccache 설치 (컴파일 속도 향상)
echo -e "\n==== [0] ccache 설치 ===="
echo "컴파일 속도 향상을 위한 ccache를 설치합니다."
sudo apt-get update
sudo apt-get install -y ccache

sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# ccache 설정 확인
if [ -f "/usr/lib/ccache/gcc" ] && [ -f "/usr/lib/ccache/g++" ]; then
    echo -e "\033[1;32m[INFO] ccache가 정상적으로 설치되었습니다.\033[0m"
else
    echo -e "\033[1;33m[WARNING] ccache 설치는 되었지만 심볼릭 링크가 없을 수 있습니다. 계속 진행합니다.\033[0m"
fi

# 1. yaml-cpp (소스 빌드)
echo -e "\n==== [1] yaml-cpp (소스 빌드) ===="
echo "yaml-cpp 0.6.3을 소스에서 빌드합니다."
echo "이미 설치되어 있다면 이 단계는 건너뛰세요."
cd ~/

# 기존 파일 정리
rm -rf yaml-cpp-yaml-cpp-0.6.3 yaml-cpp-0.6.3.zip

wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.3.zip -O yaml-cpp-0.6.3.zip
unzip -o yaml-cpp-0.6.3.zip
cd yaml-cpp-yaml-cpp-0.6.3
mkdir -p build && cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig  # 라이브러리 링크 업데이트
cd ~/
rm -rf yaml-cpp-0.6.3.zip yaml-cpp-yaml-cpp-0.6.3

# 2. jsoncpp (apt)
echo -e "\n==== [2] jsoncpp ===="
sudo apt update
sudo apt install -y libjsoncpp-dev

# 3. OpenCV (소스 빌드 + pip)
echo -e "\n==== [3] OpenCV (소스 빌드) ===="
echo "OpenCV를 소스에서 빌드합니다. 이미 설치되어 있다면 이 단계는 건너뛰세요."
cd ~/

# OpenCV 필수 의존성 먼저 설치
sudo apt-get install -y python3-pip build-essential cmake git pkg-config \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev \
    libtiff-dev gfortran openexr libatlas-base-dev python3-dev \
    python3-numpy libtbb2 libtbb-dev libdc1394-dev

pip3 install numpy
pip3 install natsort

# 기존 opencv 디렉토리가 있으면 업데이트, 없으면 클론
if [ -d "opencv" ]; then
    echo "기존 OpenCV 리포지토리를 업데이트합니다."
    cd opencv && git pull origin master
else
    echo "OpenCV 리포지토리를 클론합니다."
    git clone https://github.com/opencv/opencv.git
    cd opencv
fi

mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from sysconfig import get_paths as gp; print(gp()['include'])") \
-D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "import site; print(site.getsitepackages()[0])") \
-D PYTHON3_LIBRARY=$(python3 -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))") \
-D WITH_GTK=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
..
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~/
pip3 install opencv-contrib-python==4.8.0.74

# 4. BehaviorTree.CPP (소스 빌드)
echo -e "\n==== [4] BehaviorTree.CPP (소스 빌드) ===="
sudo apt-get install -y libzmq3-dev
cd ~/

# 기존 BehaviorTree.CPP 디렉토리가 있으면 업데이트, 없으면 클론
if [ -d "BehaviorTree.CPP" ]; then
    echo "기존 BehaviorTree.CPP 리포지토리를 업데이트합니다."
    cd BehaviorTree.CPP && git pull origin master
else
    echo "BehaviorTree.CPP 리포지토리를 클론합니다."
    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
    cd BehaviorTree.CPP
fi

mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
cd ~/

# 5. joy & teleop_joy (apt)
echo -e "\n==== [5] joy & teleop_joy ===="
sudo apt install -y ros-$distro-joy ros-$distro-teleop-twist-joy

# 6. localization (apt)
echo -e "\n==== [6] localization ===="
sudo apt install -y libgeographic-dev libgpiod-dev

# 7. libserial (apt)
echo -e "\n==== [7] libserial ===="
sudo apt-get install -y libserial-dev

# pip install mujoco xacro

# 8. IMU 드라이버
echo -e "\n==== [8] ICM20948 Driver ===="
sudo pip3 install sparkfun-qwiic-icm20948
sudo usermod -aG i2c $USER

# 9. HRI 관련 패키지
echo -e "\n==== [9] HRI 관련 패키지 ===="
pip3 install dlib==19.24.2


# === [STT 및 오디오 관련 의존성 추가 설치] ===
sudo apt-get update
sudo apt-get install -y portaudio19-dev libportaudio2 libportaudiocpp0 libasound2-dev python3-dev alsa-utils
sudo apt install flac -y
pip3 install SpeechRecognition pyaudio

# 10. ros2 관련 패키지
echo -e "\n==== [10] ros2 관련 패키지 ===="
echo "ros2 관련 패키지를 설치합니다."
sudo apt-get install -y \
    ros-${distro}-ros2-control \
    ros-${distro}-ros2-controllers \
    ros-${distro}-xacro \
    ros-${distro}-gazebo-ros \
    ros-${distro}-gazebo-ros2-control \
    ros-${distro}-robot-localization \
    ros-${distro}-joint-state-broadcaster \
    ros-${distro}-plotjuggler-ros \
    ros-${distro}-control-toolbox \
    ros-${distro}-pcl-ros


echo -e "\n==== [EDIE8 의존성 설치 완료] ===="
echo -e "\033[1;32m[SUCCESS] 모든 의존성이 성공적으로 설치되었습니다! sudo reboot 진행해주세요.\033[0m"
