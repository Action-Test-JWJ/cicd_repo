FROM ros:humble

# 0. ccache + 기본 빌드 도구
RUN apt-get update && apt-get install -y \
    ccache \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    python3-pip \
    python3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# 1. yaml-cpp 0.6.3 (소스 빌드)
RUN cd /tmp && \
    wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.3.zip -O yaml-cpp-0.6.3.zip && \
    unzip -o yaml-cpp-0.6.3.zip && \
    cd yaml-cpp-yaml-cpp-0.6.3 && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/yaml-cpp*

# 2. jsoncpp
RUN apt-get update && apt-get install -y \
    libjsoncpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. OpenCV (소스 빌드 + pip)
RUN apt-get update && apt-get install -y \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev \
    libtiff-dev gfortran openexr libatlas-base-dev \
    libtbb2 libtbb-dev libdc1394-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    git clone --depth 1 https://github.com/opencv/opencv.git && \
    cd opencv && mkdir -p build && cd build && \
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
    .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/opencv

RUN pip3 install numpy natsort opencv-contrib-python==4.8.0.74

# 4. BehaviorTree.CPP (소스 빌드)
RUN apt-get update && apt-get install -y \
    libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    git clone --depth 1 https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && mkdir -p build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/BehaviorTree.CPP

# 5~10. ROS2 패키지 + 시스템 라이브러리
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-joint-state-broadcaster \
    ros-humble-plotjuggler-ros \
    ros-humble-control-toolbox \
    ros-humble-pcl-ros \
    libgeographic-dev \
    libgpiod-dev \
    libserial-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    libasound2-dev \
    alsa-utils \
    flac \
    && rm -rf /var/lib/apt/lists/*

# pip 패키지
RUN pip3 install \
    sparkfun-qwiic-icm20948 \
    dlib==19.24.2 \
    SpeechRecognition \
    pyaudio
