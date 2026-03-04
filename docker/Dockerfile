ARG BASE_IMAGE=jaewook6488/cicd_docker_repo:dev
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-behaviortree-cpp \
    ros-humble-backward-ros \
    ros-humble-yaml-cpp-vendor \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-transmission-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-realtime-tools \
    ros-humble-generate-parameter-library \
    ros-humble-ament-cmake-gmock \
    libeigen3-dev \
    libsdl2-dev \
    libsdl2-image-dev \
    v4l-utils \
    gir1.2-gstreamer-1.0 \
    gstreamer1.0-alsa \
    gstreamer1.0-plugins-good \
    python3-gi \
    && rm -rf /var/lib/apt/lists/*

COPY edie8 /home/ros2_ws/src/edie8

WORKDIR /home/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-up-to aeirobot_llm aeirobot_stt edie_emotion_detection edie_vora_infer_manager_python && \
    colcon build && \
    rm -rf src

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
