FROM ros:humble

RUN apt update && apt install -y \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*
