ARG BASE_IMAGE=jaewook6488/cicd_docker_repo:dev
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
    gpg wget \
    && wget -qO- https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main" > /etc/apt/sources.list.d/kitware.list \
    && apt-get update && apt-get install -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/*

COPY edie8 /home/ros2_ws/src/edie8

WORKDIR /home/ros2_ws
RUN ldconfig && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    rm -rf src

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
