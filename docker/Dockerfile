ARG BASE_IMAGE=jaewook6488/cicd_docker_repo:dev
FROM ${BASE_IMAGE}

COPY edie8 /home/ros2_ws/src/edie8

WORKDIR /home/ros2_ws
RUN sed -i 's/NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH/NO_CMAKE_FIND_ROOT_PATH/g' \
    /opt/ros/humble/share/*/cmake/ament_cmake_export_libraries-extras.cmake && \
    ldconfig && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    rm -rf src

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
