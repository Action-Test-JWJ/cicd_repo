FROM ros:humble AS build

COPY test_counter /ros2_ws/src/test_counter

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build && \
    colcon test && \
    colcon test-result --verbose

FROM ros:humble

COPY --from=build /ros2_ws/install /ros2_ws/install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "test_counter", "counter_node"]
