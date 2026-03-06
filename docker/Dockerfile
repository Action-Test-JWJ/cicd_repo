ARG BASE_IMAGE=jaewook6488/cicd_docker_repo:dev
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y rsync && rm -rf /var/lib/apt/lists/*

COPY edie8 /tmp/edie8_new

WORKDIR /home/ros2_ws
RUN rsync -a --checksum /tmp/edie8_new/ src/edie8/ && \
    rm -rf /tmp/edie8_new && \
    . /opt/ros/humble/setup.sh && \
    colcon build && \
    rm -rf src

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
