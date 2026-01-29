FROM ros:kilted-perception

WORKDIR /root/ros2_ws

RUN apt update && apt install -y \
    python3-serial \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY src /root/ros2_ws/src

RUN /bin/bash -c "source /opt/ros/kilted/setup.bash && colcon build"

RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
