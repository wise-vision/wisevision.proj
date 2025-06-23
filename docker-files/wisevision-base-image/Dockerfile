ARG BASE_IMAGE=humble-ros-base
FROM ros:${BASE_IMAGE}

WORKDIR /root/wisevision_ws

COPY  . /root/wisevision_ws/src

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /root/wisevision_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

ENTRYPOINT ["/bin/bash", "-c", " \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/wisevision_ws/install/setup.bash && \
    bash"]