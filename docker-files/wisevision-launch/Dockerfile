FROM wisevision/ros_with_wisevision_msgs_wisevision_core_and_grpc:humble


ENV ROS_PARAM_FILE=/root/wisevision_proj_ws/src/ros_params.yaml
ENV USE_EMAIL_NOTIFIER=true
ENV USE_FIREBASE_NOTIFIER=false

# Build env for lorawan bridge
ENV GRPC_INSTALL_DIR=/root/grpc_install_dir
ENV PATH=$GRPC_INSTALL_DIR/bin:$PATH
ENV LD_LIBRARY_PATH=${GRPC_INSTALL_DIR}/lib:/usr/local/lib



WORKDIR /root/wisevision_proj_ws

COPY  . /root/wisevision_proj_ws/src

RUN apt-get update && \
    apt-get install -y libyaml-cpp-dev && \
    rosdep fix-permissions && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src -r -y --rosdistro humble
SHELL ["/bin/bash", "-c"]

RUN echo '{' > /root/wisevision_proj_ws/config.json && \
    echo '  "zenoh_url": "http://localhost:8000/"' >> /root/wisevision_proj_ws/config.json && \
    echo '}' >> /root/wisevision_proj_ws/config.json

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/wisevision_ws/install/setup.bash && \
    colcon build --packages-select wisevision_action_executor wisevision_gps_tools wisevision_notification_manager wisevision_data_black_box wisevision_lorawan_bridge chirpstack_api standard_parser"



ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/wisevision_proj_ws/install/setup.bash && ros2 launch /root/wisevision_proj_ws/src/launch/launch.py config_file:=/root/wisevision_proj_ws/src/ros_params.yaml"]