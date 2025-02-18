FROM  wisevision/ros_with_wisevision_msgs_and_wisevision_core:humble

ENV GRPC_INSTALL_DIR=/root/grpc_install_dir


RUN apt-get update && \
    apt-get install -y wget build-essential gcc make cmake libssl-dev autoconf libtool pkg-config && \
    apt-get install -y libpaho-mqtt-dev  libpaho-mqttpp-dev && \
    apt-get install -y libboost-all-dev && \
    export GRPC_INSTALL_DIR=$HOME/grpc_install_dir && \
    mkdir -p $GRPC_INSTALL_DIR && \
    git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc $HOME/grpc && \
    cd $HOME/grpc && \
    mkdir -p cmake/build && \
    cd cmake/build && \
    cmake -DCMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR \
          -DBUILD_SHARED_LIBS=ON \
          -DgRPC_BUILD_GRPC_CPP_PLUGIN=ON \
          -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF \
          -DgRPC_BUILD_GRPC_NODE_PLUGIN=OFF \
          -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF \
          -DgRPC_BUILD_GRPC_PHP_PLUGIN=OFF \
          -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF \
          -DgRPC_BUILD_PROTOBUF=ON \
          -DgRPC_BUILD_GRPC_RUBY_PLUGIN=OFF \
          ../.. && \
    make -j 4 && \
    make install && \
    cd -


SHELL ["/bin/bash", "-c"]

ENTRYPOINT ["/bin/bash", "-c", " \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/wisevision_ws/install/setup.bash && \
    export PATH=${GRPC_INSTALL_DIR}/bin:${PATH} && \
    export LD_LIBRARY_PATH=${GRPC_INSTALL_DIR}/lib:${LD_LIBRARY_PATH} && \
    bash"]