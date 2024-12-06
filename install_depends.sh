#!/bin/bash

export DEBIAN_FRONTEND=noninteractive


sudo apt-get update
# general
sudo apt-get install -y python3-pip
# ros2_automatic_action_execution
sudo apt-get -y install libyaml-cpp-dev libjsoncpp-dev libcurl4-openssl-dev
# ros2_notifications
sudo apt-get install -y libcurl4-openssl-dev libjsoncpp-dev libyaml-cpp-dev
# ros2_black_box
sudo apt-get install -y curl libcurl4-openssl-dev libjsoncpp-dev
# ros2_lora_bridge

sudo apt-get install -y wget build-essential gcc make cmake libssl-dev autoconf libtool pkg-config
sudo apt-get install -y libpaho-mqtt-dev libpaho-mqttpp-dev
sudo apt-get install -y libboost-all-dev
mkdir -p $GRPC_INSTALL_DIR
git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc $HOME/grpc
cd $HOME/grpc
mkdir -p cmake/build
pushd cmake/build
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
      ../..
make -j 4
make install
popd
# dash-board
sudo apt install -y nodejs npm docker.io docker-compose
npm install -y react-router-dom
pip install flask