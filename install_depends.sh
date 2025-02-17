#!/bin/sh
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

export DEBIAN_FRONTEND=noninteractive
export GRPC_INSTALL_DIR=$HOME/grpc_install_dir


sudo apt-get update

# general
sudo apt-get install -y python3-pip

# wisevision_action_executor
sudo apt-get install -y libyaml-cpp-dev libjsoncpp-dev libcurl4-openssl-dev

# wisevision_notification_manager
sudo apt-get install -y libcurl4-openssl-dev libjsoncpp-dev libyaml-cpp-dev

# wisevision_data_black_box
sudo apt-get install -y curl libcurl4-openssl-dev libjsoncpp-dev

# wisevision_lorawan_bridge
sudo apt-get install -y wget build-essential gcc make cmake libssl-dev autoconf libtool pkg-config
sudo apt-get install -y libpaho-mqtt-dev libpaho-mqttpp-dev
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
sudo apt-get install -y nodejs npm docker.io docker-compose
npm install -y react-router-dom
pip install flask
