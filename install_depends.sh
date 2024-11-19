#!/bin/bash

export DEBIAN_FRONTEND=noninteractive


sudo apt-get update
# general
sudo apt-get install -y python3-pip
# ros2_automatic_action_execution
sudo apt-get -y install libyaml-cpp-dev
# ros2_lora_bridge
sudo apt-get install -y libpaho-mqtt-dev  libpaho-mqttpp-dev
sudo apt install  -y protobuf-compiler libboost-all-dev
# dash-board
sudo apt install -y nodejs npm docker.io docker-compose
npm install -y react-router-dom
pip install flask