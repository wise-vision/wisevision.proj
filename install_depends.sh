#!/bin/bash

# Install libpaho-mqtt-dev non-interactively
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y libpaho-mqtt-dev  libpaho-mqttpp-dev
# ros2_automatic_action_execution
sudo apt-get install libyaml-cpp-dev
# ros2_lora_bridge
sudo apt install protobuf-compiler
# dash-board
sudo apt install nodejs npm
sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
sudo apt install docker-compose
npm install
npm install react-router-dom
pip install flask