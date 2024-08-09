#!/bin/bash

# Install libpaho-mqtt-dev non-interactively
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y libpaho-mqtt-dev  libpaho-mqttpp-dev