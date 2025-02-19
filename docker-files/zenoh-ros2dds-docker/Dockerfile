# 
#  Copyright (C) 2025  wisevision
# 
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

FROM ros:humble

RUN apt-get update && apt-get install -y wget unzip

# Set environment variables for version and architecture
ARG ZENOH_VERSION=0.11.0
ARG ARCH=$(uname -m)

# Use conditions to download appropriate Zenoh version depending on architecture
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip; \
    elif [ "$ARCH" = "aarch64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    unzip /tmp/zenoh.zip -d /tmp && \
    dpkg -i /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb /tmp/zenoh_${ZENOH_VERSION}_*.deb /tmp/zenohd_${ZENOH_VERSION}_*.deb && \
    rm -rf /tmp/zenoh.zip /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb /tmp/zenoh_${ZENOH_VERSION}_*.deb /tmp/zenohd_${ZENOH_VERSION}_*.deb

RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh-plugin-ros2dds/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-ros2.zip; \
    elif [ "$ARCH" = "aarch64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh-plugin-ros2dds/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-ros2.zip; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    unzip /tmp/zenoh-ros2.zip -d /tmp && \
    dpkg -i /tmp/zenoh-bridge-ros2dds_${ZENOH_VERSION}_*.deb && \
    dpkg -i /tmp/zenoh-plugin-ros2dds_${ZENOH_VERSION}_*.deb  && \
    rm -rf /tmp/zenoh-ros2.zip /tmp/zenoh-bridge-ros2dds_${ZENOH_VERSION}_*.deb /tmp/zenoh-plugin-ros2dds_${ZENOH_VERSION}_*.deb
    
# Set the entrypoint and default command
ENTRYPOINT ["zenohd"]
CMD ["-c", "/root/.zenoh/zenoh_client.json5"]