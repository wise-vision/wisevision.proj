FROM ubuntu:22.04

RUN apt-get update && apt-get install -y wget unzip

# Set environment variables for version and architecture
ARG ZENOH_VERSION=1.0.0
ARG ARCH=$(uname -m)

# Use conditions to download appropriate Zenoh version depending on architecture
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/latest/zenoh-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip; \
    elif [ "$ARCH" = "aarch64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh/latest/zenoh-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    unzip /tmp/zenoh.zip -d /tmp && \
    dpkg -i /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb /tmp/zenoh_${ZENOH_VERSION}_*.deb /tmp/zenohd_${ZENOH_VERSION}_*.deb && \
    rm -rf /tmp/zenoh.zip /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb /tmp/zenoh_${ZENOH_VERSION}_*.deb /tmp/zenohd_${ZENOH_VERSION}_*.deb

# Use conditions to download appropriate Zenoh InfluxDB backend depending on architecture
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/latest/zenoh-backend-influxdb-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip; \
    elif [ "$ARCH" = "aarch64" ]; then \
        wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/latest/zenoh-backend-influxdb-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip; \
    else \
        echo "Unsupported architecture: $ARCH" && exit 1; \
    fi && \
    unzip /tmp/zenoh-backend-influxdb.zip -d /tmp && \
    dpkg -i /tmp/zenoh-backend-influxdb-v1_${ZENOH_VERSION}_*.deb /tmp/zenoh-backend-influxdb-v2_${ZENOH_VERSION}_*.deb && \
    rm -rf /tmp/zenoh-backend-influxdb.zip /tmp/zenoh-backend-influxdb-v1_${ZENOH_VERSION}_*.deb /tmp/zenoh-backend-influxdb-v2_${ZENOH_VERSION}_*.deb
    
# Set the entrypoint and default command
ENTRYPOINT ["zenohd"]
CMD ["-c", "/root/.zenoh/zenoh.json5"]