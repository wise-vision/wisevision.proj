FROM ubuntu:22.04

RUN apt-get update && apt-get install -y wget unzip

RUN wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/latest/zenoh-0.11.0-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip && \
    unzip /tmp/zenoh.zip -d /tmp && \
    dpkg -i /tmp/zenoh-plugin-storage-manager_0.11.0_amd64.deb /tmp/zenoh-plugin-rest_0.11.0_amd64.deb /tmp/zenoh_0.11.0_amd64.deb /tmp/zenohd_0.11.0_amd64.deb && \
    rm -rf /tmp/zenoh.zip /tmp/zenoh-plugin-storage-manager_0.11.0_amd64.deb /tmp/zenoh-plugin-rest_0.11.0_amd64.deb /tmp/zenoh_0.11.0_amd64.deb /tmp/zenohd_0.11.0_amd64.deb

RUN wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/latest/zenoh-backend-influxdb-0.11.0-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip && \
    unzip /tmp/zenoh-backend-influxdb.zip -d /tmp && \
    dpkg -i /tmp/zenoh-backend-influxdb-v1_0.11.0_amd64.deb /tmp/zenoh-backend-influxdb-v2_0.11.0_amd64.deb && \
    rm -rf /tmp/zenoh-backend-influxdb.zip /tmp/zenoh-backend-influxdb-v1_0.11.0_amd64.deb /tmp/zenoh-backend-influxdb-v2_0.11.0_amd64.deb

ENTRYPOINT ["zenohd"]

CMD ["-c", "/root/.zenoh/zenoh.json5"]
