# How to set up local?
## Influxdb install and run
- Install:
``` bash
wget -qO- https://repos.influxdata.com/influxdb.key | sudo apt-key add -
echo "deb https://repos.influxdata.com/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/influxdb.list
sudo apt update
sudo apt install influxdb=1.8.*
sudo systemctl start influxdb
sudo systemctl enable influxdb
```
- Run:
``` bash
influxd
```
- Problem with permissions:
``` bash
sudo chown -R <user_name>:<user_name> /var/lib/influxdb
```

## Zenoh local install and run

- Install:
``` bash
# Set the Zenoh version
ZENOH_VERSION=0.11.0
ARCH=$(uname -m)

# Update and install necessary packages
sudo apt-get update && sudo apt-get install -y wget unzip

# Download the appropriate Zenoh version depending on the architecture
if [ "$ARCH" = "x86_64" ]; then
    wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip
elif [ "$ARCH" = "aarch64" ]; then
    wget https://ftp.fau.de/eclipse/zenoh/zenoh/${ZENOH_VERSION}/zenoh-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

# Unzip and install Zenoh packages
sudo unzip /tmp/zenoh.zip -d /tmp
sudo dpkg -i /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh_${ZENOH_VERSION}_*.deb \
       /tmp/zenohd_${ZENOH_VERSION}_*.deb

# Clean up downloaded files
sudo rm -rf /tmp/zenoh.zip \
       /tmp/zenoh-plugin-storage-manager_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh-plugin-rest_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh_${ZENOH_VERSION}_*.deb \
       /tmp/zenohd_${ZENOH_VERSION}_*.deb

# Download the appropriate Zenoh InfluxDB backend depending on the architecture
if [ "$ARCH" = "x86_64" ]; then
    wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/${ZENOH_VERSION}/zenoh-backend-influxdb-${ZENOH_VERSION}-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip
elif [ "$ARCH" = "aarch64" ]; then
    wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/${ZENOH_VERSION}/zenoh-backend-influxdb-${ZENOH_VERSION}-aarch64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

# Unzip and install InfluxDB backend for Zenoh
sudo unzip /tmp/zenoh-backend-influxdb.zip -d /tmp
sudo dpkg -i /tmp/zenoh-backend-influxdb-v1_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh-backend-influxdb-v2_${ZENOH_VERSION}_*.deb

# Clean up downloaded files
sudo rm -rf /tmp/zenoh-backend-influxdb.zip \
       /tmp/zenoh-backend-influxdb-v1_${ZENOH_VERSION}_*.deb \
       /tmp/zenoh-backend-influxdb-v2_${ZENOH_VERSION}_*.deb

```
- Run (in directory with `zenoh.json5` file, Run after runinning influxdb):
``` bash
zenohd -c zenoh.json5
```
