# Installation Guide for Zenoh and Zenoh Backend

Follow the steps below to manually install Zenoh and its backend on your local

---

## Prerequisites

Ensure you have `wget` and `unzip` installed on your system.

```bash
sudo apt-get update
sudo apt-get install -y wget unzip
```

## Step 1: Download and Install Zenoh
``` bash
# Download the 0.11 Zenoh release
wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/latest/zenoh-1.1.0-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip

# Extract the downloaded files
unzip /tmp/zenoh.zip -d /tmp

# Install the extracted .deb packages
sudo dpkg -i /tmp/zenoh-plugin-storage-manager_1.1.0_amd64.deb /tmp/zenoh-plugin-rest_1.1.0_amd64.deb /tmp/zenoh_1.1.0_amd64.deb /tmp/zenohd_1.1.0_amd64.deb

# Clean up temporary files
rm -rf /tmp/zenoh.zip /tmp/zenoh-plugin-storage-manager_1.1.0_amd64.deb /tmp/zenoh-plugin-rest_1.1.0_amd64.deb /tmp/zenoh_1.1.0_amd64.deb /tmp/zenohd_1.1.0_amd64.deb
```

## Step 2: Download and Install Zenoh Backend for InfluxDB

``` bash
# Download the latest Zenoh Backend for InfluxDB
wget https://ftp.fau.de/eclipse/zenoh/zenoh-backend-influxdb/latest/zenoh-backend-influxdb-1.1.0-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh-backend-influxdb.zip

# Extract the downloaded files
unzip /tmp/zenoh-backend-influxdb.zip -d /tmp

# Install the extracted .deb packages
sudo dpkg -i /tmp/zenoh-backend-influxdb-v1_1.1.0_amd64.deb /tmp/zenoh-backend-influxdb-v2_1.1.0_amd64.deb

# Clean up temporary files
rm -rf /tmp/zenoh-backend-influxdb.zip /tmp/zenoh-backend-influxdb-v1_1.1.0_amd64.deb /tmp/zenoh-backend-influxdb-v2_1.1.0_amd64.deb
```

## Step 3: Download and Install Zenoh Plugin for ROS2

``` bash
# Download the latest Zenoh Plugin for ROS2DDS
wget -O /tmp/zenoh-plugin.zip https://ftp.fau.de/eclipse/zenoh/zenoh-plugin-ros2dds/1.1.0/zenoh-plugin-ros2dds-1.1.0-x86_64-unknown-linux-gnu-debian.zip

# Extract the downloaded files
unzip /tmp/zenoh-plugin.zip -d /tmp

# Install the extracted .deb packages for ROS2DDS
sudo dpkg -i /tmp/zenoh-bridge-ros2dds_1.1.0_amd64.deb /tmp/zenoh-plugin-ros2dds_1.1.0_amd64.deb || sudo apt-get install -f -y

# Clean up temporary files
rm -rf /tmp/zenoh-plugin.zip /tmp/zenoh-bridge-ros2dds_1.1.0_amd64.deb /tmp/zenoh-plugin-ros2dds_1.1.0_amd64.deb
```

## Step 4: Running Zenoh
Prequistances:
- Create config file `zenoh.json5` for zenoh.
```json
{
{
  "mode": "router",
  "plugins": {
    "ros2dds": {
      "nodename": "zenoh_bridge",
      "ros_localhost_only": false
    },
    "storage_manager": {
      "volumes": {
        "influxdb": {
          "url": "http://localhost:8086"
        }
      },
      "storages": {
        "demo": {
          "key_expr": "sensor_publisher_e5_board_temp/**",
          "volume": {
            "id": "influxdb",
            "db": "zenoh_example_1",
            "create_db": true,
            "on_closure": "do_nothing"
          }
        },
        "devices": {
          "key_expr": "devices_data/**",
          "volume": {
            "id": "influxdb",
            "db": "zenoh_devices_data",
            "create_db": true,
            "on_closure": "do_nothing"
          }
        },
        "demo_test": {
          "key_expr": "notifications/**",
          "volume": {
            "id": "influxdb",
            "db": "zenoh_example_2",
            "create_db": true,
            "on_closure": "do_nothing",
            "private": {
              "encoding": "base64"
            }
          }
        }
      }
    },
    "rest": {
      "http_port": 8000
    }
  },
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"
    ]
  }
}
```

Run:
```bash
zenohd -c /path/to/zenoh.json5
```