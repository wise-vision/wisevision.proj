# Installation Guide for Zenoh and Zenoh Backend

Follow the steps below to install Zenoh and its backend on your local machine.

---

## Prerequisites

Ensure you have `wget` and `unzip` installed on your system.

```bash
sudo apt-get update
sudo apt-get install -y wget unzip
```

### Deb Package Installation

The following steps will guide you through the installation of Zenoh and its backend on your local machine.

```bash
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
sudo apt update
sudo apt install -y zenoh-plugin-ros2dds zenoh-backend-influxdb-v* zenoh-plugin-storage-manager zenohd zenoh-plugin-rest

```

### Manual Installation

For the manual installation, please follow the Zenoh documentation from the [eclipse-zenoh organization](https://github.com/eclipse-zenoh).



### Post Installation

For convinience there is a `example_zenoh_router.json5` file. You can use it as a template for your configuration file.

```bash
mkdir -p /home/$USER/.config/wisevision
cp example_zenoh_router.json5 /home/$USER/.config/wisevision/zenoh.json5
```

Run:
```bash
zenohd -c /home/$USER/.config/wisevision/zenoh.json5
```

#### Auto Start Zenoh

To start Zenoh automatically on system boot, you can use the following command:

```bash
sudo systemctl enable zenohd.service
```

#### Check Zenoh Status

If everything is set up correctly, you can see the following output of the `zenohd` command:

```bash
2025-01-05T15:46:02.282428Z  INFO main ThreadId(01) zenoh::net::runtime: Using ZID: dfa4dc974061d0574a83b61f212d4531
2025-01-05T15:46:02.282515Z  INFO main ThreadId(01) zenoh::api::loader: Loading  plugin "rest"
2025-01-05T15:46:02.283759Z  INFO main ThreadId(01) zenoh::api::loader: Loading  plugin "ros2dds"
2025-01-05T15:46:02.285075Z  INFO main ThreadId(01) zenoh::api::loader: Loading  plugin "storage_manager"
2025-01-05T15:46:02.286457Z  INFO main ThreadId(01) zenoh::api::loader: Starting  plugin "rest"
2025-01-05T15:46:02.289295Z  INFO main ThreadId(01) zenoh::api::loader: Successfully started plugin rest from "/usr/lib/libzenoh_plugin_rest.so"
2025-01-05T15:46:02.289325Z  INFO main ThreadId(01) zenoh::api::loader: Finished loading plugins
2025-01-05T15:46:02.289328Z  INFO main ThreadId(01) zenoh::api::loader: Starting  plugin "ros2dds"
2025-01-05T15:46:02.289532Z  INFO main ThreadId(01) zenoh::api::loader: Successfully started plugin ros2dds from "/usr/lib/libzenoh_plugin_ros2dds.so"
2025-01-05T15:46:02.289537Z  INFO main ThreadId(01) zenoh::api::loader: Finished loading plugins
2025-01-05T15:46:02.289539Z  INFO main ThreadId(01) zenoh::api::loader: Starting  plugin "storage_manager"
2025-01-05T15:46:02.354175Z  INFO main ThreadId(01) zenoh::api::loader: Successfully started plugin storage_manager from "/usr/lib/libzenoh_plugin_storage_manager.so"
2025-01-05T15:46:02.354218Z  INFO main ThreadId(01) zenoh::api::loader: Finished loading plugins
2025-01-05T15:46:02.354543Z  INFO main ThreadId(01) zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/192.168.0.188:7447
2025-01-05T15:46:02.354662Z  INFO main ThreadId(01) zenoh::net::runtime::orchestrator: zenohd listening scout messages on 224.0.0.224:7446
```

In case of issues please check the permissions. 
