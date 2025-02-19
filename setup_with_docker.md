# Setup Environment on Server with Docker
## Zenoh Server
In terminal:
``` bash
docker run --rm --init --net=host eclipse/zenoh
```
## Zenoh ROS2DDS Plugin
Before run docker create file `DEFAULT_CONFIG.json5`, with:

```json5
{
  "mode": "client",
  "plugins": {
    "ros2dds": {
      "nodename": "zenoh_bridge",
      "ros_localhost_only": false
    }
  },
  "listen": {
    "endpoints": [
      "<proto>/<ip>:<port>"
    ]
  }
}
```
Run in terminal:
``` bash
docker pull eclipse/zenoh-bridge-ros2dds:latest
docker run --net=host -v ~/<path_to_file>/DEFAULT_CONFIG.json5:/root/DEFAULT_CONFIG.json5 -e ROS_DISTRO=humble eclipse/zenoh-bridge-ros2dds:latest -c /root/DEFAULT_CONFIG.json5
```
## ros2_automatic_action_execution

```bash
cd /<ws_path>/ros2_automatic_action_execution
docker build -t ros2_automatic_action_execution .
docker run -d --net=host --ipc=host --pid=host --name ros2_automatic_action_execution_container ros2_automatic_action_execution
docker exec -it ros2_automatic_action_execution_container /bin/bash
```


## Calling Service from Host Terminal
Before service call command:
``` bash
sudo su
source /opt/ros/humble/setup.bash
```
To exit from super user:
``` bash
exit
```

## Connecting Zenoh with InfluxDB and ROS2
To work everything correctly check the version of every zenoh plugin ( on 0.11.0 everything works fine v.1.0.0 has some problems)
### Prerequisites (Local)
*  [Install zenoh server](https://download.eclipse.org/zenoh/zenoh/latest/)
   * Download file for your system
   * Unzip `unzip <file_name.zip>`
   * Install all packages by `sudo dpkg -i *.deb`
* [Install zenoh-backend-influxdb plugin](https://download.eclipse.org/zenoh/zenoh-backend-influxdb/latest/)
   * Download file for your system
   * Unzip `unzip <file_name.zip>`
   * Install all packages by `sudo dpkg -i *.deb`
### Start
* Before run serve, run influxdb in Docker:
``` bash
docker run --rm -p 8086:8086 influxdb:1.8
```
* In on terminal run zenoh server with this config file:
``` json5
{
  plugins: {
    storage_manager: {
      volumes: {
        influxdb: {
          url: "http://localhost:8086",
          private: {
          }
        }
      },
      storages: {
        demo: {
          // the key expression this storage will subscribes to
          key_expr: "test_topic/**",
          volume: {
            id: "influxdb",
            db: "zenoh_example",
            create_db: true,
            on_closure: "do_nothing",
            private: {

            }
          }
        }
      }
    },
    // REST plugin
   rest: { http_port: '8000' }
  },
}

```
* Run:
``` bash
zenohd -c <config_file_name>.json5
```
* [On onther maschine run ros2dds zenoh bridge](dds_router.md#wan-client)
* Start publishing topic with name provide in server config file

## Run Server with InfluxDB in Docker

* Before run serve, run influxdb in Docker:
``` bash
docker run --rm -p 8086:8086 influxdb:1.8
```
In folder with Dockerfile run:
``` bash 
git clone git@github.com:wise-vision/wisevision.proj.git
cd ~/docker-files/zenoh-influxdb-docker
docker build -t my-zenoh-image .
```
After build and create file with config for zenoh:
``` bash
docker run --rm --init --net=host -v /<path_to_cnofig_file>/zenoh.json5:/root/.zenoh/zenoh.json5 my-zenoh-image -c /root/.zenoh/zenoh.json5