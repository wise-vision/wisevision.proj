# How to set up local?
## Prequistances

## Install and run dependencies
### 1. Influxdb install and run
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

### 2. [Zenoh local install and run](docs/install_zenoh.md)
Clicl on this to see how to intsall and run zenoh. Run before start any app.

### 3. Download and intsall chirsptack
1. Clone repo:
```bash
git clone https://github.com/chirpstack/chirpstack-docker.git
cd chirpstack-docker
```
2. Run `chirpstack`
``` bash
docker-compose up
```
## Locacl Build

1. Download repositories
```bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```
2. Instal dependencies
```bash
cd wisevision.proj
./install_depends.sh
```
3. Build

Export variables to be able to load built shared libraries and include headers. It is recommended to put those variables inside .bashrc file.
```bash
export GRPC_INSTALL_DIR=$HOME/grpc_install_dir
export PATH=$GRPC_INSTALL_DIR/bin:$PATH
export LD_LIBRARY_PATH=$GRPC_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```
Build:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Set up config files and run
### Config wisevision_data_black_box and run

The black box requires a configuration file and to be named `config.json`. 

The only required field is `zenoh_url` which should be the url of the zenoh server.  The example file is provided as `example_config.json5` by default can be copied into.

1. Copy `exmaple_config.json` into `ros2_black_box` directory.
``` bash
cp config_example.json config.json
```
2. Before run set environment variables.
```bash
export DB_ADDRESS=localhost
export DB_PORT=8086
```
3. Run.
```bash
source install/setup.bash
ros2 run black_box black_box
```

#### Config ros2_lora_bridge and run
Before run `ros2_lora_bridge` run in another terminal `chirpstack`
1. After start chirpstack go to http://localhost:8080 and login with admin/admin.
2. Before run set environment variables.
 - On chirpstack ui after login on the left bar go to `API keys` and than creat API key by click on the `Add API key`, provide name for API key -> genearte `API key` -> copy `API key` -> paste into `my_new_token` in command bellow and run this command
    ```
    export API_TOKEN=<my_new_token>
    ```
 - On chirpstack ui add aplication: 
    - On the left bar clic on `Application`
    - In the right corner click on `Add aplication`
    - Provide name of the application
    - On the top of the UI copy `aplication id` paste into `my_new_application_id` in command bellow and run this command
    ```
    export APPLICATION_ID=<my_new_application_id>
    ```
- Before run ros2_lora_bridge start gateway in another terminal: https://github.com/Lora-net/sx1302_hal
3. Run:
```bash
source install/setup.bash
ros2 run ros2_lora_bridge ros2_lora_bridge --ros-args  --param application_id:=$APPLICATION_ID
```

### ros2_automatic_action_execution run
1. Run:
```bash
source install/setup.bash
ros2 run automatic_action_execution automatic_action_service
```

###  wisevision_gps_tools run
1. Run:
```bash
source intsall/setup.bash
ros2 run wisevision_gps_tools gps_device_manager_node
```

###  wisevision_notification_manager run
**Push notifications**
* In `notifications_ws`  or for docker in `~/notifications_ws/src/ros2_notifcations` create `deviceTokens.json`
```bash
cd ~/notifications_ws
mkdir deviceTokens.json
```
In this file add [device token from app](https://github.com/wise-vision/notificator_app/blob/c_k/dev_android_app/README.md#L20) in this way:
```json
{
 "devices": [
    {"token": "your-device-token-from-app"}
  ]
}
```
* Download file from firebase console with service account password `serviceAccount.json` and copy it to `wisevision.proj`
    * Go to firebase console
    * Go to project setting by click on gear icon
    * In settings go to Service accounts
    * In service accounts choose `Java` in admin SDK configuration and click on `Generate new private key`
    * Copy this file to `wisevision.proj`


**Email**

Before start create in  `wisevision.proj`  file `config_email.yaml` with:
```yaml
smtp_server: "smtp://stmp_server"
username: "sender_email"
password: "app_passowrd"
recipients:
  - "recipient_email_1"
  - "recipient_email_2"
```
**Hints**
- To use mail as smtp server go to security settings in mail and create app password
1. Run (set use_email_notifier:=t[rue or false]-p use_firebase_notifier:=[true or false]):
```bash
source intsall/setup.bash
ros2 run notifications notifications_handler --ros-args -p use_email_notifier:=true-p use_firebase_notifier:=false
```

### wisevision_dashboard 

1. Install requuirments and run server.
```bash
source intsall/setup.bash
cd src/wisevision_dashboard/app/server
pip3 install --no-cache-dir -r requirements.txt
cd ../..
python3 -m app.server.run
```
2. Install dependencies an run frontend.
```bash
cd src/wisevision_dashboard/app/client
npm install
npm start
```
