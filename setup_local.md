# Local Environment Setup

## Prerequisites

Ensure you have the following prerequisites installed on your system:
- Git
- VCS Tool
- ROS2 & Colcon


## Install and Run Dependencies

The project requires the following dependencies to be installed and running on your local machine:
- InfluxDB
- Zenoh
- Zenoh ROS2DDS
- ChirpStack (LoRaWAN Network Server)

### 1. Install and Run InfluxDB

#### Install InfluxDB
- **Install**:
``` bash
curl -fsSL https://repos.influxdata.com/influxdata-archive_compat.key | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/influxdata.gpg
echo 'deb [signed-by=/etc/apt/trusted.gpg.d/influxdata.gpg] https://repos.influxdata.com/debian stable main' | sudo tee /etc/apt/sources.list.d/influxdata.list
sudo apt update
sudo apt install influxdb=1.8.*
sudo systemctl enable influxdb
sudo systemctl start influxdb
```

#### Grant permissions
``` bash
sudo ufw allow 8086/tcp
sudo chown -R $USER:$USER /var/lib/influxdb
sudo chown -R influxdb:influxdb /var/lib/influxdb
```

#### Run InfluxDB
``` bash
influxd
```

### 2. [Zenoh local install and run](docs/install_zenoh.md)
Click the link to learn how to install and run Zenoh. Run Zenoh before starting any application.

### 3. [Setup gateway for LoRaWAN communication](docs/set_up_gateway.md)

### 4. [Download, Install and Set Up ChirpStack with Gateway](docs/set_up_chirpstack.md)

Click the link to learn how to install and run Chirpstack with gateway. Run Chirpstack before starting any application.

---

## Local Build

1. Download repositories:
```bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```
2. Install dependencies:
```bash
cd wisevision.proj
./install_depends.sh
```
3. Build:

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
## Set Up Config Files and Run
### Configure and Run wisevision_data_black_box

The black box requires a `config.json` file. The only required field is `zenoh_url`, which should point to the Zenoh server’s URL.

1. Copy `exmaple_config.json` in `.proj` directory:
``` bash
cp config_example.json config.json
```
2. Set environment variables:
```bash
export DB_ADDRESS=localhost
export DB_PORT=8086
```
3. Run:
```bash
source install/setup.bash
ros2 run wisevision_data_black_box black_box
```

### Configure and Run wisevision_lorawan_bridge
1. Start Chirpstack in another terminal and open http://localhost:8080. Log in with admin/admin.
2. Set environment variables.
 - [Create API Key](docs/set_up_chirpstack.md#how-to-create-api-key) and paste into `my_new_token` in command bellow and run this command:
    ```bash
    export CHIRPSTACK_API_KEY=<my_new_token>
    ```
 - On Chirpstack UI [add aplication](docs/set_up_chirpstack.md#how-to-create-application).
  - [Copy `aplication id`](docs/set_up_chirpstack.md#how-to-get-application-id) and paste into `my_new_application_id` in command bellow and run this command:
    ```bash
    export APPLICATION_ID=<my_new_application_id>
    ```
3. Run.
```bash
source install/setup.bash
ros2 run wisevision_lorawan_bridge lorawan_bridge --ros-args --param application_id:=$APPLICATION_ID --param use_only_standard:=false
```

### Run wisevision_action_executor
1. Run:
```bash
source install/setup.bash
ros2 run wisevision_action_executor automatic_action_service
```

###  Run wisevision_gps_tools
1. Run:
```bash
source intsall/setup.bash
ros2 run wisevision_gps_tools gps_device_manager_node
```

###  Run wisevision_notification_manager

1. Run (set use_email_notifier:=t[rue or false]-p use_firebase_notifier:=[true or false]):
```bash
source intsall/setup.bash
ros2 run wisevision_notification_manager notifications_handler --ros-args -p use_email_notifier:=true-p use_firebase_notifier:=false
```

### Run wisevision_dashboard 

1. Install requirments and run server.
```bash
source intsall/setup.bash
cd src/wisevision_dashboard/app/server
pip3 install --no-cache-dir -r requirements.txt
cd ../..
exec gunicorn -k gthread -w 1 --threads 4 -b 0.0.0.0:5000 app.server.run:app"
```
2. Beofre run frontend provied addres to backend server by copy file .`.env_example` into `.env`:
```bash
cp src/wisevision_dashboard/app/client/.env_example src/wisevision_dashboard/app/client/.env
```
> [!NOTE]
> Default addres is set to `http://localhost:5000`, in any case to change it in `.env` file to backend server addres.

2. Install dependencies and run frontend.
```bash
cd src/wisevision_dashboard/app/client
npm install
npm start
```
3. In browser go to address `localhost:5000` to see dashboard.
