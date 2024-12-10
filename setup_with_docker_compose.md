# How to run docker compose?
## Before run
- Clone repos:
``` bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```
- [Download, Install and Set Up ChirpStack with Gateway](docs/set_up_chirpstack.md)
### Configurations

#### Config Zenoh 

The zenoh dds which is currently the only one supported requires a configuration file to be presentfile should be named `zenoh.json`. The example file is provided as `example_zenoh.json5` by default can be copied into.

``` bash
cp example_zenoh_router.json5 zenoh_router.json
```

#### Config wisevision_data_black_box

The black box requires a configuration file and to be named `config.json`. 

The only required field is `zenoh_url` which should be the url of the zenoh server.  The example file is provided as `example_config.json5` by default can be copied into.

Copy `exmaple_config.json` into `ros2_black_box` directory
``` bash
cp config_example.json src/ros2_black_box/config.json
```
#### Config ros2_lora_bridge
1. Create `.env` file for enviroment variables
``` bash
cd ~/wisevision.proj
cp src/ros2_lora_bridge/.env_example src/ros2_lora_bridge/.env
```
 - [Create API Key](docs/set_up_chirpstack.md#how-to-create-api-key) and paste into `my_new_token` in command bellow and run this command:
```
 sed -i 's/^API_TOKEN=.*/API_TOKEN=my_new_token/' /src/ros2_lora_bridge/.env
 ```
 - On chirpstack ui [add aplication](docs/set_up_chirpstack.md#how-to-create-application).
   - [Copy `aplication id`](docs/set_up_chirpstack.md#how-to-get-application-id) and paste into `my_new_application_id` in command bellow and run this command:
  ```
  sed -i 's/^APPLICATION_ID=.*/APPLICATION_ID=my_new_application_id/' /src/ros2_lora_bridge/.env
  ```
#### Config wisevision_notificator_manager
1. Creat config files
- Config for email
``` bash
cd ~/wisevision.proj 
cp src/ros2_notifications/config_example.yaml src/ros2_notifications/config.yaml
```
After copy fill file with personl data
- Config for `wisevision_notificator_app`
``` bash
cd ~/wisevision.proj 
cp src/ros2_notifications/deviceTokens_example.json src/ros2_notifications/deviceTokens.json
```
After copy fill file with personl data
Download file from firebase console with service account password `serviceAccount.json` and copy it to `notifications_ws` or for docker in `~/notifications_ws/src/ros2_notifcations`
* Go to firebase console
* Go to project setting by click on gear icon
* In settings go to Service accounts
* In service accounts choose `Java` in admin SDK configuration and click on `Generate new private key`
* Copy this file to `notifications_ws`
2. In file `docker-compose.yml` in `wisevision.proj` dricetory set args variables:
```docker-compose
args:
  - USE_EMAIL_NOTIFIER: "<true or false>"
  - USE_FIREBASE_NOTIFIER: "<true or false>"
```

### Config wisevision_dashboard
1. Creat config files
- Config for email
``` bash
cd ~/wisevision.proj 
cp src/wisevision-dashboard/app/client/.env_example src/wisevision-dashboard/app/client/.env
```
`.env` File has address of the backend server. To use dashboard outside localhost change it to your server ip. File `.env` has default local address.

## Run

``` bash
docker-compose up
```
