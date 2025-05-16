# Docker Compose Setup
## Before run
- Clone repos:
``` bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```
- [Download, Install and Set Up ChirpStack with Gateway](docs/set_up_chirpstack.md)
### Configurations

#### Config Zenoh 

The zenoh dds which is currently the only one supported requires a configuration file to be presentfile should be named `zenoh_router.json5`. The example file is provided as `example_zenoh_router.json5` by default can be copied into.

``` bash
cp example_zenoh_router.json5 zenoh_router.json5
```

### Env variables for ROS components:
Create file in `wisevision.proj` directory `.env` by:
```bash
touch .env
```
and fill it with:
```txt
ROS_PARAM_FILE=/root/wisevision_proj_ws/src/ros_params.yaml
USE_EMAIL_NOTIFIER=true
USE_FIREBASE_NOTIFIER=false
EMAIL_USERNAME_NOTIFICATION=<myuser@example.com>
EMAIL_PASSWORD_NOTIFICATION=<supersecret>
EMAIL_RECIPIENTS_NOTIFICATION=<admin@example.com>
DEVICE_TOKENS_FIREBASE=<token1,token2>
CHIRPSTACK_API_KEY=<your_chripstack_api_key>
```
**Chirpstak setup**
 - [Create API Key](docs/set_up_chirpstack.md#how-to-create-api-key) and paste into `your_chripstack_api_key`.

**Firebase setup**

Download file from firebase console with service account password `serviceAccount.json` and copy it to `wisevision.proj`
* Go to firebase console
* Go to project setting by click on gear icon
* In settings go to Service accounts
* In service accounts choose `Java` in admin SDK configuration and click on `Generate new private key`
* Copy this file to `wisevision.proj`
* Set `USE_FIREBASE_NOTIFIER` in ROS components to true

 

### ROS parameters file
Create file in `wisevision.proj` directory `ros_params.yaml` by:
```bash
touch ros_params.yaml
```
and fill it with:
```yaml
notification_handler:
  ros__parameters:
    use_email_notifier: true # default is true
    use_firebase_notifier: false # default is false
    smtp_server: "<smtp_server>"
    service_account_path: "<service_account_path>"
lorawan_bridge:
  ros__parameters:
    application_id: "application_id"
    use_only_standard: false # default is true
```

### Config wisevision_dashboard
1. Creat config files
``` bash
cd ~/wisevision.proj 
cp src/wisevision_dashboard/app/client/.env_example src/wisevision_dashboard/app/client/.env
```
`.env` File has address of the backend server. To use dashboard outside localhost change it to your server ip. File `.env` has default local address.

## Run

``` bash
docker-compose up
```