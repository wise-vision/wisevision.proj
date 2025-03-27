
# wisevision.proj snap

This snap packages wivevision.proj with zenoh and inlfux.

## Instalation

Install the snap as follows:

```bash
snap install wisevisionproj
```

## Prequistances


- [Run chirpstack](https://www.chirpstack.io/docs/getting-started/docker.html)

- Create zenoh router file:
  ```bash
  mkdir wisevision_proj_ws
  cd wisevision_proj_ws
  git clone https://github.com/wise-vision/wisevision.proj.git
  cp example_zenoh_router.json5 zenoh_router.json5
  ```
**ROS2 parameters**

To set paramters create paramter file, the example of this file is here:
```yaml
notification_handler:
ros__parameters:
  use_email_notifier: true # default is true
  use_firebase_notifier: false # default is false
  smtp_server: "<smtp_server>"
  service_account_path: "<service_account_path>"
lorawan_bridge:
  ros__parameters:
    application_id: "<chirpstack_application_id>"
    use_only_standard: false # default is true
```
**Restrictions:**
- Paths to files have to be absolute,
- Every variable has to be set.

```bash
  sudo snap set wisevisionproj config-file=<path_to_zenoh_config_file>
  sudo snap set wisevisionproj ros-config-file=<path_to_ros_config_parameters_file>
  sudo snap set wisevisionproj email-username-notification=<email_username_to_send_notications>
  sudo snap set wisevisionproj email-password-notification=<email_password_to_send_notications>
  sudo snap set wisevisionproj email-recipients-notification=<email_recipients_to_send_notications>
  sudo snap set wisevisionproj chirpstack-api-key=<chirpstack_api_key>
  sudo snap set wisevisionproj backend-api-url=<backend_api_url>
```


# Run
After set up variables, run wisevision.proj by:
```bash
sudo snap start wisevisionproj.wisevision-proj
```

## Serivces in snap
- influxdb
- zenoh
- backend
- frontend
- wisevision-proj-ros
- wisevision-proj(starts all service above)

## Build snap locally

- [Install snapcraft](https://snapcraft.io/docs/snapcraft-setup)
- Run build:
  ```bash
  SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1 snapcraft
  ```
- Install:
  ```bash
  sudo snap install wisevisionproj_1.0_arm64.snap --devmode --dangerous 
  ```

