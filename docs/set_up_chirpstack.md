# How to Set Up ChirpStack

Chirpstack is an open-source LoRaWAN Network Server stack. It is responsible for handling the communication between the LoRaWAN end-devices and the application server. In the WiseVision project, it is used to manage the communication between the LoRaWAN end-devices and the WiseVision bridge.

## Run Chirpstack in Docker

### Clone the repository
1. Clone the repository:
```bash
vcs import --recursive < project.repos
```

### Run `Chirpstack`
1. Run:
``` bash
cd src/wisevision_lorawan_bridge/chirpstack_docker
docker compose up 
# or for older versions docker-compose up 
```

## How to Add Gateway
1. Start the gateway in another terminal (if it's already set up, if not [setup gateway](set_up_gateway.md)):
``` bash
cd sx1302_hal/packet_forwarder
sudo ./lora_pkt_fwd
```
2. Open http://localhost:8080. Log in with admin/admin.
3. On the left bar click on `Gateways`. (`1.` on the image below)
4. Click on the `Add gateway`. (`2.` on the image below)

<img src="assets/add_gateway_start.png" alt="Add gateway start" width="900" />

5. Write name for gateway. (`1`. on the image below)
6. Write gateway ID (EUI) [How to get gateway ID](set_up_gateway.md#get-gateway-id) (`2.` on the image below)
7. Save gateway by click `Submit` (`3.` on the image below)

<img src="assets/add_gateway_data.png" alt="Add gateway data" width="900" />

## How to Create Application
1. On the left bar click on `Aplications`. (`1.` on the image below)
2. Click on the `Add application`. (`2.` on the image below)

<img src="assets/add_application_start.png" alt="Add application start" width="900" />

3. Write name for application. (`1`. on the image below)
4. Save application by click `Submit` (`2`. on the image below)

<img src="assets/add_application_data.png" alt="Add applcation data" width="900" />

## How to Create Device Profile
1. On the left bar click on `Device Profiles`. (`1.` on the image below)
2. Click on the `Add device profile`. (`2.` on the image below)

<img src="assets/add_device_profile_start.png" alt="Add device profile start" width="900" />

3. Write name for device profile (this name has to be the same name of the parser class in `wisevision_lorawan_bridge`
for this device, this name is case sensitive) (`1.` on the image below)
4. If device is class C device click on the `Class-C` (`2.` on the image below)

<img src="assets/add_device_profile_data.png" alt="Add device profile data" width="900" />

5. Switch `Device supports Class-C` (`1.` on the image below)
6. Save device profile by click `Submit` (`2.` on the image below)

<img src="assets/add_device_profile_class_c.png" alt="Add device profile class C" width="900" />

## How to Add Device to Application
1. On the left bar click on `Applications`. (`1.` on the image below)
2. Choose the application to add the device to. (`2.` on the image below)

<img src="assets/add_device_start.png" alt="Add device start" width="900" />

3. Click on the `Add device`. (`1.` on the image below)

<img src="assets/add_device_application.png" alt="Add device application" width="900" />

4. Write name for device. (`1.` on the image below)
5. Write device `EUI`. (`2.` on the image below)
6. Write device `Join EUI`. (`3.` on the image below)
7. Choose correct device profile. (`4.` on the image below)
8. Save device by click `Submit` (`5.` on the image below)

<img src="assets/add_device_data.png" alt="Add device data" width="900" />

## How to Get Application ID

1. On the left bar click on `Applications`. (`1.` on the image below)
2. Choose the application to get the `application ID` from. (`2.` on the image below)

<img src="assets/application_id_start.png" alt="Application id start" width="900" />

3. Application id is next to app name. (`1.` on the image below)

<img src="assets/application_id_data.png" alt="Application id data" width="900" />

## How to Create API Key

1. On the left bar click on `API keys`. (`1.` on the image below)
2. Click on the `Add API key`. (`2.` on the image below)

<img src="assets/add_api_key_start.png" alt="API key start" width="900" />

3. Wrtie name of the `API keys`. (`1.` on the image below)
4. Click on the `Submit`. (`2.` on the image below)

<img src="assets/add_api_key_data.png" alt="API key data" width="900" />

5. `API key` is here (`1.` on the image below)
6. Click on the `Submit`. (`2.` on the image below)
7. Store the value in environment variable `CHIRPSTACK_API_TOKEN`.

<img src="assets/add_api_key_copy.png" alt="API key copy" width="900" />

> [!NOTE]
> It will not be possible to retrieve API key later. If you lose the key value,
you will need to generate a new key.
