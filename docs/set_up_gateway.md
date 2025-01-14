# Setting up the LoRaWAN gateway
## Setup steps

> [!NOTE]
> The instruction cover [LoRaWAN gateway RAK7371](https://docs.rakwireless.com/Product-Categories/WisGate/RAK7271-7371/Quickstart/#using-a-linux-machine-as-a-host)
for the other gateways should be similar but it may require additional steps.

1. Install the needed make and gcc tools with the commands:
   ```bash
   sudo apt install make gcc
   ```
2. Download the archive from the Semtech Repository:
   ```bash
   wget https://github.com/Lora-net/sx1302_hal/archive/V2.1.0.tar.gz
   ```
3. After the download is complete, extract the files with the command:
   ```bash
   tar -xf V2.1.0.tar.gz
   ```
4. Rename extracted directory.
   ```bash
   mv sx1302_hal-2.1.0 sx1302_hal
   ```
5. Enter the created folder:
   ```bash
   cd sx1302_hal
   ```
6. Do a make with the command:
   ```bash
   make
   ```
7. Open the Packet forwarder folder with the following code:
   ```bash
   cd packet_forwarder
   ```
8. Then list the files and folders:
   ```bash
   ls -l
   ```
9. You can see that there is a different example configuration files for different LoRaWAN bands and different types of concentrators.
In this setup, you are using RAK Developer Base, which is the USB, and the EU868 band. Run this command to rename the correct file to `global_conf.json`:
   ```bash
   cp global_conf.json.sx1250.EU868.USB global_conf.json
   ```

### Get Gateway ID

Concentrator EUI can be used as gateway `ID`. Firstly, run packet forwarder for the first time:
```bash
sudo ./lora_pkt_fwd
```
If everything started correctly, on the bottom of the screen look for the line:
```bash
INFO: concentrator EUI: 0x<Gateway EUI>
```
Highlight the <Gateway EUI> value and copy it and close packet forwarder with `Ctrl + c`.
> [!NOTE]
> Make sure to copy the `Gateway EUI` after the **`0x`**.

10. Open `global_conf.json` file:
    ```bash
    nano global_conf.json
    ```
    and change following line:
    ```
    "gateway_ID": "<default ID>",
    ```
    into:
    ```
    "gateway_ID": "<Gateway EUI>",
    ```
    To save changes press `Ctrl+x` then `y` and `enter`.

11. Start packet forwarder by:
    ```bash
    sudo ./lora_pkt_fwd
    ```
