# Memory limitation in influxdb
## Docker compose edit
Before run `docker compose` edit file `docker-compose.yml`:
- Add to `influxdb` service, at the end:
``` docker
storage_opt:
    size: 2G
```
- If docker-compoes fails with error message `docker: Error response from daemon: --storage-opt is supported only for overlay over xfs with 'pquota' mount option.` Create new disk partition with correct file system:
1. Create a new logical volume of 20 GB:
``` bash
sudo lvcreate -L 20G -n docker-lv ubuntu-vg
```
2. Formatting the new volume as XFS with f type=1
``` bash
sudo mkfs.xfs -n ftype=1 /dev/ubuntu-vg/docker-lv
```
3. Creating a mount point:
``` bash
sudo mv /var/lib/docker /var/lib/docker_backup
sudo mkdir /var/lib/docker
```
4. Add this line into this file by open this:
``` bash
sudo nano /etc/fstab
```
and paste `/dev/ubuntu-vg/docker-lv /var/lib/docker xfs defaults,pquota 0 0`
Exit by `control + x`
5. Mounting a new volume:
``` bash
sudo mount -a
```
6. Restart docker
``` bash
sudo systemctl restart docker
```