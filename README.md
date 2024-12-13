# wisevision.proj
Repository containing sub-repos for setting up the whole project


## Download 

### VCSTool

Install the [vcs](https://github.com/dirk-thomas/vcstool) tool:

```bash
pip3 install vcstool
```

### Get the project
```bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Source 

```bash
source install/setup.bash
```

## Docker Run

Other way to run the project is to use docker-compose. This step requires `docker`, `docker-compose` installed and also the GitHub token to be set in the environment

### Setup

It is required to set up 

```bash
vcs import --recursive < project.repos
cp src/ros2_lora_bridge/.env_example src/ros2_lora_bridge/.env  
```

### Run with docker-compose

```
$ GITHUB_TOKEN=<YOUR-GH-TOKEN> docker-compose up --build  
```

## FAQ

### Permission denied for docker hub

In case of permission denied for docker hub -> create the required image locally:

``` bash
cd src/wisevision_msgs
docker build -t wisevision/ros_with_wisevision_msgs:humble -f Dockerfile .
cd ../..
docker-compose up --build
```

## Run workflow with act

Install `act` tool [link](https://github.com/nektos/act).

```bash
act pull_request -W .github/workflows/ros2_ci.yml -j build -P ubuntu-22.04=catthehacker/ubuntu:act-22.04 --secret SSH_KEY="$(cat path/to/your/private_key)"
```
