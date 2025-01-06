# wisevision.proj
Repository containing sub-repos for setting up the whole project


## Download 

### VCSTool

The project uses the `vcs` tool to configure the dependent repositories.

Install the [vcstool](https://github.com/dirk-thomas/vcstool):

```bash
pip3 install vcstool
```

### Get the project
```bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj # git checkout 2411 # (or any other branch/tag) 
vcs import --recursive < project.repos
```


### Install dependencies

In case you have done the installation of the dependencies before, you can skip this step to [Run without docker](#run-without-docker)

#### ROS2

So far the **ROS 2** `Humble` and `Jazzy` distributions are supported. The original instruction can be found here: [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) below instruction is for `Humble` distribution.

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools python3-rosdep
sudo rosdep init
rosdep update

source /opt/ros/humble/setup.bash # for convenience echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### LoRaWAN bridge

Follow steps from [README](https://github.com/wise-vision/wisevision_lorawan_bridge/blob/dev/README.md) 
file in `wisevision_lorawan_bridge` repository.

## Build

```bash
cd wisevision.proj # or the directory where the project.repos file is located
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Run without docker

```bash
source install/setup.bash
```

TODO: Add the one liner to run the project

## 

## Docker Run

Other way to run the project is to use docker-compose. This step requires `docker`, `docker-compose` installed and also the GitHub token to be set in the environment

### Setup

It is required to set up 

```bash
vcs import --recursive < project.repos
cp src/ros2_lora_bridge/.env_example src/ros2_lora_bridge/.env  
```

### Run with docker-compose

Will be removed in the future but for now, it is required to build the image locally to get the [GitHub token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens).

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
