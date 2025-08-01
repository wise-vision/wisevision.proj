# wisevision.proj
Repository containing sub-repos for setting up the whole project



## Place where ROS2 meets LoRaWAN

- WiseVision is about connecting the ROS2 ecosystem with other technologies, currently focusing on LoRaWAN. From 8000+ LoRaWAN devices all can be integrated into the ROS2 ecosystem with the WiseVision. 
- Use for free and contribute to the project to make it better.
- Enjoy the power of ROS 2 and LoRaWAN together.

![ros2_lorawan](docs/assets//readme_ros2_lorawan.png)

## Table of Contents

- [wisevision.proj](#wisevisionproj)
  - [Place where ROS2 meets LoRaWAN](#place-where-ros2-meets-lorawan)
  - [Table of Contents](#table-of-contents)
  - [Download](#download)
    - [VCSTool](#vcstool)
    - [Get the project](#get-the-project)
    - [Install dependencies](#install-dependencies)
      - [ROS2](#ros2)
      - [MQTT C++ Client Library](#mqtt-c-client-library)
      - [gRPC](#grpc)
  - [Build](#build)
  - [Docker Run](#docker-run)
    - [Setup](#setup)
    - [Run with docker-compose](#run-with-docker-compose)
  - [FAQ](#faq)
    - [Permission denied for docker hub](#permission-denied-for-docker-hub)
  - [Run workflow with act](#run-workflow-with-act)

**For more details, visit**:
- [Setup Local](setup_local.md)
- [Setup with Docker Compose](setup_with_docker_compose.md)
- [Setup with Docker](setup_with_docker.md)

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

So far the **ROS 2** `Humble` and `Jazzy` distributions are supported. The original instruction can be found here: [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) below instruction is for `Jazzy` distribution.

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-rosdep
sudo rosdep init
rosdep update

source /opt/ros/jazzy/setup.bash # for convenience echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

#### MQTT C++ Client Library

MQTT client library is used to connect with Chirpstack API in LoRaWAN bridge.

```bash
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
```

#### gRPC

gRPC has to be built from source to use C++ plugin which generates C++ source files from `.proto`.

1. Define gRPC installation directory and export variable.
```bash
export GRPC_INSTALL_DIR=$HOME/grpc_install_dir
mkdir $GRPC_INSTALL_DIR
```

2. Make sure that there is CMake version 3.13 or later. Try installing from package.
```bash
sudo apt install -y cmake
```
Check CMake version and if it is too low, install if from source.
```bash
cmake --version
```

3. Install other required packages.
```bash
sudo apt install -y build-essential autoconf libtool pkg-config
```

4. Clone gRPC repository.
```bash
git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc
```

5. Build and install gRPC and Protocol Buffers.
```bash
cd grpc
mkdir -p cmake/build
pushd cmake/build
cmake -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR \
    -DgRPC_BUILD_GRPC_CPP_PLUGIN=ON \
    -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_NODE_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_PHP_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_RUBY_PLUGIN=OFF \
    ../..
make -j$(nproc --ignore=2) # if your machine is stronger, consider increasing number of jobs or skip it altogether to run without constraints
make install
popd
```

6. Export variables to be able to load built shared libraries and include headers. It is recommended to put those variables inside `.bashrc` file.
```bash
export GRPC_INSTALL_DIR=${HOME}/grpc_install_dir
export PATH=$PATH:${GRPC_INSTALL_DIR}/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${GRPC_INSTALL_DIR}/lib
```

## Build

```bash
cd wisevision.proj # or the directory where the project.repos file is located
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**To run WiseVision locally, follow steps**: [Setup Local](setup_local.md)

## Docker Run

Other way to run the project is to use docker-compose. This step requires `docker`, `docker-compose`
installed and also the GitHub token to be set in the environment.

### Setup

It is required to set up.

```bash
vcs import --recursive < project.repos
cp src/wisevision_lorawan_bridge/.env_example src/wisevision_lorawan_bridge/.env  
```

**To run WiseVision docker, follow steps**: [Setup with Docker](setup_with_docker.md)

### Run with docker-compose

Will be removed in the future but for now, it is required to build the image locally to get the [GitHub token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens).

```bash
$ GITHUB_TOKEN=<YOUR-GH-TOKEN> docker-compose up --build  
```

**To run WiseVision docker, follow steps**: [Setup with Docker Compose](setup_with_docker_compose.md)

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

Run:
```bash
act pull_request -W .github/workflows/ros2_ci.yml -j build -P ubuntu-22.04=catthehacker/ubuntu:act-22.04 --secret SSH_KEY="$(cat path/to/your/private_key)"
```
