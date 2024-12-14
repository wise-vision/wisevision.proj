# wisevision.proj
Repository containing sub-repos for setting up the whole project


## Download 

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

Easiest way to run the project is to use docker-compose.

```bash
docker-compose up --build
```

Permission denied for docker hub -> create image locally:
``` bash
cd src/wisevision_msgs
docker build -t wisevision/ros_with_wisevision_msgs:humble -f Dockerfile .
cd ../..
docker-compose up --build
```

## Run workflow with act

Act tool [link](https://github.com/nektos/act).

```bash
act pull_request -W .github/workflows/ros2_ci.yml -j build -P ubuntu-22.04=catthehacker/ubuntu:act-22.04 --secret SSH_KEY="$(cat path/to/your/private_key)"
```
