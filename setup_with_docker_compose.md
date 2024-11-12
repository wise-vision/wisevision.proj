# How to run docker compose?
## Before run
Clone repos:
``` bash
git clone git@github.com:wise-vision/wisevision.proj.git && cd wisevision.proj
vcs import --recursive < project.repos
```
import repos dependecies:
```
cd src/ros2_automatic_action_execution
vcs import  < automatic_action_execution.repos
cd ../ros2_black_box
vcs import  < black_box.repos
```

## Configurations

### Config Zenoh 

The zenoh dds which is currently the only one supported requires a configuration file to be presentfile should be named `zenoh.json`. The example file is provided as `example_zenoh.json5` by default can be copied into.

``` bash
cp example_zenoh.json5 zenoh.json
```

### Config wisevision_data_black_box

The black box requires a configuration file and to be named `config.json`. 

The only required field is `zenoh_url` which should be the url of the zenoh server.  The example file is provided as `example_config.json5` by default can be copied into.

``` bash
cp example_config.json config.json
``` 


## Run

``` bash
docker-compose up
```
