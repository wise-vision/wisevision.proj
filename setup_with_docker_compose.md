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
vcs import  < blac_box.repos
```
Create configs files in wisevision.proj directory:
### `zenoh.json5` fill with zenoh server configs
``` bash
touch zenoh.json5
```
Fill with:
``` json5
{
  plugins: {
    storage_manager: {
      volumes: {
        influxdb: {
          url: "http://localhost:8086",
          private: {
          }
        }
      },
      storages: {
        demo: {
          // the key expression this storage will subscribes to
          key_expr: "sensor_publisher/**",
          // this prefix will be stripped from the received key when converting to database key.
          // i.e.: "demo/example/a/b" will be stored as "a/b"
          // this option is optional
          // strip_prefix: "rt",
          volume: {
            id: "influxdb",
            db: "zenoh_example",
            create_db: true,
            on_closure: "do_nothing",
            private: {
              // InfluxDB credentials for read-write on the bucket
              // username: "cezary",
              // password: "NoweHaslo123!"
            }
          }
        },
        demo_test: {
          // the key expression this storage will subscribes to
          key_expr: "eui_<device_eui>/uplink/custom/**",
          // this prefix will be stripped from the received key when converting to database key.
          // i.e.: "demo/example/a/b" will be stored as "a/b"
          // this option is optional
          // strip_prefix: "rt",
          volume: {
            id: "influxdb",
            db: "zenoh_example",
            create_db: true,
            on_closure: "do_nothing",
            private: {
            }
          }
        }
      }
    },
   rest: { http_port: '8000' }
  },
}
```
### `config.json` for black_box

``` bash
touch config.json
```
fill with:
``` json
{
    "zenoh_url": "<your_url>"
  }
```

## Run

``` bash
docker-compose up
```