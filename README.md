# wisevision.proj
Repository containing sub-repos for setting up the whole project


## Download 

```bash
git clone git@github.com:wise-vision/wisevision.proj.git
vcs import --recursive < projerct.repos
```

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
