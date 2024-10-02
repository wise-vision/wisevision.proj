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
