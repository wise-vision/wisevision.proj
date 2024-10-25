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
## Source 

```bash
source install/setup.bash
```
## Source 

```bash
source install/setup.bash
```
## Source 

```bash
source install/setup.bash
```

## Docker Run

Easiest way to run the project is to use docker-compose with the GitHub token.

```bash
GITHUB_TOKEN=ghp_YOUR_GH_TOKEN docker-compose up --build
```

Without the GitHub token: TODO
