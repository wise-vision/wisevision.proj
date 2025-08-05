# WiseVision.proj

[![License](https://img.shields.io/badge/license-MPL--2.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20|%20Jazzy-blue.svg)](https://docs.ros.org/)
[![LoRaWAN](https://img.shields.io/badge/LoRaWAN-8000%2B%20devices-green.svg)](https://lora-alliance.org/)

<p align="center">
  <a href="https://snapcraft.io/wisevision-iot">
    <img src="https://snapcraft.io/en/dark/install.svg" alt="Get it from the Snap Store">
  </a>
</p>

> **Where ROS2 meets LoRaWAN** - Connect 8000+ LoRaWAN devices to the ROS2 ecosystem

![ros2_lorawan](docs/assets//readme_ros2_lorawan.png)

## 🚀 Quick Start

Choose your preferred setup method:

| Method | Best For | Time | Complexity |
|--------|----------|------|------------|
| [📦 Snap Package](QUICK_START.md#-5-minute-setup-snap) | Production deployment | 5 min | ⭐ |
| [🐳 Docker Compose](QUICK_START.md#-10-minute-setup-docker) | Development & testing | 10 min | ⭐⭐ |
| [🔧 Local Build](QUICK_START.md#-30-minute-setup-local-build) | Development & customization | 30 min | ⭐⭐⭐ |

**New to WiseVision?** → Start with the [📖 Quick Start Guide](QUICK_START.md)

**Need help with setup?** → Check [📋 Prerequisites](PREREQUISITES.md)

## 📋 Prerequisites

Before starting, ensure your system meets these requirements:

### System Requirements
- **OS**: Ubuntu 20.04+ or Debian 11+
- **Memory**: Minimum 8GB RAM (16GB recommended)
- **Disk**: Minimum 20GB free space
- **Network**: Internet connection for package downloads

### Required Tools
```bash
# Install basic tools
sudo apt update
sudo apt install -y git curl wget build-essential

# Install VCS tool for repository management
pip3 install vcstool
```

### Architecture Support
- **Supported**: x86_64, ARM64
- **Tested on**: Ubuntu 22.04 LTS, Ubuntu 24.04 LTS

## Table of Contents

- [WiseVision.proj](#wisevisionproj)
  - [🚀 Quick Start](#-quick-start)
  - [📋 Prerequisites](#-prerequisites)
    - [System Requirements](#system-requirements)
    - [Required Tools](#required-tools)
    - [Architecture Support](#architecture-support)
  - [Table of Contents](#table-of-contents)
  - [📦 Snap Installation](#-snap-installation)
  - [🐳 Docker Installation](#-docker-installation)
  - [🔧 Local Installation](#-local-installation)
    - [Step 1: Download Project](#step-1-download-project)
    - [Step 2: Install Dependencies](#step-2-install-dependencies)
      - [Option A: Automated Installation (Recommended)](#option-a-automated-installation-recommended)
      - [Option B: Manual Installation](#option-b-manual-installation)
        - [ROS2](#ros2)
        - [MQTT C++ Client Library](#mqtt-c-client-library)
        - [gRPC](#grpc)
    - [Step 3: Build Project](#step-3-build-project)
    - [Step 4: Verification](#step-4-verification)
  - [⚙️ Configuration](#️-configuration)
    - [Required Configuration Files](#required-configuration-files)
    - [External Dependencies](#external-dependencies)
  - [🔍 Verification \& Testing](#-verification--testing)
    - [Basic Functionality Test](#basic-functionality-test)
    - [Integration Test](#integration-test)
    - [Health Check](#health-check)
  - [📚 Advanced Setup](#-advanced-setup)
    - [Development Options](#development-options)
    - [Performance Tuning](#performance-tuning)
    - [CI/CD Integration](#cicd-integration)
  - [❓ FAQ \& Troubleshooting](#-faq--troubleshooting)
    - [Common Issues](#common-issues)
    - [Getting Help](#getting-help)
  - [🤝 Contributing](#-contributing)
    - [Development Workflow](#development-workflow)
  - [📄 License](#-license)
  - [🙏 Acknowledgments](#-acknowledgments)

**📖 Additional Guides**:
- [🚀 Quick Start Guide](QUICK_START.md) - Get running in 5-30 minutes
- [📋 Prerequisites Guide](PREREQUISITES.md) - System requirements & preparation
- [🔧 Troubleshooting Guide](TROUBLESHOOTING.md) - Common issues & solutions
- [Setup Local](setup_local.md) - Detailed local installation
- [Setup with Docker Compose](setup_with_docker_compose.md) - Docker development setup
- [Setup with Docker](setup_with_docker.md) - Advanced Docker configuration
- [Setup with Snap](setup_with_snap.md) - Snap package deployment

## 📦 Snap Installation

For the fastest setup, use the pre-built snap package:

```bash
# Install the snap
sudo snap install wisevision-iot

# Follow the configuration guide
```

📖 **Detailed guide**: [Setup with Snap](setup_with_snap.md)

---

## 🐳 Docker Installation

Perfect for development and testing environments:

```bash
# Clone and setup
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj
vcs import --recursive < project.repos

# Configure and run
docker-compose up --build
```

📖 **Detailed guides**: 
- [Setup with Docker Compose](setup_with_docker_compose.md)
- [Setup with Docker](setup_with_docker.md)

---

## 🔧 Local Installation

For development and customization. Follow these steps in order:

### Step 1: Download Project

```bash
# Clone the main repository
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj

# Import all sub-repositories
vcs import --recursive < project.repos
```

**✅ Verification**: Check that `src/` directory exists with multiple sub-projects:
```bash
ls src/  # Should show: wisevision_action_executor, wisevision_dashboard, etc.
```

### Step 2: Install Dependencies

Choose your approach:

#### Option A: Automated Installation (Recommended)

```bash
# Run the automated dependency installer
./install_depends.sh
```

#### Option B: Manual Installation

If you prefer manual control or the automated script fails:

##### ROS2

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

**✅ Verification**: Check ROS2 installation:
```bash
ros2 --version  # Should show: ros2 cli version
```

##### MQTT C++ Client Library

MQTT client library is used to connect with Chirpstack API in LoRaWAN bridge.

```bash
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
```

**✅ Verification**: Check MQTT libraries:
```bash
pkg-config --exists paho-mqtt && echo "MQTT libraries installed" || echo "MQTT libraries missing"
```

##### gRPC

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

**✅ Verification**: Check gRPC installation:
```bash
${GRPC_INSTALL_DIR}/bin/protoc --version  # Should show: libprotoc version
```

### Step 3: Build Project

```bash
cd wisevision.proj # or the directory where the project.repos file is located
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**✅ Verification**: Check build success:
```bash
source install/setup.bash
ros2 pkg list | grep wisevision  # Should show multiple wisevision packages
```

### Step 4: Verification

Run a quick test to ensure everything works:

```bash
# Source the environment
source install/setup.bash

# Test ROS2 nodes are available
ros2 node list  # Should run without errors

# Check if wisevision packages are available
ros2 pkg list | grep wisevision
```

📖 **Detailed guide**: [Setup Local](setup_local.md)

---

## ⚙️ Configuration

After installation, you need to configure the system:

### Required Configuration Files

1. **Zenoh Router Configuration**
   ```bash
   cp example_zenoh_router.json5 zenoh_router.json5
   # Edit zenoh_router.json5 with your settings
   ```

2. **ROS2 Parameters**
   ```bash
   cp config/params.yaml my_params.yaml
   # Edit my_params.yaml with your specific settings
   ```

3. **Environment Variables**
   ```bash
   # For local setup
   export CHIRPSTACK_API_KEY=<your_api_key>
   export APPLICATION_ID=<your_application_id>
   ```

### External Dependencies

- **ChirpStack**: LoRaWAN Network Server
- **InfluxDB**: Time-series database
- **Zenoh**: Communication middleware

📖 **Configuration guides**:
- [ChirpStack Setup](docs/set_up_chirpstack.md)
- [Gateway Setup](docs/set_up_gateway.md)
- [Zenoh Installation](docs/install_zenoh.md)

---

## 🔍 Verification & Testing

Verify your installation works correctly:

### Basic Functionality Test

```bash
# 1. Start the core services
source install/setup.bash

# 2. Run a basic component test
ros2 run wisevision_data_black_box black_box

# 3. Check system status
ros2 topic list  # Should show wisevision topics
```

### Integration Test

```bash
# Test the complete pipeline (requires ChirpStack running)
ros2 launch launch.py
```

### Health Check

```bash
# Check all services are running
systemctl status zenohd
systemctl status influxdb
```

---

## 📚 Advanced Setup

### Development Options

- **Custom builds**: Modify CMake flags for specific needs
- **Debug builds**: Use `-DCMAKE_BUILD_TYPE=Debug`
- **Custom ROS2 distributions**: Adapt for other ROS2 versions

### Performance Tuning

- **Memory optimization**: [InfluxDB memory limits](memory_limitation_influxdb.md)
- **Network optimization**: Configure Zenoh endpoints
- **Multi-machine setup**: [DDS Router configuration](dds_router.md)

### CI/CD Integration

```bash
# Run automated tests
act pull_request -W .github/workflows/ros2_ci.yml -j build -P ubuntu-22.04=catthehacker/ubuntu:act-22.04 --secret SSH_KEY="$(cat path/to/your/private_key)"
```

---

## ❓ FAQ & Troubleshooting

### Common Issues

**Q: "src directory not found"**
A: Run `vcs import --recursive < project.repos` first

**Q: "Permission denied for docker hub"**
A: Build the image locally:

``` bash
cd src/wisevision_msgs
docker build -t wisevision/ros_with_wisevision_msgs:humble -f Dockerfile .
cd ../..
docker-compose up --build
```

**Q: "gRPC build fails"**
A: Ensure you have sufficient memory (8GB+) and disk space (20GB+)

**Q: "ROS2 packages not found"**
A: Source the environment: `source install/setup.bash`

### Getting Help

- 📖 **Quick Start**: [5-minute setup guide](QUICK_START.md)
- 📋 **Prerequisites**: [System requirements](PREREQUISITES.md)
- 🔧 **Troubleshooting**: [Common issues & solutions](TROUBLESHOOTING.md)
- 🐛 **Bug Reports**: [GitHub Issues](https://github.com/wise-vision/wisevision.proj/issues)
- 💬 **Questions**: [GitHub Discussions](https://github.com/wise-vision/wisevision.proj/discussions)

---

## 🤝 Contributing

We welcome contributions! Here's how to get started:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b your-username/amazing-feature`
3. **Test your changes**: Ensure all tests pass
4. **Submit a pull request**: Include detailed description

### Development Workflow

```bash
# Setup development environment
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj
vcs import --recursive < project.repos

# Make changes and test
colcon build --symlink-install
colcon test

# Submit changes
git add .
git commit -m "feat: add amazing feature"
git push origin your-username/amazing-feature
```

---

## 📄 License

This project is licensed under the Mozilla Public License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **ROS2 Community** for the excellent robotics framework
- **LoRa Alliance** for the LoRaWAN specification
- **Eclipse Zenoh** for the communication middleware
- **All contributors** who make this project possible

---

<div align="center">

**⭐ Star us on GitHub if this project helped you! ⭐**

[🏠 Homepage](https://github.com/wise-vision/wisevision.proj) • 
[📖 Documentation](docs/) • 
[🚀 Quick Start](#-quick-start) • 
[🤝 Contributing](#-contributing)

</div>
