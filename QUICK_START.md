# 🚀 WiseVision Quick Start Guide

Get WiseVision up and running in under 10 minutes!

## 📋 Prerequisites Check

Before starting, verify your system:

```bash
# Check OS version
lsb_release -a

# Check available memory (need 8GB+)
free -h

# Check disk space (need 20GB+)
df -h

# Check if basic tools are installed
which git curl python3 pip3
```

## ⚡ 5-Minute Setup (Snap)

The fastest way to get started:

```bash
# 1. Install the snap
sudo snap install wisevision-iot

# 2. Set up Zenoh config
mkdir -p ~/wisevision_workspace
cd ~/wisevision_workspace
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj
cp example_zenoh_router.json5 zenoh_router.json5

# 3. Create ROS parameters file
cp config/params.yaml ros_params.yaml

# 4. Configure snap (replace with your values)
sudo snap set wisevision-iot \
  config-file=$PWD/zenoh_router.json5 \
  ros-config-file=$PWD/ros_params.yaml \
  email-username-notification=your-email@example.com \
  email-password-notification=your-password \
  email-recipients-notification=admin@example.com \
  chirpstack-api-token=your-api-token \
  backend-api-url=http://localhost:5000

# 5. Start the service
sudo snap start wisevision-iot.wisevision-proj
```

✅ **Verification**: Visit `http://localhost:3000` to see the dashboard

📖 **Detailed guide**: [Setup with Snap](setup_with_snap.md)

---

## 🐳 10-Minute Setup (Docker)

For development and testing:

```bash
# 1. Clone and setup repositories
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj
pip3 install vcstool
vcs import --recursive < project.repos

# 2. Configure Zenoh
cp example_zenoh_router.json5 zenoh_router.json5

# 3. Configure ROS parameters
cp config/params.yaml ros_params.yaml
# Edit ros_params.yaml with your settings

# 4. Set up environment variables
cat > .env << EOF
ROS_PARAM_FILE=/root/wisevision_proj_ws/src/ros_params.yaml
USE_EMAIL_NOTIFIER=true
USE_FIREBASE_NOTIFIER=false
EMAIL_USERNAME_NOTIFICATION=your-email@example.com
EMAIL_PASSWORD_NOTIFICATION=your-password
EMAIL_RECIPIENTS_NOTIFICATION=admin@example.com
CHIRPSTACK_API_KEY=your-api-key
EOF

# 5. Start all services
docker-compose up --build
```

✅ **Verification**: 
- Dashboard: `http://localhost:3000`
- Backend API: `http://localhost:5000`
- InfluxDB: `http://localhost:8086`

📖 **Detailed guide**: [Setup with Docker Compose](setup_with_docker_compose.md)

---

## 🔧 30-Minute Setup (Local Build)

For development and customization:

```bash
# 1. Install dependencies
sudo apt update
sudo apt install -y git curl build-essential cmake
pip3 install vcstool

# 2. Clone and setup
git clone https://github.com/wise-vision/wisevision.proj.git
cd wisevision.proj
vcs import --recursive < project.repos

# 3. Install dependencies (automated)
./install_depends.sh

# 4. Build the project
export GRPC_INSTALL_DIR=$HOME/grpc_install_dir
export PATH=$PATH:${GRPC_INSTALL_DIR}/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${GRPC_INSTALL_DIR}/lib
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Configure and test
source install/setup.bash
cp example_zenoh_router.json5 config.json
export CHIRPSTACK_API_KEY=your-api-key
export APPLICATION_ID=your-application-id
```

✅ **Verification**:
```bash
# Check ROS2 packages
ros2 pkg list | grep wisevision

# Test a component
ros2 run wisevision_data_black_box black_box
```

📖 **Detailed guide**: [Setup Local](setup_local.md)

---

## 🔍 Verification Steps

After any setup method, verify your installation:

### 1. Check Services Status

```bash
# For snap installation
sudo snap services wisevision-iot

# For docker installation
docker-compose ps

# For local installation
ros2 node list
```

### 2. Test Basic Functionality

```bash
# Check ROS2 topics
ros2 topic list

# Check Zenoh connectivity (if configured)
curl http://localhost:8000

# Check InfluxDB (if running)
curl http://localhost:8086/ping
```

### 3. Access Web Interfaces

- **Dashboard**: http://localhost:3000
- **Backend API**: http://localhost:5000  
- **InfluxDB**: http://localhost:8086
- **ChirpStack**: http://localhost:8080 (if configured)

---

## 🆘 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| "src directory not found" | Run `vcs import --recursive < project.repos` |
| "Permission denied" | Add user to docker group: `sudo usermod -aG docker $USER` and restart |
| "Port already in use" | Stop conflicting services or change ports |
| "Out of memory" | Ensure 8GB+ RAM available |
| "gRPC build fails" | Check disk space (need 20GB+) |

### Get More Help

- 📖 **Full Documentation**: [README.md](README.md)
- 🐛 **Report Issues**: [GitHub Issues](https://github.com/wise-vision/wisevision.proj/issues)
- 💬 **Ask Questions**: [GitHub Discussions](https://github.com/wise-vision/wisevision.proj/discussions)

---

## 🎯 Next Steps

Once you have WiseVision running:

1. **Configure LoRaWAN**: [Set up ChirpStack](docs/set_up_chirpstack.md)
2. **Add Devices**: [Gateway Setup](docs/set_up_gateway.md)
3. **Customize**: Explore the [detailed setup guides](README.md#-configuration)
4. **Develop**: Read the [contributing guide](README.md#-contributing)

---

<div align="center">

**🎉 Congratulations! You have WiseVision running! 🎉**

[🏠 Back to Main README](README.md) • [📖 Full Documentation](docs/) • [🤝 Contributing](README.md#-contributing)

</div>
