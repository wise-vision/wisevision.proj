# 📋 WiseVision Prerequisites Guide

This guide helps you prepare your system before installing WiseVision.

## 🖥️ System Requirements

### Minimum Requirements
- **Operating System**: Ubuntu 22.04 LTS or newer, Debian 11+
- **Architecture**: x86_64 (AMD64) or ARM64
- **Memory**: 8 GB RAM
- **Storage**: 20 GB free disk space
- **Network**: Stable internet connection for downloads

### Recommended Requirements
- **Operating System**: Ubuntu 22.04 LTS or Ubuntu 24.04 LTS
- **Architecture**: x86_64 (AMD64)
- **Memory**: 16 GB RAM or more
- **Storage**: 50 GB free disk space (SSD preferred)
- **Network**: High-speed internet connection

### Tested Platforms
✅ **Fully Tested & Supported**:
- Ubuntu 22.04 LTS (x86_64)
- Ubuntu 24.04 LTS (x86_64)


❌ **Not Supported**:
- Windows (use WSL2 with Ubuntu)
- macOS (use Docker)
- 32-bit systems

## 🔧 Required Software

### Core Tools

```bash
# Update package list
sudo apt update

# Install essential tools
sudo apt install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    pkg-config
```

### Python Environment

```bash
# Install Python tools
pip3 install --user \
    vcstool \
    colcon-common-extensions

# Verify installation
vcs --version
colcon --help
```

### Version Requirements

| Tool | Minimum Version | Recommended | Check Command |
|------|-----------------|-------------|---------------|
| Git | 2.17+ | 2.34+ | `git --version` |
| Python | 3.8+ | 3.10+ | `python3 --version` |
| CMake | 3.13+ | 3.22+ | `cmake --version` |
| GCC | 9+ | 11+ | `gcc --version` |

## 🐳 Installation Method Prerequisites

### For Snap Installation

```bash
# Install snapd (if not already installed)
sudo apt install snapd

# Verify snap is working
snap version
```

### For Docker Installation

```bash
# Install Docker
sudo apt install -y docker.io docker-compose

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and back in, then verify
docker --version
docker-compose --version
```

### For Local Build Installation

```bash
# Install ROS2 dependencies
sudo apt install -y \
    software-properties-common \
    lsb-release \
    gnupg2

# Install build tools
sudo apt install -y \
    autoconf \
    libtool \
    pkg-config \
    libssl-dev
```

## 🌐 Network Requirements

### Firewall Configuration

If you have a firewall enabled, ensure these ports are accessible:

| Port | Service | Required For |
|------|---------|--------------|
| 3000 | Dashboard Frontend | Web interface |
| 5000 | Backend API | API access |
| 7447 | Zenoh Router | ROS2 communication |
| 8000 | Zenoh REST | REST API |
| 8086 | InfluxDB | Database access |
| 8080 | ChirpStack | LoRaWAN management |

```bash
# Example UFW configuration
sudo ufw allow 3000
sudo ufw allow 5000
sudo ufw allow 7447
sudo ufw allow 8000
sudo ufw allow 8086
sudo ufw allow 8080
```

### Internet Access

WiseVision requires internet access for:
- Package downloads during installation
- Docker image pulling
- GitHub repository cloning
- ROS2 package dependencies

## 🔐 Permissions & Security

### User Permissions

```bash
# Add user to necessary groups
sudo usermod -aG docker $USER      # For Docker access
sudo usermod -aG dialout $USER     # For serial port access (gateways)

# Apply group changes (logout/login required)
newgrp docker
```

### Directory Permissions

```bash
# Create workspace directory
mkdir -p ~/wisevision_workspace
cd ~/wisevision_workspace

# Ensure proper ownership
sudo chown -R $USER:$USER ~/wisevision_workspace
```

### Snap Confinement

For snap installation, WiseVision requires:
- Network access
- Home directory access
- System service control

These are automatically granted during installation.

## 🎯 Next Steps

Once your system meets all prerequisites:

1. **Choose Installation Method**: 
   - 📦 [Snap](QUICK_START.md#-5-minute-setup-snap) - Fastest
   - 🐳 [Docker](QUICK_START.md#-10-minute-setup-docker) - Development
   - 🔧 [Local](QUICK_START.md#-30-minute-setup-local-build) - Customization

2. **Follow Quick Start**: [QUICK_START.md](QUICK_START.md)

3. **Read Detailed Guides**: [README.md](README.md)

---

## 📞 Getting Help

If you encounter issues with prerequisites:

- 🐛 **Questions/Issues**: [GitHub Issues](https://github.com/wise-vision/wisevision.proj/issues)
- 📖 **Documentation**: [Main README](README.md)

---

<div align="center">

**✅ Prerequisites Complete? Start with the [Quick Start Guide](QUICK_START.md)! ✅**

</div>
