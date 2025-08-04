# 🔧 WiseVision Troubleshooting Guide

Common issues and their solutions for WiseVision installation and operation.

## 📋 Quick Diagnostic

Before diving into specific issues, run this diagnostic:

```bash
# Check system resources
echo "=== System Status ==="
free -h
df -h /
lsb_release -a

# Check running services
echo "=== Service Status ==="
# For snap
sudo snap services wisevision-iot 2>/dev/null || echo "Snap not installed"

# For docker
docker-compose ps 2>/dev/null || echo "Docker compose not running"

# For local
ros2 node list 2>/dev/null || echo "ROS2 not sourced or running"

# Check network connectivity
echo "=== Network Status ==="
curl -s http://localhost:8000 > /dev/null && echo "Zenoh: ✅" || echo "Zenoh: ❌"
curl -s http://localhost:8086/ping > /dev/null && echo "InfluxDB: ✅" || echo "InfluxDB: ❌"
curl -s http://localhost:3000 > /dev/null && echo "Dashboard: ✅" || echo "Dashboard: ❌"
```

## 🚨 Installation Issues

### "src directory not found"

**Problem**: Commands fail because `src/` directory doesn't exist
```
bash: cd: src/wisevision_dashboard: No such file or directory
```

**Solution**: Run repository import first
```bash
cd wisevision.proj
vcs import --recursive < project.repos
ls src/  # Should show multiple directories
```

**Prevention**: Always run `vcs import` before referencing `src/` files

---

### "Permission denied" for Docker

**Problem**: Docker commands fail with permission errors
```
Got permission denied while trying to connect to the Docker daemon socket
```

**Solution**: Add user to docker group
```bash
sudo usermod -aG docker $USER
newgrp docker  # Apply immediately
# Or logout and login again
```

**Verification**: 
```bash
docker ps  # Should run without sudo
```

---

### "No space left on device"

**Problem**: Build fails due to insufficient disk space
```
No space left on device
```

**Solution**: Free up disk space
```bash
# Check current usage
df -h

# Clean package cache
sudo apt clean
sudo apt autoremove

# Remove old Docker images (if using Docker)
docker system prune -af

# Check again
df -h
```

**Prevention**: Ensure 20GB+ free space before starting

---

### "gRPC build fails"

**Problem**: gRPC compilation errors or hangs
```
make: *** [all] Error 2
```

**Solutions**:
1. **Check memory**: Ensure 8GB+ RAM available
   ```bash
   free -h
   # Close other applications if needed
   ```

2. **Reduce build parallelism**:
   ```bash
   make -j2  # Instead of make -j$(nproc)
   ```

3. **Use automated installer**:
   ```bash
   ./install_depends.sh  # Often more reliable
   ```

---

### "ROS2 packages not found"

**Problem**: colcon build succeeds but packages aren't available
```
Package 'wisevision_data_black_box' not found
```

**Solution**: Source the environment
```bash
source install/setup.bash
ros2 pkg list | grep wisevision  # Should show packages
```

**Add to bashrc** for persistence:
```bash
echo "source ~/wisevision.proj/install/setup.bash" >> ~/.bashrc
```

---

## 🐳 Docker Issues

### "Port already in use"

**Problem**: Docker fails to start due to port conflicts
```
bind: address already in use
```

**Solution**: Stop conflicting services
```bash
# Find what's using the port
sudo netstat -tulpn | grep :8086

# Stop the service (example for InfluxDB)
sudo systemctl stop influxdb

# Or change ports in docker-compose.yml
```

---

### "Image not found"

**Problem**: Docker can't pull custom images
```
pull access denied for wisevision/ros_with_wisevision_msgs
```

**Solution**: Build image locally
```bash
cd src/wisevision_msgs
docker build -t wisevision/ros_with_wisevision_msgs:humble -f Dockerfile .
cd ../..
docker-compose up --build
```

---

### "Container exits immediately"

**Problem**: Containers start then immediately stop

**Diagnosis**: Check logs
```bash
docker-compose logs service_name
docker logs container_name
```

**Common causes**:
- Missing environment variables
- Configuration file errors
- Dependency services not ready

**Solution**: Fix configuration and dependencies
```bash
# Check environment file
cat .env

# Verify configuration files exist
ls zenoh_router.json5 ros_params.yaml

# Start dependencies first
docker-compose up influxdb
# Wait for it to be healthy, then start others
```

---

## 📦 Snap Issues

### "Snap not found"

**Problem**: Can't install WiseVision snap
```
error: snap "wisevision-iot" not found
```

**Solutions**:
1. **Check snap store connectivity**:
   ```bash
   snap find hello-world
   ```

2. **Install from local file**:
   ```bash
   sudo snap install wisevision-iot_1.0_amd64.snap --devmode --dangerous
   ```

---

### "Configuration not applied"

**Problem**: Snap configuration changes don't take effect

**Solution**: Restart snap services
```bash
sudo snap restart wisevision-iot
sudo snap services wisevision-iot  # Check status
```

**Check configuration**:
```bash
snap get wisevision-iot  # View all settings
```

---

## 🌐 Network Issues

### "Can't connect to Zenoh"

**Problem**: ROS2 nodes can't communicate through Zenoh
```
Failed to connect to zenoh router
```

**Diagnosis**:
```bash
# Check if Zenoh is running
curl http://localhost:8000
netstat -tulpn | grep :7447

# Check configuration
cat zenoh_router.json5
```

**Solutions**:
1. **Start Zenoh manually**:
   ```bash
   zenohd -c zenoh_router.json5
   ```

2. **Check firewall**:
   ```bash
   sudo ufw status
   sudo ufw allow 7447
   sudo ufw allow 8000
   ```

3. **Verify configuration**:
   ```bash
   # Ensure valid JSON5
   python3 -c "import json5; json5.load(open('zenoh_router.json5'))"
   ```

---

### "InfluxDB connection failed"

**Problem**: Can't write data to InfluxDB
```
Failed to write to InfluxDB: connection refused
```

**Diagnosis**:
```bash
# Check if InfluxDB is running
curl http://localhost:8086/ping
systemctl status influxdb
```

**Solutions**:
1. **Start InfluxDB**:
   ```bash
   # For system service
   sudo systemctl start influxdb
   sudo systemctl enable influxdb

   # For Docker
   docker run -d -p 8086:8086 influxdb:1.8
   ```

2. **Check configuration**:
   ```bash
   # Verify database exists
   curl -G http://localhost:8086/query --data-urlencode "q=SHOW DATABASES"
   ```

---

## 🔧 Runtime Issues

### "ChirpStack API errors"

**Problem**: LoRaWAN bridge can't connect to ChirpStack
```
Failed to authenticate with ChirpStack API
```

**Solutions**:
1. **Verify API token**:
   ```bash
   echo $CHIRPSTACK_API_KEY
   # Should show your API key
   ```

2. **Test API manually**:
   ```bash
   curl -H "Grpc-Metadata-authorization: Bearer $CHIRPSTACK_API_KEY" \
        http://localhost:8080/api/applications
   ```

3. **Check ChirpStack status**:
   ```bash
   curl http://localhost:8080/api/internal/version
   ```

---

### "High memory usage"

**Problem**: System runs out of memory

**Diagnosis**:
```bash
# Check memory usage
free -h
top -o %MEM

# Check for memory leaks
valgrind --tool=memcheck ros2 run package_name node_name
```

**Solutions**:
1. **Limit InfluxDB memory** (see [memory_limitation_influxdb.md](memory_limitation_influxdb.md))

2. **Reduce ROS2 node instances**:
   ```bash
   # Check running nodes
   ros2 node list
   
   # Stop unnecessary nodes
   ```

3. **Optimize build type**:
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

---

## 🔍 Performance Issues

### "Slow message processing"

**Problem**: High latency in ROS2 message handling

**Solutions**:
1. **Check DDS configuration**:
   ```bash
   # Set ROS domain
   export ROS_DOMAIN_ID=0
   
   # Check network interfaces
   ros2 daemon stop
   ros2 daemon start
   ```

2. **Optimize Zenoh settings**:
   ```json5
   // In zenoh_router.json5
   {
     "transport": {
       "link": {
         "tx": {
           "sequence_number_resolution": "16bit"
         }
       }
     }
   }
   ```

---

### "Dashboard loading slowly"

**Problem**: Web interface is unresponsive

**Solutions**:
1. **Check backend API**:
   ```bash
   curl http://localhost:5000/api/health
   ```

2. **Restart services**:
   ```bash
   # For Docker
   docker-compose restart wisevision-dashboard-backend wisevision-frontend
   
   # For snap
   sudo snap restart wisevision-iot.backend wisevision-iot.frontend
   ```

3. **Check browser console** for JavaScript errors

---

## 📞 Getting Help

### Enable Debug Logging

```bash
# For ROS2 nodes
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 run package_name node_name

# For Zenoh
zenohd -c config.json5 -v

# For Docker
docker-compose up --verbose
```

### Collect System Information

```bash
# Create debug report
cat > debug_report.txt << EOF
=== System Info ===
$(uname -a)
$(lsb_release -a)
$(free -h)
$(df -h)

=== WiseVision Status ===
$(ros2 pkg list | grep wisevision 2>/dev/null || echo "ROS2 not available")
$(docker-compose ps 2>/dev/null || echo "Docker compose not running")
$(snap services wisevision-iot 2>/dev/null || echo "Snap not installed")

=== Network Status ===
$(netstat -tulpn | grep -E "(3000|5000|7447|8000|8086|8080)")

=== Recent Logs ===
$(journalctl -u zenohd --since "10 minutes ago" --no-pager 2>/dev/null || echo "No zenohd logs")
EOF

echo "Debug report saved to debug_report.txt"
```

### Contact Support

When reporting issues, include:

1. **Installation method** (snap/docker/local)
2. **Operating system** and version
3. **Error messages** (full output)
4. **Steps to reproduce**
5. **Debug report** (from above)

**Channels**:
- 🐛 **Bug Reports**: [GitHub Issues](https://github.com/wise-vision/wisevision.proj/issues)
- 💬 **Questions**: [GitHub Discussions](https://github.com/wise-vision/wisevision.proj/discussions)
- 📖 **Documentation**: [README.md](README.md)

---

## 🔄 Recovery Procedures

### Complete Reset (Docker)

```bash
# Stop and remove everything
docker-compose down -v
docker system prune -af

# Start fresh
git pull origin main
vcs import --recursive < project.repos
docker-compose up --build
```

### Complete Reset (Local)

```bash
# Clean build artifacts
rm -rf build/ install/ log/

# Rebuild everything
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Complete Reset (Snap)

```bash
# Remove and reinstall
sudo snap remove wisevision-iot
sudo snap install wisevision-iot

# Reconfigure
sudo snap set wisevision-iot config-file=... # (your settings)
```

---

<div align="center">

**🎯 Still having issues? Check the [GitHub Issues](https://github.com/wise-vision/wisevision.proj/issues)!**

[🏠 Back to README](README.md) • [🚀 Quick Start](QUICK_START.md) • [📋 Prerequisites](PREREQUISITES.md)

</div>
