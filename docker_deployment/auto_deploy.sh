HOST_DATA_DIR=/home/$USER/Downloads/saved_data  # Change this to your desired host directory
CONTAINER_DATA_DIR=/home/headlightai/saved_data # Here headlightai is the user in the docker container

mkdir -p "$HOST_DATA_DIR"
sudo chown $USER:$USER "$HOST_DATA_DIR"   # Match HOST_UID and HOST_GID in Dockerfile

# The device-cgroup-rules are required only for realsense cameras
# A udev rule is added to the host to allow access to the sbg_imu
# Daheng camera needs its drivers to be installed in the host
docker run -dit \
  --privileged \
  --device-cgroup-rule='c 81:* rmw' \
  --device-cgroup-rule='c 189:* rmw' \
  --restart unless-stopped  \
  -v /dev/sbg_imu:/dev/sbg_imu \
  -v "$HOST_DATA_DIR:$CONTAINER_DATA_DIR" \
  --ipc=host --pid=host --net=host \
  --name hai-telesto-ros2 \
  ghcr.io/headlightai/telesto-ros2-deploy
