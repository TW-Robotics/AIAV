#!/bin/bash

# Dieses Skript führt den Beispielcode für den Embedded AI HW Use Case aus.
# Basiert auf: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/scripts/run_dev.sh
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2024 <schwaige@technikum-wien.at>

docker build -f Dockerfile.jetson -t jetson_detectron:latest .

# Setzen der korrekten Docker Argumente für Jetson Boards
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
DOCKER_ARGS+=("--pid=host")
DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

# If jtop present, give the container access
#if [[ $(getent group jtop) ]]; then
#    DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
#    JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
#    DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
#fi

xhost +

# Start von JupyerLab im Container
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $PWD/app:/app \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/video1:/dev/video1 \
    --name "jetson_detectron_container" \
    --runtime nvidia \
    --workdir /app \
    $@ \
    jetson_detectron:latest \
    python3 -m jupyterlab --ip='0.0.0.0' --no-browser --allow-root
