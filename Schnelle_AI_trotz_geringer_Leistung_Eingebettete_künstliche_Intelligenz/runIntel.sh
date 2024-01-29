#!/bin/bash

# Dieses Skript führt den Beispielcode für den Embedded AI HW Use Case aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2024 <schwaige@technikum-wien.at>

docker build -f Dockerfile.intel -t intel_detectron:latest .

xhost +local:docker

# Setzen der korrekten Docker Argumente
DOCKER_ARGS+=("--device=/dev/dri:/dev/dri")
DOCKER_ARGS+=("-e DISPLAY=$DISPLAY")
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $PWD/app:/app")
DOCKER_ARGS+=("--net=host")
DOCKER_ARGS+=("--workdir /app")

# Start von JupyterLab im Container
docker run -it --rm --name ros_ml_container \
            $DOCKER_RUN_ARGS ${DOCKER_ARGS[@]} \
            intel_detectron:latest python3 -m jupyterlab --ip='0.0.0.0' --no-browser --allow-root