#!/bin/bash

# Dieses Skript führt den Beispielcode für die Suchen 2 Labyrinthnavigation aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

mkdir src
cd src
# Get mir packages, check for updates if the repo is already cloned
git clone https://github.com/dfki-ric/mir_robot.git || (cd src; git pull; cd ..)
cd ..
# Set environment variable for graphics acceleration in the container
# Possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# If GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-cpu}"
echo Using graphics platform $GRAPHICS_PLATFORM

# if amdpro driver is used, download it if it is not already present
AMDPROFILE="amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz"
if [ "$GRAPHICS_PLATFORM" == "amdpro" ] && [ ! -f "$AMDPROFILE" ]; then
    wget --referer=http://support.amd.com  https://drivers.amd.com/drivers/linux/amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz
fi

# Build container
docker build -t mir_search --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM .

# Set xhost permissions for docker
# TODO better solution
# https://unix.stackexchange.com/questions/330366/how-can-i-run-a-graphical-application-in-a-container-under-wayland
xhost +local:docker

if [ "$GRAPHICS_PLATFORM" == "nvidia" ]; then
    # NVIDIA
    # run container with necessary args
    #xhost + #TODO: check if xhost + is necessary on some os'
    docker run -it \
                --rm \
                --name mir_search \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --gpus all \
               mir_search:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "cpu" ]; then
    # CPU
    # run normally, without passing through any devices
    #xhost +
    docker run -it \
                --rm \
                --name mir_search \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "amdpro" ]; then
    # run container in normal mode but pass through dri and kfd devices
    #xhost +
    docker run -it \
                --rm \
                --name mir_search \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --device=/dev/dri \
                --device=/dev/kfd \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
else
    # OPENSOURCE and INTEL
    # run container in normal mode but pass through dri device
    #xhost +
    docker run -it \
                --rm \
                --name mir_search \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --device=/dev/dri \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
fi