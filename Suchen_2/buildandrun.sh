#!/bin/bash

# Dieses Skript führt den Beispielcode für die Suchen 2 Labyrinthnavigation aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

mkdir src
# get mir packages, check for updates if the repo is already cloned
git clone https://github.com/dfki-ric/mir_robot.git || (cd mir_robot ; git pull)
cd ..
# set environment variable for graphics acceleration in the container
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# if GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-cpu}"
echo Using graphics platform $GRAPHICS_PLATFORM

# build container
docker build -t mir_search --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM .

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