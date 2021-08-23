# Dieses Dockerfile erstellt die Umgebung für den Beispielcode des Use Cases Suchen 2.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>


# Setzten der Plattform für GPU Beschleunigung, siehe readme
ARG GRAPHICS_PLATFORM=cpu

# CPU Image braucht keine Modifikation
FROM ros:melodic-ros-core as build_cpu

# Opensource Image benötigt mesa Updates
FROM ros:melodic-ros-core as build_opensource
ONBUILD RUN apt-get update && apt-get -y install libgl1-mesa-glx libgl1-mesa-dri

# Amdpro Image benötigt den amdgpu pro treiber. Dieser muss manuell heruntergeladen und im Verzeichnis des Usecases plaziert werden
FROM ros:melodic-ros-core as build_amdpro

ONBUILD ENV AMDGPUDRIVERFILE="amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz"
ONBUILD ENV AMDGPUDIRNAME="amdgpu-pro-20.45-1188099-ubuntu-20.04"

ONBUILD ADD ./$AMDGPUDRIVERFILE .

# Installation der amdgpu pro Voraussetzungen
ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    pciutils \
    wget \
    build-essential \
    cmake \
    libboost-all-dev \
    apt-utils \
    && dpkg --add-architecture i386

ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
            libgl1-mesa-glx libgl1-mesa-dri

# Manuelle Installation der amdgpu pro Komponenten neben dem open source amgpu Treiber
ONBUILD RUN cd $AMDGPUDIRNAME \
    && ./amdgpu-install -y --no-dkms \
    && apt install -y \
    amdgpu-core \
    comgr-amdgpu-pro \
    hip-rocr-amdgpu-pro \
    hsa-runtime-rocr-amdgpu \
    hsakmt-roct-amdgpu \
    libdrm-amdgpu-amdgpu1 \
    libdrm-amdgpu-common \
    libdrm2-amdgpu \
    ocl-icd-libopencl1-amdgpu-pro \
    opencl-rocr-amdgpu-pro \
    && dpkg -i opencl-rocr-amdgpu-pro_*_amd64.deb \
    && dpkg -i opencl-rocr-amdgpu-pro-dev_*_amd64.deb \
    && dpkg -i rocm-device-libs-amdgpu-pro_*_amd64.deb \
    && dpkg -i hsa-runtime-rocr-amdgpu_*_amd64.deb \
    && dpkg -i hsa-runtime-rocr-amdgpu-dev_*_amd64.deb \
    && dpkg -i hsakmt-roct-amdgpu_*_amd64.deb \
    && dpkg -i hsakmt-roct-amdgpu-dev_*_amd64.deb \
    && dpkg -i hip-rocr-amdgpu-pro_*_amd64.deb \
    && dpkg -i comgr-amdgpu-pro_*_amd64.deb \
    && dpkg -i comgr-amdgpu-pro-dev_*_amd64.deb \
    && dpkg -i opencl-orca-amdgpu-pro-icd_*_amd64.deb \
    && dpkg -i libdrm-amdgpu-amdgpu1_*_amd64.deb \
    && dpkg -i libdrm2-amdgpu_*_amd64.deb \
    && cd .. \
    && rm -rf $AMDGPUDIRNAME \
    && rm -rf /var/lib/{apt,dpkg,cache,log} \
    && apt install -y ocl-icd-opencl-dev

# Das Nvidia Image benötigt das Container Toolkit. (basierend auf https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html)
FROM nvidia/cudagl:10.0-runtime-ubuntu18.04 as build_nvidia

# Nachbau des ROS Melodic Images. (basierend auf https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-core/Dockerfile)
ONBUILD RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

ONBUILD RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

ONBUILD RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
ONBUILD RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

ONBUILD ENV LANG C.UTF-8
ONBUILD ENV LC_ALL C.UTF-8

ONBUILD RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

ONBUILD RUN rosdep init && \
  rosdep update --rosdistro melodic

#########################################################################
##########    Hier startet die Erstellung des finalen Images   ##########

# Die Basis für das Image wird anhand der GPU Beschleunigung gewählt
FROM build_${GRAPHICS_PLATFORM}

RUN apt-get update \
    && apt-get install -y build-essential python-catkin-tools python-rosdep cmake python-catkin-lint

# Installation der vollen ROS Melodic umgebung
# Damit werden auch die GUI Tools zur Simulation installiert
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full

# Installation der ROS Python 3 Komponenten
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-tk \
    python3-venv \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python3-wstool \
    && python3 -m venv ~/myenv

# Upgrade zur neuesten PIP version
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip \
    && deactivate"

# Installation der benötigten Python 3 Pakete
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    wheel \
    && pip3 install -U rosdep \
    rospkg \
    rosinstall_generator \
    wstool \
    rosinstall \
    empy \
    catkin_tools \
    defusedxml \
    numpy \
    mazelib \
    matplotlib \
    && pip3 install --upgrade setuptools "

# Kopieren der MiR100 Pakete
COPY ./src /catkin_ws/src
WORKDIR /catkin_ws

# Rosdep (basierend auf Python 3) wird verwendet, um alle für die MiR100 Integration benötikten Pakete zu installieren
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src -i -y --rosdistro melodic"

# Kompilation des Catkin Workspaces (die Argumente aktivieren Python 3 für diesen Workspace)
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash \
    && source ~/myenv/bin/activate \
    && catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so"

# Beheben des ignition Fehlers in Gazebo 9 und früher
# https://answers.gazebosim.org//question/22263/error-in-rest-request-for-accessing-apiignitionorg/
# RUN sed -i "s|https://api.ignitionfuel.org|https://api.ignitionrobotics.org |g" ~/.ignition/fuel/config.yaml
RUN mkdir -p ~/.ignition/fuel && touch ~/.ignition/fuel/config.yaml \
    && echo "---" >> ~/.ignition/fuel/config.yaml \
    && echo "servers:" >> ~/.ignition/fuel/config.yaml \
    && echo "  -" >> ~/.ignition/fuel/config.yaml \
    && echo "      name: osrf" >> ~/.ignition/fuel/config.yaml \
    && echo "      url: https://api.ignitionrobotics.org" >> ~/.ignition/fuel/config.yaml

# Reset der Workdirectory
WORKDIR /

