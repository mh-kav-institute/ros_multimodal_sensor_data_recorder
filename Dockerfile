# README!

### --- Description:
#
# This file creates an docker image for detectron2 framework
# Check compatibility for your hardware, i.e GPU (see below)
#

### ---- Docker build and run command:
#
# docker build -t <image_name>:<version> .
# docker run -it --gpus all --restart=unless-stopped --shm-size=16g --name=<container_name> -v <your_local_data_path>:/workspace/data 
# -v <your_local_project_path>:/workspace/repos /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -p <Port>:22 --rm <image_name>
#

# --- Includes:
#	- Ubuntu 20.4 (with September 2021 updates)
#	- CUDA 11.3.1
#	- CUDNN 8.2.1
#   - TensorRT 7.2.3.4
#	- Pytorch 1.9

# Define base image
FROM nvcr.io/nvidia/cuda:11.5.0-devel-ubuntu20.04

# User specific params
ARG USER=kav
ARG PW=kav
ARG UID=1000
ARG GID=1000

# Variables
ENV SHELL /bin/bash
ENV DEBIAN_FRONTEND noninteractive

# Set ssh daemon to allow root login
RUN echo 'root:root' | chpasswd
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PW}" | chpasswd && adduser ${USER} sudo

# Additional apt installations
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        pkg-config \
        apt-utils \
        build-essential \
        nano \
        yasm \
        git \
        acl \
        python3-pip \
        python3-dev \
        python3-opencv \
        sudo \
        openssh-server \
        htop \
        nload \
		cmake \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# The place for your repositories and data
RUN mkdir -p /workspace/repos && mkdir /workspace/data && mkdir -p /workspace/install

# Install additional python packages from requirements.txt
COPY requirements.txt /workspace
RUN python3 -m pip install --upgrade pip && pip install --upgrade -r /workspace/requirements.txt

# Copy entrypoint script
COPY entrypoint.sh /workspace/entrypoint.sh
COPY entrypoint_dev.sh /workspace/entrypoint_dev.sh
RUN chmod +x /workspace/entrypoint.sh && chmod +x /workspace/entrypoint_dev.sh

# Set workspace path
CMD ["/bin/bash"]
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
USER ${UID}:${GID}
WORKDIR /workspace