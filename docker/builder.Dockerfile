# syntax=docker/dockerfile:1.7
FROM ubuntu@sha256:224a1869083a311ef3f13648a154ba79832fbef6364d31493642ca03082da254 AS builder

ARG COLCON_COMMON_EXTENSIONS_VERSION=0.3.0
ARG ROSDEP_VERSION=0.27.0
ARG VCSTOOL_VERSION=0.3.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
      ca-certificates \
      curl \
      gnupg \
      build-essential \
      cmake \
      git \
      pkg-config \
      python3 \
      python3-dev \
      python3-pip \
      python3-venv \
 && rm -rf /var/lib/apt/lists/*

RUN install -d -m 0755 /etc/apt/keyrings \
 && curl --fail --location --silent --show-error \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      --output /etc/apt/keyrings/ros-archive-keyring.gpg \
 && test "$(gpg --show-keys --with-colons /etc/apt/keyrings/ros-archive-keyring.gpg | awk -F: '$1 == "fpr" { print $10; exit }')" = "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654" \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install --no-install-recommends -y \
      ros-jazzy-ros-base \
      ros-jazzy-hardware-interface \
      ros-jazzy-pluginlib \
      ros-jazzy-rclcpp \
      ros-jazzy-rclcpp-lifecycle \
      ros-jazzy-ament-cmake \
      python3-colcon-ros \
      python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages \
      "colcon-common-extensions==${COLCON_COMMON_EXTENSIONS_VERSION}" \
      "rosdep==${ROSDEP_VERSION}" \
      "vcstool==${VCSTOOL_VERSION}"

WORKDIR /ws
COPY tools/pin_audit.py /usr/local/bin/pin_audit.py
RUN chmod 0755 /usr/local/bin/pin_audit.py

ENTRYPOINT ["/bin/bash"]