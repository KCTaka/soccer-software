# syntax=docker/dockerfile:1.7
FROM ubuntu@sha256:224a1869083a311ef3f13648a154ba79832fbef6364d31493642ca03082da254 AS builder

ARG COLCON_COMMON_EXTENSIONS_VERSION=0.3.0
ARG ROSDEP_VERSION=0.27.0
ARG VCSTOOL_VERSION=0.3.0
ARG MUJOCO_VERSION=3.4.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ROS_DISTRO=jazzy \
    CMAKE_PREFIX_PATH=/opt/mujoco

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# pyyaml is installed automatically
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
      python3-jsonschema \
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
      ros-jazzy-controller-manager \
      ros-jazzy-controller-interface \
      ros-jazzy-ros2controlcli \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-hardware-interface \
      ros-jazzy-pluginlib \
      ros-jazzy-rclcpp \
      ros-jazzy-rclcpp-lifecycle \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-ament-cmake \
      ros-jazzy-mujoco-vendor \
      ros-jazzy-rviz2 \
      python3-colcon-ros \
      python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# Download MuJoCo C++ binaries for the active architecture (amd64 or arm64)
RUN ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "amd64" ]; then MUJOCO_ARCH="x86_64"; \
    elif [ "$ARCH" = "arm64" ]; then MUJOCO_ARCH="aarch64"; \
    else echo "Unsupported architecture: $ARCH" && exit 1; fi && \
    curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz" \
    | tar -xz -C /opt/ \
 && ln -s /opt/mujoco-${MUJOCO_VERSION} /opt/mujoco

RUN python3 -m pip install --no-cache-dir --break-system-packages \
      "colcon-common-extensions==${COLCON_COMMON_EXTENSIONS_VERSION}" \
      "rosdep==${ROSDEP_VERSION}" \
      "vcstool==${VCSTOOL_VERSION}" \
      "mujoco==${MUJOCO_VERSION}"

RUN echo '/opt/mujoco-3.4.0/lib' > /etc/ld.so.conf.d/mujoco.conf && ldconfig

WORKDIR /ws
COPY tools/pin_audit.py /usr/local/bin/pin_audit.py
RUN chmod 0755 /usr/local/bin/pin_audit.py

ENTRYPOINT ["/bin/bash"]