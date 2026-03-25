ARG BASE_OS=nvcr.io/nvidia/cuda:12.5.1-base-ubuntu24.04

FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y full-upgrade

# locale

RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# base dependencies
RUN apt-get update \
  && apt-get install -y \
  curl \
  git \
  lxqt-core \
  python3-pip \
  wget \
  && rm -rf /var/lib/apt/lists/*



RUN apt-get update \
  && apt-get install curl -y \
  && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release \
  && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
  && dpkg -i /tmp/ros2-apt-source.deb \
  && apt-get update \
  && apt-get -y full-upgrade \
  && apt-get install -y ros-jazzy-desktop \
  && rm -rf /var/lib/apt/lists/*

#ROS2 dev tools
RUN apt-get update && apt-get install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools \
  && rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc


# Gazebo

RUN apt-get update \
  && apt-get -y full-upgrade \
  && apt-get install -y ros-jazzy-ros-gz \
  && rm -rf /var/lib/apt/lists/*


ARG UNAME=el7009
ARG UID=1000
ARG GID=1000

RUN set -eux; \
    : "${UID:?UID not set}" "${GID:?GID not set}" "${UNAME:?UNAME not set}"; \
    \
    # Ensure group with GID exists and has the correct name
    if getent group "${GID}" >/dev/null; then \
        EXISTING_GROUP="$(getent group "${GID}" | cut -d: -f1)"; \
        if [ "${EXISTING_GROUP}" != "${UNAME}" ]; then \
            groupmod -n "${UNAME}" "${EXISTING_GROUP}"; \
        fi; \
    else \
        if getent group "${UNAME}" >/dev/null; then \
            groupmod -g "${GID}" "${UNAME}"; \
        else \
            groupadd -g "${GID}" "${UNAME}"; \
        fi; \
    fi; \
    \
    # Ensure user with UID exists and has correct name, group, and home
    if getent passwd "${UID}" >/dev/null; then \
        EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1)"; \
        if [ "${EXISTING_USER}" != "${UNAME}" ]; then \
            usermod -l "${UNAME}" "${EXISTING_USER}"; \
        fi; \
        usermod -g "${GID}" -d "/home/${UNAME}" -m "${UNAME}"; \
    else \
        if getent passwd "${UNAME}" >/dev/null; then \
            usermod -u "${UID}" -g "${GID}" -d "/home/${UNAME}" -m "${UNAME}"; \
        else \
            useradd -m -u "${UID}" -g "${GID}" -d "/home/${UNAME}" "${UNAME}"; \
        fi; \
    fi; \
    passwd -d "${UNAME}"

# #FIX Permisions in unterlying image
# RUN chmod -R a+rX /usr/local/lib/python3.12/dist-packages && \
#  chmod -R a+rX /root && \
#  chmod -R a+rX /usr/local 

RUN apt-get update && apt-get install -y \
      git \
      ros-jazzy-rviz2 \
      sudo \
       && \
    rm -rf /var/lib/apt/lists/*
  

USER $UNAME

WORKDIR /home/${UNAME}/ws

