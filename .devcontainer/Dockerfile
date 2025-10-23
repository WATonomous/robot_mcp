# ROS2 CPU development container
ARG ROS_DISTRO=humble

# ===============================================
# Install Common Development Tools
# ===============================================
FROM ros:${ROS_DISTRO}-ros-base AS dev-tools

# Install development tools not in base image
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    openssh-client \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create a system-wide virtual environment and install pre-commit
RUN python3 -m venv --system-site-packages /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir pre-commit==3.8.0

# Set working dir (matches VSCode workspace)
WORKDIR /deep_ros_ws

# ===============================================
# Gather Dependencies
# ===============================================
FROM dev-tools AS source

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN --mount=type=bind,source=.,target=/tmp/src \
    apt-get -qq update && rosdep update && \
    rosdep install --from-paths /tmp/src --ignore-src -r -s \
        | (grep 'apt-get install' || true) \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

# ===============================================
# Install Dependencies
# ===============================================
FROM dev-tools AS dependencies

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get update && \
    xargs -a /tmp/colcon_install_list apt-get install -qq -y --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# ===============================================
# Add User Configuration
# ===============================================
FROM dependencies as user-conf
ARG USERNAME
ARG USER_GID
ARG USER_UID
ARG ROS_DISTRO

# Cater image to user
SHELL ["/bin/bash", "-c"]
# hadolint ignore=SC2086
RUN groupadd --gid ${USER_GID} ${USERNAME} || groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1) \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m $USERNAME --shell /bin/bash || usermod -l ${USERNAME} -d /home/${USERNAME} -m $(getent passwd ${USER_UID} | cut -d: -f1) \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(ALL\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && cp /etc/skel/.bashrc /home/$USERNAME/.bashrc \
    && cp /etc/skel/.profile /home/$USERNAME/.profile \
    && chown $USERNAME:$USERNAME /home/$USERNAME/.bashrc /home/$USERNAME/.profile \
    && rm -rf /var/lib/apt/lists/*

# Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Install Claude Code natively
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN curl -fsSL https://claude.ai/install.sh | bash

# Source ROS and Python venv in user's bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/venv/bin/activate" >> /home/$USERNAME/.bashrc
