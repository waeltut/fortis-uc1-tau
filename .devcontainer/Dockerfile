FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=fortis_dev
ARG UID=1000
ARG GID=$UID
ARG ROS_DOMAIN_ID=67

# -------------------------
# Base system dependencies
# -------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    wget \
    unzip \
    cmake \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libssl-dev \
    libudev-dev \
    libceres-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libasound2-dev \
    libsndfile1 \
    portaudio19-dev \
    python3-pip \
    python3-websocket \
    python3-rosdep \
    python3-colcon-common-extensions \
    software-properties-common \
    xterm \
    alsa-utils \
    pulseaudio \
    pulseaudio-utils \
    usbutils \
    v4l-utils \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    x11-xserver-utils \
    xauth x11-apps \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Build RealSense SDK (ARM)
# -------------------------
WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense \
    && cd /opt/librealsense \
    && git checkout v2.56.4 \
    && mkdir build && cd build \
    && cmake ../ \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_LIBUVC=true \
    && make -j$(nproc) \
    && make install

# -------------------------
# Build Piper
# -------------------------
    # Install Piper (offline TTS)

RUN mkdir -p /opt/piper && \
    wget https://github.com/rhasspy/piper/releases/latest/download/piper_linux_aarch64.tar.gz -O /tmp/piper.tar.gz && \
    tar -xvf /tmp/piper.tar.gz -C /opt && \
    mv /opt/piper /opt/piper_tmp && \
    mv /opt/piper_tmp /opt/piper && \
    chmod +x /opt/piper/piper && \
    rm /tmp/piper.tar.gz

    # Make Piper global
RUN ln -s /opt/piper/piper /usr/local/bin/piper

    # Install voice model
RUN mkdir -p /opt/piper/models && \
    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx -P /opt/piper/models && \
    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx.json -P /opt/piper/models

    # Optional: set default model
ENV PIPER_MODEL=/opt/piper/models/en_US-amy-medium.onnx

    
# -------------------------
# Python dependencies
# -------------------------
WORKDIR /
COPY requirements.txt /

# Keep Python packaging tools mutually compatible.
# setuptools >= 71 requires packaging >= 22 for canonicalize_version(..., strip_trailing_zero=...)
RUN python3 -m pip install --upgrade \
    pip \
    setuptools \
    wheel \
    "packaging>=22"

RUN python3 -m pip install --index-url https://download.pytorch.org/whl/cu130 \
    torch torchvision torchaudio

RUN python3 -m pip install --no-cache-dir -r requirements.txt

RUN python3 -m pip install --no-cache-dir roslibpy

# -------------------------
# Install ROS2 Humble (base only)
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-nav2-msgs \
    ros-humble-dwb-msgs \
    ros-humble-slam-toolbox \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rtabmap-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ur \
    ros-humble-ur-client-library \
    ros-humble-ur-description \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Non-root user
# -------------------------
RUN groupadd -g $GID $USERNAME && \
    useradd -m -u $UID -g $USERNAME -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# -------------------------
# ROS environment
# -------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Clone wrapper
RUN cd /home/$USERNAME/ros2_ws/src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && \
    git checkout 4.56.4

RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"


RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /home/$USERNAME/.bashrc

COPY ./ros2_entrypoint.sh /
ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]