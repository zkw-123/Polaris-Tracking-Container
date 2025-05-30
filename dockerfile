FROM ros:humble-ros-base

# 安装必要的 ROS 2 包
RUN apt-get update && apt-get install -y \
    python3-pip \
    libusb-1.0-0-dev \
    udev \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-ros \
    ros-humble-rclpy \
    python3-colcon-common-extensions \
    usbutils \   
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 包
RUN pip3 install numpy scikit-surgerynditracker

# 创建工作空间
RUN mkdir -p /ros2_ws/src

# 设置工作目录
WORKDIR /ros2_ws

# 复制入口脚本
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# 设置入口脚本
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
