# 使用官方 ROS Noetic 基础镜像
FROM osrf/ros:noetic-desktop-full

# 替换为阿里云源加速 apt 下载
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list

# 安装 SG-SLAM 核心依赖及编译工具
RUN apt-get update && \
    apt-get install -y --no-install-recommends libtbb-dev software-properties-common python3-catkin-tools python3-osrf-pycommon && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt-get update && \
    apt-get install -y --no-install-recommends libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# 自动 source ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 预先创建工作空间结构
RUN mkdir -p /home/user/catkin_ws/src

# 设置默认进入的路径为工作空间根目录
WORKDIR /home/user/catkin_ws
