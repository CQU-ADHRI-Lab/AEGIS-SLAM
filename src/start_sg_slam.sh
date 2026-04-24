#!/bin/bash

# --- 配置区 ---
DOCKER_IMAGE="sg_slam_env:v3"
CONTAINER_NAME="sg_slam_dev"

# 更新为你的外层 SLAM 目录
REPO_DIR="/home/zhoujiacong/SLAM"
DATA_DIR="/home/zhoujiacong/dataset"

# --- 清理旧容器 ---
if [ $(docker ps -aq -f name=^/${CONTAINER_NAME}$) ]; then
    echo "清理旧容器..."
    docker rm -f ${CONTAINER_NAME}
fi

echo "正在启动 SG-SLAM 开发环境..."
echo "宿主机路径: ${REPO_DIR}/SG-SLAM"
echo "映射目标: 容器内 /home/user/catkin_ws/src/SG-SLAM"

# --- 启动容器 ---
# 将 SLAM 目录挂载到 src，这样其下的 SG-SLAM 自动进入工作空间
docker run -it -d \
    --name ${CONTAINER_NAME} \
    --gpus all \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${REPO_DIR}:/home/user/catkin_ws/src \
    -v ${DATA_DIR}:/data \
    ${DOCKER_IMAGE} \
    bash

# 进入容器
docker exec -it ${CONTAINER_NAME} bash
