#!/bin/bash

IMAGE_NAME="ros1_mpc_galbot"
CONTAINER_NAME="mpc_dev_env"

# 1. 构建 Docker 镜像
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "正在构建 Docker 镜像 $IMAGE_NAME..."
    docker build -t $IMAGE_NAME .
fi

# 2. 允许本地 X11 访问 (为了能够显示 Gazebo 和 Rviz)
xhost +local:root

# 3. 运行容器 (启用了 NVIDIA GPU 硬件加速)
echo "正在启动容器(启用 NVIDIA GPU 支持)..."
docker run -it --rm \
    --name $CONTAINER_NAME \
    --net=host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/workspace" \
    $IMAGE_NAME \
    bash -c "source /opt/ros/noetic/setup.bash && bash"
