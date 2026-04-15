# 使用官方 ROS Noetic 桌面完整版
FROM osrf/ros:noetic-desktop-full

# 设置环境变量，避免安装过程中的交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装基础编译工具、依赖和常用工具
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    nano \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    coinor-libipopt-dev \
    libblas-dev \
    liblapack-dev \
    gfortran \
    && rm -rf /var/lib/apt/lists/*

# # ==========================================
# # 安装 MuJoCo 3.3.0
# # ==========================================
# RUN apt-get update && apt-get install -y \
#     wget \
#     libgl1-mesa-dev \
#     libgl1-mesa-glx \
#     libosmesa6-dev \
#     libglew-dev \
#     patchelf \
#     && rm -rf /var/lib/apt/lists/*

# RUN mkdir -p /root/.mujoco \
#     && wget https://github.com/google-deepmind/mujoco/releases/download/3.3.0/mujoco-3.3.0-linux-x86_64.tar.gz -O mujoco.tar.gz \
#     && tar -zxvf mujoco.tar.gz -C /root/.mujoco \
#     && rm mujoco.tar.gz

# # 配置环境变量
# ENV MUJOCO_DIR=/root/.mujoco/mujoco-3.3.0
# ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}
# # 如果你的宿主机有显卡并配置了 nvidia-docker，推荐使用 EGL 渲染加快仿真速度
# ENV MUJOCO_GL="egl"

# 设置为清华镜像源
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip3 install --upgrade pip

# 创建目标目录
RUN mkdir -p /home/galbot/Ipopt_pkg

# ==========================================
# 1. 使用本地 CasADi 源码重新编译
# ==========================================
COPY casadi_local /home/galbot/casadi-3.5.5
WORKDIR /home/galbot/casadi-3.5.5
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON .. && \
    make -j$(nproc) && \
    make install

# ==========================================
# 2. 使用本地 Acados 源码重新编译
# ==========================================
COPY acados_local /home/galbot/Ipopt_pkg/acados
WORKDIR /home/galbot/Ipopt_pkg/acados
RUN mkdir -p build && cd build && \
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON .. && \
    make -j$(nproc) && \
    make install

# 安装 acados 的 Python接口
RUN pip3 install /home/galbot/Ipopt_pkg/acados/interfaces/acados_template
ENV ACADOS_SOURCE_DIR=/home/galbot/Ipopt_pkg/acados

# 设置动态链接库路径
ENV LD_LIBRARY_PATH="/home/galbot/casadi-3.5.5/build/lib:/home/galbot/Ipopt_pkg/acados/lib:/usr/local/lib"

# 设置工作空间目录
WORKDIR /workspace

# 默认加载 ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# RUN apt-get update && apt-get install -y \
#     ros-noetic-velocity-controllers \
#     ros-noetic-effort-controllers \
#     && rm -rf /var/lib/apt/lists/*

# 默认启动 bash
CMD ["bash"]
