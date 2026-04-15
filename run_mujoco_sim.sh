#!/bin/bash

# ==========================================
# 1. 设置环境变量 LD_LIBRARY_PATH
# ==========================================
# 告诉系统去哪里加载 CasADi, Acados 和 MuJoCo 的动态库 (.so 文件)
export LD_LIBRARY_PATH=/home/galbot/mujoco/lib:/home/galbot/casadi-3.5.5/lib:/home/galbot/Ipopt_pkg/acados/lib:$LD_LIBRARY_PATH

# ==========================================
# 2. 进入项目根目录
# ==========================================
# 确保在 /home/galbot/galbot_ws/aca_test 下执行
cd /home/galbot/galbot_ws/aca_test

# ==========================================
# 3. 创建构建目录并编译
# ==========================================
echo "==> Preparing build directory..."
mkdir -p build_mujoco
cd build_mujoco

echo "==> Running CMake..."
/usr/bin/cmake ..

echo "==> Running Make..."
make -j$(nproc) # 自动使用所有可用的 CPU 核心进行并行编译

# ==========================================
# 4. 运行可执行文件
# ==========================================
if [ $? -eq 0 ]; then
    echo "==> Build successful! Starting MuJoCo Simulation..."
    # 确保在运行可执行文件时，相对路径是正确的，因此我们在 aca_test 根目录下运行它
    cd .. 
    ./build_mujoco/mujoco_sim_run
else
    echo "==> Build failed. Please check the errors above."
fi