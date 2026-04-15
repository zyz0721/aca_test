cmake_minimum_required(VERSION 3.10)
project(galbot_mujoco_nmpc)

set(CMAKE_CXX_STANDARD 17)

# ==========================================
# 1. 外部依赖库的绝对路径设置
# ==========================================
set(CASADI_DIR "/home/galbot/casadi-3.5.5")
set(ACADOS_DIR "/home/galbot/Ipopt_pkg/acados")
set(MUJOCO_DIR "/home/galbot/mujoco")

# ==========================================
# 2. 引入外部依赖库的头文件和链接目录
# ==========================================
# CasADi
include_directories(${CASADI_DIR}/include)
link_directories(${CASADI_DIR}/lib)

# Acados & Blasfeo & Hpipm
include_directories(
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
)
link_directories(${ACADOS_DIR}/lib)

# MuJoCo
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/lib)

# ==========================================
# 3. 查找系统级的图形学库 (用于 MuJoCo 可视化)
# ==========================================
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED) 

# ==========================================
# 4. 引入本项目(aca_test)的头文件
# ==========================================
include_directories(
    src/nmpc_ctrl/include
    src/nmpc_ctrl/src
)

# ==========================================
# 5. 收集需要编译的 C++ 源代码文件
# ==========================================
# 我们需要 NMPC 后端相关的源文件，但排除掉包含 ROS 相关的 mpc_node.cpp
file(GLOB NMPC_SRC 
    "src/nmpc_ctrl/src/mpc_backend/*.cpp"
    "src/nmpc_ctrl/src/mpc_solver.cpp"
)

# ==========================================
# 6. 生成可执行文件
# ==========================================
# 将新的 mujoco_nmpc_sim.cpp 作为 main 函数所在文件
add_executable(mujoco_sim_run 
    src/nmpc_ctrl/src/mujoco_nmpc_sim.cpp 
    ${NMPC_SRC}
)

# 链接所有的动态链接库
target_link_libraries(mujoco_sim_run
    casadi
    acados
    blasfeo
    hpipm
    mujoco
    glfw
    OpenGL::GL
    dl
    m
)