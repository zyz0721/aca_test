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
include_directories(
    ${MUJOCO_DIR}/include
    ${MUJOCO_DIR}/simulate  # 包含 simulate.h 等 UI 头文件
)
link_directories(${MUJOCO_DIR}/lib)

# ==========================================
# 3. 查找系统级的图形学库 (用于 MuJoCo 可视化)
# ==========================================
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED) 
find_package(yaml-cpp REQUIRED)

# ==========================================
# 修复 lodepng 依赖
# ==========================================
set(LODEPNG_DIR "${CMAKE_BINARY_DIR}/third_party/lodepng")
if(NOT EXISTS "${LODEPNG_DIR}/lodepng.h")
    message(STATUS "Downloading missing lodepng for MuJoCo Simulate UI...")
    file(DOWNLOAD "https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.h" "${LODEPNG_DIR}/lodepng.h")
    file(DOWNLOAD "https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.cpp" "${LODEPNG_DIR}/lodepng.cpp")
endif()
include_directories(${LODEPNG_DIR}) # 让 simulate.cc 能找到 lodepng.h

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

# 收集 MuJoCo 原版 UI界面的源文件
set(MUJOCO_SIMULATE_SRC
    ${MUJOCO_DIR}/simulate/simulate.cc
    ${MUJOCO_DIR}/simulate/glfw_dispatch.cc
    ${MUJOCO_DIR}/simulate/platform_ui_adapter.cc
    ${MUJOCO_DIR}/simulate/glfw_adapter.cc
    ${LODEPNG_DIR}/lodepng.cpp
)

# ==========================================
# 6. 生成可执行文件
# ==========================================
# 将新的 mujoco_nmpc_sim.cpp 作为 main 函数所在文件
# add_executable(mujoco_sim_run 
#     src/nmpc_ctrl/src/mujoco_nmpc_sim.cpp 
#     ${NMPC_SRC}
#     ${MUJOCO_SIMULATE_SRC}        # MuJoCo 原版界面源码
# )
add_executable(mujoco_sim_run
    ${MUJOCO_DIR}/simulate/main.cc
    ${NMPC_SRC}
    ${MUJOCO_SIMULATE_SRC}        # MuJoCo 原版界面源码
    src/nmpc_ctrl/src/mujoco_chassis_nmpc_sim_v2.cpp
    # src/nmpc_ctrl/src/mujoco_chassis_nmpc_sim.cpp
)

add_executable(arm_sim_run
    ${MUJOCO_DIR}/simulate/main.cc
    ${NMPC_SRC}
    ${MUJOCO_SIMULATE_SRC}        # MuJoCo 原版界面源码
    src/nmpc_ctrl/src/mujoco_nmpc_sim.cpp 
)

# 链接所有的动态链接库
target_link_libraries(mujoco_sim_run
    casadi
    acados
    blasfeo
    hpipm
    mujoco
    yaml-cpp
    glfw
    OpenGL::GL
    dl
    m
)

target_link_libraries(arm_sim_run
    casadi
    acados
    blasfeo
    hpipm
    mujoco
    yaml-cpp
    glfw
    OpenGL::GL
    dl
    m
)