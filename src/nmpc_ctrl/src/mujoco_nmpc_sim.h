#ifndef MUJOCO_NMPC_SIM_H
#define MUJOCO_NMPC_SIM_H

#include <mujoco/mujoco.h>
#include <vector>
#include <mutex>

// ==========================================
// 1. 定义 MPC UI 控制参数和轨迹存储结构
// ==========================================
struct MPCUIState {
    int enable_mpc = 1;          // UI 复选框: 1 开启, 0 关闭
    int show_trajectory = 1;     // UI 复选框: 是否显示轨迹
    int horizon_steps = 20;      // UI 滑动条: 显示预测步数

    // 存储 NMPC 预测的末端(End-Effector) 3D 轨迹
    std::vector<std::vector<double>> predicted_ee_pos; 
    std::mutex mtx;              // 保护数据跨线程读写
};

extern MPCUIState g_mpc_ui_state;

// 初始化NMPC求解器（在simulate.cc的主循环前调用一次）
void init_nmpc(const mjModel* m, mjData* d);

// MuJoCo原生控制回调函数（将在物理引擎的每次step前自动触发）
void nmpc_control_callback(const mjModel* m, mjData* d);

// ==========================================
// 2. 轨迹渲染接口 (供 simulate.cc 调用)
// ==========================================
// 在 mjv_updateScene 之后调用此函数，将轨迹注入到场景中
void render_mpc_trajectory(mjvScene* scn);

#endif // MUJOCO_NMPC_SIM_H