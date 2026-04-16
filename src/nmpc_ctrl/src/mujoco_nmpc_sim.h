#ifndef MUJOCO_NMPC_SIM_H
#define MUJOCO_NMPC_SIM_H

#include <mujoco/mujoco.h>

// 初始化NMPC求解器（在simulate.cc的主循环前调用一次）
void init_nmpc(const mjModel* m, mjData* d);

// MuJoCo原生控制回调函数（将在物理引擎的每次step前自动触发）
void nmpc_control_callback(const mjModel* m, mjData* d);

#endif // MUJOCO_NMPC_SIM_H