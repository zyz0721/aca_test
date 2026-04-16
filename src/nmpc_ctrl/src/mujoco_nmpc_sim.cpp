#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>

// 用一个静态指针记录当前绑定的模型，用来检测用户是否在UI里重新加载了模型
static const mjModel* cached_model = nullptr;

struct RobotHardwareIDs {
    int left_arm_j1_qpos_adr;
    int wheel_1_vel_dof_adr;
    int left_arm_j1_ctrl_id;
    int wheel_1_steer_ctrl_id;
    int wheel_1_drive_ctrl_id;
};
static RobotHardwareIDs hw_ids;

// 只在模型加载时执行一次的初始化
void init_nmpc(const mjModel* m, mjData* d) {
    std::cout << "\n[NMPC] Detected new MuJoCo model, Initializing NMPC Solver..." << std::endl;

    // 1. 在这里初始化或重置你的 Acados / CasADi 求解器
    // mpc_solver = std::make_unique<AcadosWrapper>(...);

    // 2. 查找你在 galbot_S1_v1.0.xml 中定义的各类控制器的 ID
    // 假设 XML 中有 <position name="left_arm_joint1" ...>
    hw_ids.left_arm_j1_ctrl_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_arm_joint1"); 
    
    // 假设 XML 中有控制轮子的驱动
    hw_ids.wheel_1_steer_ctrl_id = mj_name2id(m, mjOBJ_ACTUATOR, "steer_joint_1"); 
    hw_ids.wheel_1_drive_ctrl_id = mj_name2id(m, mjOBJ_ACTUATOR, "drive_joint_1");

    if (hw_ids.left_arm_j1_ctrl_id < 0) {
        std::cerr << "[NMPC WARNING] Actuator 'left_arm_joint1' not found in XML!" << std::endl;
    }

    std::cout << "[NMPC] Initialization complete. Ready to control." << std::endl;
}

// 每次物理步长（例如 0.001s）会自动进入此函数
void nmpc_control_callback(const mjModel* m, mjData* d) {
    // 自动检测机制：如果发现当前跑的模型指针不是我们初始化的那个（比如用户点击了Reload）
    // 自动触发一次初始化逻辑，完美避开修改 simulate.cc
    if (m != cached_model) {
        init_nmpc(m, d);
        cached_model = m;
    }

    // ============================================
    // 在这里写你的 NMPC 控制循环
    // ============================================
    
    // 1. 读取状态反馈 (使用 d->qpos, d->qvel 等)
    // 2. 将状态传入 Acados: mpc_solver->set_current_state(...)
    // 3. 求解: mpc_solver->solve()
    // 4. 获取控制律
    
    // 测试：给机器人手臂写个正弦波动，验证控制已接管
    double test_ctrl = 0.5 * sin(d->time); 
    
    // 5. 下发控制到 MuJoCo 引擎
    if (hw_ids.left_arm_j1_ctrl_id >= 0) {
        d->ctrl[hw_ids.left_arm_j1_ctrl_id] = test_ctrl; 
    }
}