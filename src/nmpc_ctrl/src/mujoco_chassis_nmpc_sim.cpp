#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <mutex>

// 包含你的四舵轮 NMPC 头文件
#include "mpc_solver.h"
#include "mpc_common.h"
#include "reference_generator.h"
#include "steering_ik.h"

// ==========================================================
// 全局状态与资源
// ==========================================================
MPCUIState g_mpc_ui_state;

static const mjModel* cached_model = nullptr;

// 建议使用 unique_ptr 管理生命周期
static std::unique_ptr<MPCSolver> chassis_mpc_solver;
static std::unique_ptr<ReferenceGenerator> ref_generator;

static std::unique_ptr<SteeringIK> steering_ik_solver; 

// 存储硬件在 MuJoCo 内存数组中的索引
struct ChassisHWIDs {
    int base_qpos_adr = -1;
    int base_qvel_adr = -1;
    std::vector<int> drive_ctrl_idx;
    std::vector<int> steer_ctrl_idx;
    std::vector<int> steer_qpos_idx;
    
    ChassisHWIDs() : drive_ctrl_idx(4, -1), steer_ctrl_idx(4, -1), steer_qpos_idx(4, -1) {}
};
static ChassisHWIDs hw_ids;

// MPC 运行频率控制
static double last_mpc_time = -1.0;
static const double MPC_DT = 0.02; // 50Hz = 0.02s

// 轨迹存储
static std::vector<std::vector<double>> g_ref_traj; // 参考轨迹
static std::vector<std::vector<double>> g_act_traj; // 实际轨迹
static const size_t MAX_ACT_TRAJ_LEN = 300;         // 实际轨迹保留的最大长度

// 辅助函数：四元数转 Yaw 角
static double get_yaw_from_quaternion(double w, double x, double y, double z) {
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// ==========================================================
// 初始化函数
// ==========================================================
void init_nmpc(const mjModel* m, mjData* d) {
    cached_model = m;
    
    // 1. 寻找 Base Link 索引 (根据 XML 中 Free Joint 的名字)
    int base_jnt = mj_name2id(m, mjOBJ_JOINT, "base_joint"); 
    if(base_jnt != -1) {
        hw_ids.base_qpos_adr = m->jnt_qposadr[base_jnt];
        hw_ids.base_qvel_adr = m->jnt_dofadr[base_jnt];
    } else {
        std::cerr << "[Warning] Could not find base_joint" << std::endl;
    }

    // 2. 寻找四舵轮执行器和关节的索引
    std::string drive_names[4] = {"wheel_1_drive_motor", "wheel_2_drive_motor", "wheel_3_drive_motor", "wheel_4_drive_motor"};
    std::string steer_names[4] = {"wheel_1_steer_motor", "wheel_2_steer_motor", "wheel_3_steer_motor", "wheel_4_steer_motor"};
    std::string steer_jnt_names[4] = {"Wheel_1_direction_joint", "Wheel_2_direction_joint", "Wheel_3_direction_joint", "Wheel_4_direction_joint"};
    
    for(int i = 0; i < 4; ++i) {
        hw_ids.drive_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, drive_names[i].c_str());
        hw_ids.steer_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, steer_names[i].c_str());
        int steer_jnt = mj_name2id(m, mjOBJ_JOINT, steer_jnt_names[i].c_str());
        if(steer_jnt != -1) hw_ids.steer_qpos_idx[i] = m->jnt_qposadr[steer_jnt];
    }

    std::cout << "FIND MUJOCO JOINT" << std::endl;

    // 3. 初始化 NMPC 求解器
    std::string config_path = "/home/galbot/galbot_ws/aca_test/src/nmpc_ctrl/config/mpc_config.yaml";
    MpcParams params;
    params.loadFromYaml(config_path);

    chassis_mpc_solver = std::make_unique<MPCSolver>(params.solver_cfg);
    
    ref_generator = std::make_unique<ReferenceGenerator>();
    ref_generator->init(params, params.solver_cfg.nx, params.solver_cfg.nu);

    steering_ik_solver = std::make_unique<SteeringIK>();
    steering_ik_solver->init(params); 

    // 清空历史轨迹
    g_ref_traj.clear();
    g_act_traj.clear();
    
    std::cout << "[NMPC] Chassis NMPC Initialized." << std::endl;
}

// ==========================================================
// 控制回调函数
// ==========================================================
void nmpc_control_callback(const mjModel* m, mjData* d) {
    if (!g_mpc_ui_state.enable_mpc) return;

    // 重新加载模型时自动重新初始化
    if (m != cached_model) {
        init_nmpc(m, d);
        cached_model = m;
    }

    if (!chassis_mpc_solver) return;
    std::cout << "STEP Fin" << std::endl;

    double current_time = d->time;

    // 记录实际轨迹用于可视化 (无论是否到 MPC 结算周期都记录)
    if (hw_ids.base_qpos_adr != -1) {
        double current_x = d->qpos[hw_ids.base_qpos_adr];
        double current_y = d->qpos[hw_ids.base_qpos_adr + 1];
        // 假设底盘高度 Z 在 0.1 左右，用于渲染线段
        g_act_traj.push_back({current_x, current_y, 0.1});
        if (g_act_traj.size() > MAX_ACT_TRAJ_LEN) g_act_traj.erase(g_act_traj.begin());
    }

    // 频率控制 (例如 50Hz)
    if (current_time - last_mpc_time < MPC_DT) {
        return; 
    }
    last_mpc_time = current_time;

    // 1. 提取当前底盘状态
    std::vector<double> current_state(6, 0.0);
    if (hw_ids.base_qpos_adr != -1) {
        current_state[0] = d->qpos[hw_ids.base_qpos_adr];
        current_state[1] = d->qpos[hw_ids.base_qpos_adr + 1];
        current_state[2] = get_yaw_from_quaternion(
            d->qpos[hw_ids.base_qpos_adr + 3], d->qpos[hw_ids.base_qpos_adr + 4], 
            d->qpos[hw_ids.base_qpos_adr + 5], d->qpos[hw_ids.base_qpos_adr + 6]);
    }
    if (hw_ids.base_qvel_adr != -1) {
        current_state[3] = d->qvel[hw_ids.base_qvel_adr];
        current_state[4] = d->qvel[hw_ids.base_qvel_adr + 1];
        current_state[5] = d->qvel[hw_ids.base_qvel_adr + 5];
    }

    std::vector<double> current_steer_angles(4, 0.0);
    for(int i=0; i<4; ++i) {
        if(hw_ids.steer_qpos_idx[i] != -1) {
            current_steer_angles[i] = d->qpos[hw_ids.steer_qpos_idx[i]];
        }
    }

    // 2. 解算 MPC
    int N = chassis_mpc_solver->N();
    int nx = chassis_mpc_solver->get_x_dimension(); // 通常为 6
    int nu = chassis_mpc_solver->get_u_dimension(); // 通常为 3

    // 1. 设置当前真实状态 x0
    chassis_mpc_solver->set_x0(current_state.data());

    // 2. 循环遍历预测视野 N，设置每个 stage 的参考轨迹
    for (int stage = 0; stage <= N; ++stage) {
        double pred_time = current_time + stage * MPC_DT;
        auto ref = ref_generator->at(pred_time); // 调用你的 at() 方法获取 RefState
        
        if (stage < N) {
            // 普通节点: yref = [x_ref, u_ref]
            std::vector<double> yref(nx + nu, 0.0);
            for(int j = 0; j < nx; ++j) yref[j] = ref.val[j];
            for(int j = 0; j < nu; ++j) yref[nx + j] = ref.u_ref[j];
            chassis_mpc_solver->set_yref(stage, yref.data());
        } else {
            // 终端节点: 只有状态，没有控制输入
            std::vector<double> yref_e(nx, 0.0);
            for(int j = 0; j < nx; ++j) yref_e[j] = ref.val[j];
            chassis_mpc_solver->set_yref(N, yref_e.data());
        }
    }

    // 3. 执行求解
    int solve_status = chassis_mpc_solver->solve();

    // 4. 提取第 0 步的最优控制指令 (通常是底盘全局速度 [vx, vy, omega])
    std::vector<double> u_opt(nu, 0.0);
    chassis_mpc_solver->get_u(0, u_opt.data());

    // F. 底盘逆运动学分配
    WheelCmd wheel_cmds[NUM_WHEELS]; // 定义接收数组
    // u_opt[0] = vx_body,  u_opt[1] = vy_body,  u_opt[2] = omega
    steering_ik_solver->compute(u_opt[0], u_opt[1], u_opt[2], wheel_cmds, MPC_DT);

    // G. 下发执行器
    for(int i = 0; i < 4; ++i) {
        if(hw_ids.drive_ctrl_idx[i] != -1)
            d->ctrl[hw_ids.drive_ctrl_idx[i]] = wheel_cmds[i].drive_vel; 
            
        if(hw_ids.steer_ctrl_idx[i] != -1)
            d->ctrl[hw_ids.steer_ctrl_idx[i]] = wheel_cmds[i].steer_angle; 
        
        printf("[MPC] Wheel %d: drive_vel=%.3f, steer_angle=%.3f\n", i+1, wheel_cmds[i].drive_vel, wheel_cmds[i].steer_angle);
    }
}

// ==========================================================
// 轨迹渲染函数
// ==========================================================
void render_mpc_trajectory(mjvScene* scn) {
    if (!g_mpc_ui_state.show_trajectory) return;

    std::vector<std::vector<double>> act_traj;
    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        act_traj  = g_act_traj;
    }

    auto draw_path = [&](const std::vector<std::vector<double>>& traj, float r, float g, float b, float a, double width) {
        if (traj.size() < 2) return;
        float rgba[4] = {r, g, b, a};
        for (size_t i = 0; i < traj.size() - 1; ++i) {
            if (scn->ngeom >= scn->maxgeom) break; 
            double pt1[3] = {traj[i][0], traj[i][1], traj[i][2]};
            double pt2[3] = {traj[i+1][0], traj[i+1][1], traj[i+1][2]};

            mjv_initGeom(&scn->geoms[scn->ngeom], mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
            mjv_connector(&scn->geoms[scn->ngeom], mjGEOM_LINE, width, pt1, pt2);
            scn->ngeom++;
        }
    };

    draw_path(act_traj, 0.2f, 0.5f, 1.0f, 0.8f, 5.0); // 实际轨迹蓝线
}