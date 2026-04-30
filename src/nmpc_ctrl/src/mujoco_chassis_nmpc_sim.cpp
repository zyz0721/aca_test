#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>

#include "mpc_solver.h"
#include "mpc_common.h"
#include "reference_generator.h"
#include "steering_ik.h"

#include "ocp_core/OcpProblem.h"
#include "mpc_backend/AcadosWrapper.h"
#include "mpc_backend/ModelCompiler.h"

#include "robot_models/SwerveDynamics.h"
#include "robot_models/SwerveContraints.h"
#include "robot_models/SwerveCost.h"


MPCUIState g_mpc_ui_state;
MpcParams params;

static const mjModel* cached_model = nullptr;

static std::unique_ptr<MPCSolver> chassis_mpc_solver;
static std::unique_ptr<AcadosWrapper> wrapper_;
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
static const double MPC_DT = 0.02; // 50Hz - 0.02s

// 轨迹存储
static std::vector<std::vector<double>> g_ref_traj; // 参考轨迹
static std::vector<std::vector<double>> g_act_traj; // 实际轨迹
static std::vector<std::vector<double>> g_pred_traj; // 预测轨迹
static const size_t MAX_ACT_TRAJ_LEN = 1000;
static const size_t REF_GLOBAL_STEPS = 3000; 

// 四元数转 Yaw 角
static double get_yaw_from_quaternion(double w, double x, double y, double z) {
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 角度归一化到 [-PI, PI]
static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ==========================================================
// 初始化
// ==========================================================
void init_nmpc(const mjModel* m, mjData* d) {
    cached_model = m;
    
    // 绑定 MuJoCo MJCF节点
    int base_jnt = mj_name2id(m, mjOBJ_JOINT, "floating_base"); 
    if(base_jnt != -1) {
        hw_ids.base_qpos_adr = m->jnt_qposadr[base_jnt];
        hw_ids.base_qvel_adr = m->jnt_dofadr[base_jnt];
    } else {
        std::cerr << "[Warning] Could not find floating_base joint" << std::endl;
    }

    std::string drive_names[4] = {"Wheel_1_drive_joint", "Wheel_2_drive_joint", "Wheel_3_drive_joint", "Wheel_4_drive_joint"};
    std::string steer_names[4] = {"Wheel_1_direction_joint", "Wheel_2_direction_joint", "Wheel_3_direction_joint", "Wheel_4_direction_joint"};
    std::string steer_jnt_names[4] = {"Wheel_1_direction_joint", "Wheel_2_direction_joint", "Wheel_3_direction_joint", "Wheel_4_direction_joint"};
    
    for(int i = 0; i < 4; ++i) {
        hw_ids.drive_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, drive_names[i].c_str());
        hw_ids.steer_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, steer_names[i].c_str());
        int steer_jnt = mj_name2id(m, mjOBJ_JOINT, steer_jnt_names[i].c_str());
        if(steer_jnt != -1) hw_ids.steer_qpos_idx[i] = m->jnt_qposadr[steer_jnt];
    }

    std::cout << "FIND MUJOCO JOINT" << std::endl;

    // MPC 参数配置加载（暂用绝对地址）
    std::string config_path = "/home/galbot/galbot_ws/aca_test/src/nmpc_ctrl/config/mpc_config.yaml";
    params.loadFromYaml(config_path);

    // 使用 OcpProblem 构建求解器
    using S = StageSelector;
    using B = BoundConstraintData;

    OcpProblem ocp(params.solver_cfg);
    
    ocp.setDynamics<SwerveDynamics>(params.wheel_base, params.track_width);

    ocp.addBound(S::pathAndTerminal(), B{B::STATE,                 
        {3, 4, 5},
        {-params.max_vel, -params.max_vel, -params.max_yaw_rate},
        { params.max_vel,  params.max_vel,  params.max_yaw_rate}});

    ocp.addBound(S::path(), B{B::INPUT,                            
        {0, 1, 2},
        {-params.max_acc, -params.max_acc, -params.max_yaw_acc},
        { params.max_acc,  params.max_acc,  params.max_yaw_acc}});

    ocp.addNonlinear<SwerveConstraints>(S::pathAndTerminal(),      
        params.wheel_base, params.track_width,
        params.steer_lim_max, params.steer_lim_min);

    ocp.setCost<SwerveCost>(params.solver_cfg.nx, params.solver_cfg.nu);    // 设定跟踪代价函数

    chassis_mpc_solver = ocp.build();
    if (!chassis_mpc_solver) {
        std::cerr << "[Error] MPC Solver build failed!" << std::endl;
        return;
    }
    wrapper_ = ocp.takeWrapper();

    // 初始化其他组件
    ref_generator = std::make_unique<ReferenceGenerator>();
    ref_generator->init(params, params.solver_cfg.nx, params.solver_cfg.nu);

    steering_ik_solver = std::make_unique<SteeringIK>();
    steering_ik_solver->init(params); 

    // 状态与参数的冷启动
    std::vector<double> init_state = params.x0;
    if (init_state.size() < chassis_mpc_solver->get_x_dimension()) {
        init_state.resize(chassis_mpc_solver->get_x_dimension(), 0.0);
    }
    std::vector<double> zero_u(chassis_mpc_solver->get_u_dimension(), 0.0);
    double initial_dir[1] = {1.0};

    for (int i = 0; i <= chassis_mpc_solver->N(); i++) {
        chassis_mpc_solver->init_x(i, init_state.data());
        if (i < chassis_mpc_solver->N()) {
            chassis_mpc_solver->init_u(i, zero_u.data());
        }
        // SwerveConstraints 强依赖 p(0) 这个在线参数，必须给初值！
        chassis_mpc_solver->set_online_parameter(i, initial_dir);
    }

    // 清空历史轨迹
    std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
    g_ref_traj.clear();
    g_act_traj.clear();
    g_pred_traj.clear();

    std::cout << "[NMPC] Chassis NMPC Initialized." << std::endl;
}

// ==========================================================
// 控制回调
// ==========================================================
void nmpc_control_callback(const mjModel* m, mjData* d) {
    if (!g_mpc_ui_state.enable_mpc) return;

    // 重新加载模型时自动重新初始化
    if (m != cached_model) {
        init_nmpc(m, d);
        cached_model = m;
    }

    if (!chassis_mpc_solver) return;

    double current_time = d->time;

    // 频率控制
    if (current_time - last_mpc_time < MPC_DT) {
        return; 
    }
    last_mpc_time = current_time;

    // 1. 提取当前底盘状态
    // 包括世界坐标系下位置(x,y)和航向角(yaw)，以及体坐标系下速度(vx, vy)和航向角速度(omega)
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

    // 暂时未用
    std::vector<double> current_steer_angles(4, 0.0);
    for(int i=0; i<4; ++i) {
        if(hw_ids.steer_qpos_idx[i] != -1) {
            current_steer_angles[i] = d->qpos[hw_ids.steer_qpos_idx[i]];
        }
    }

    // DEBUG 打印状态提取和MPC解算结果
    static int dbg_count = 0;
    dbg_count++;
    // if (dbg_count % 50 == 0) {
    //         printf("[NMPC DEBUG] current steer angles: ");
    //         for(int i = 0; i < 4; i++) {
    //             printf("W%d[steer angle=%.3f] ", i, current_steer_angles[i]);
    //         }
    //         printf("\n");
    // }

    // 2. 解算 MPC
    int N = chassis_mpc_solver->N();
    int nx = chassis_mpc_solver->get_x_dimension();
    int nu = chassis_mpc_solver->get_u_dimension();

    // auto start = std::chrono::high_resolution_clock::now();
    chassis_mpc_solver->set_x0(current_state.data());

    double vx_global_ref = ref_generator->at(current_time).val[3];
    double vy_global_ref = ref_generator->at(current_time).val[4];
    double v_body_x = vx_global_ref * std::cos(current_state[2]) + vy_global_ref * std::sin(current_state[2]);
    double current_dir = (v_body_x < -1e-2) ? -1.0 : 1.0;
    double p_val[1] = { current_dir };

    // 循环遍历预测视野 N，设置每个 stage 的参考轨迹和参数
    for (int stage = 0; stage <= N; stage++) {
        double pred_time = current_time + stage * MPC_DT;
        auto ref = ref_generator->at(pred_time); 
        
        // 处理 yaw 连续性
        double dyaw = normalize_angle(ref.val[2] - current_state[2]);
        ref.val[2] = current_state[2] + dyaw;

        // 设置在线参数，约束需要
        chassis_mpc_solver->set_online_parameter(stage, p_val);

        if (stage < N) {
            std::vector<double> yref(nx + nu, 0.0);
            for(int j = 0; j < nx; ++j) yref[j] = ref.val[j];
            for(int j = 0; j < nu; ++j) yref[nx + j] = ref.u_ref[j];
            chassis_mpc_solver->set_yref(stage, yref.data());
        } else {
            std::vector<double> yref_e(nx, 0.0);
            for(int j = 0; j < nx; ++j) yref_e[j] = ref.val[j];
            chassis_mpc_solver->set_yref(N, yref_e.data());
        }
    }

    // 3. 求解
    auto start = std::chrono::high_resolution_clock::now();
    int solve_status = chassis_mpc_solver->solve();
    auto end = std::chrono::high_resolution_clock::now();
    int sqp_iter = chassis_mpc_solver->get_sqp_iter();
    double solve_time = chassis_mpc_solver->get_solve_time();
    std::chrono::duration<double, std::micro> duration = end - start;
    if (dbg_count % 50 == 0) {
        std::cout << "[NMPC] Prediction steps: " << N << std::endl;
        // std::cout << "[NMPC] Total solve time: " << duration.count() << " us" << std::endl;
        std::cout << "[NMPC] SQP iterations: " << sqp_iter << std::endl;
        // std::cout << "[NMPC] Average time per step: " << duration.count() / N << " us" << std::endl;
        std::cout << "[NMPC] Total solve time: " << 1000 * solve_time << " ms" << std::endl;
    }

    // 提取预测位置和全局参考位置，在仿真中渲染轨迹
    if (g_mpc_ui_state.show_trajectory && solve_status == 0) {
        std::vector<std::vector<double>> current_pred_traj;
        std::vector<std::vector<double>> current_ref_traj;

        // 提取 MPC 预测的状态坐标
        for (int i = 0; i <= N; i++) {
            std::vector<double> pred_x(nx, 0.0);
            chassis_mpc_solver->get_x(i, pred_x.data());
            current_pred_traj.push_back({pred_x[0], pred_x[1], 0.12}); 
        }

        for (int i = 0; i < REF_GLOBAL_STEPS; i++) {
            auto ref = ref_generator->at(current_time + i * MPC_DT);
            current_ref_traj.push_back({ref.val[0], ref.val[1], 0.09}); // Z轴稍微低一点点
        }

        // 实际轨迹
        if (hw_ids.base_qpos_adr != -1) {
            double current_x = d->qpos[hw_ids.base_qpos_adr];
            double current_y = d->qpos[hw_ids.base_qpos_adr + 1];
            g_act_traj.push_back({current_x, current_y, 0.12});
            if (g_act_traj.size() > MAX_ACT_TRAJ_LEN) g_act_traj.erase(g_act_traj.begin());
        }

        // 传入全局绘图变量
        {
            std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
            g_pred_traj = current_pred_traj;
            g_ref_traj = current_ref_traj;
        }
    }

    // 4. 提取首步的最优预测状态作为目标执行
    std::vector<double> x1(nx, 0.0);
    chassis_mpc_solver->get_x(1, x1.data());

    // 提取出全局目标速度
    double vx_global_target = x1[3];
    double vy_global_target = x1[4];
    double dyaw_target      = x1[5];

    // if (dbg_count % 50 == 0) {
    //     printf("[NMPC DEBUG] current_state: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
    //            current_state[0], current_state[1], current_state[2],
    //            current_state[3], current_state[4], current_state[5]);
    //     printf("[NMPC DEBUG] x1 (next target): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
    //            x1[0], x1[1], x1[2], x1[3], x1[4], x1[5]);
    //     printf("[NMPC DEBUG] Global target vel: vx=%.3f vy=%.3f dyaw=%.3f\n",
    //            vx_global_target, vy_global_target, dyaw_target);
    // }

    // 将全局速度转为体坐标系速度
    double vx_body, vy_body;
    double c = cos(current_state[2]), s = sin(current_state[2]);
    vx_body =  c * vx_global_target + s * vy_global_target;
    vy_body = -s * vx_global_target + c * vy_global_target;

    // if (dbg_count % 50 == 0) {
    //     printf("[NMPC DEBUG] Body target vel: vx_body=%.3f vy_body=%.3f (yaw=%.3f)\n",
    //            vx_body, vy_body, current_state[2]);
    // }

    // G. 底盘逆运动学分配
    WheelCmd wheel_cmds[NUM_WHEELS]; 
    if (solve_status == 0) {
        steering_ik_solver->compute(vx_body, vy_body, dyaw_target, wheel_cmds, MPC_DT);
        // if (dbg_count % 50 == 0) {
        //     printf("[NMPC DEBUG] Wheel commands: ");
        //     for(int i = 0; i < 4; i++) {
        //         printf("W%d[steer=%.3f,vel=%.3f] ", i, wheel_cmds[i].steer_angle, wheel_cmds[i].drive_vel);
        //     }
        //     printf("\n");
        // }
    } else {
        // 求解失败安全停止
        steering_ik_solver->compute(0.0, 0.0, 0.0, wheel_cmds, MPC_DT);
        printf("[MPC] Warning: Solve failed with status %d\n", solve_status);
    }

    // 下发执行器
    for(int i = 0; i < 4; ++i) {
        if(hw_ids.drive_ctrl_idx[i] != -1)
            d->ctrl[hw_ids.drive_ctrl_idx[i]] = wheel_cmds[i].drive_omega; // 转为轮子角速度
            
        if(hw_ids.steer_ctrl_idx[i] != -1)
            d->ctrl[hw_ids.steer_ctrl_idx[i]] = wheel_cmds[i].steer_angle; 
    }
}

// ==========================================================
// 轨迹渲染函数
// ==========================================================
void render_mpc_trajectory(mjvScene* scn) {
    if (!g_mpc_ui_state.show_trajectory) return;

    std::vector<std::vector<double>> act_traj;
    std::vector<std::vector<double>> pred_traj;
    std::vector<std::vector<double>> ref_traj;
    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        pred_traj = g_pred_traj;
        ref_traj  = g_ref_traj;
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

    draw_path(act_traj, 0.0f, 0.5f, 1.0f, 1.0f, 5.0);     // 实际轨迹
    draw_path(ref_traj, 0.0f, 1.0f, 0.0f, 1.0f, 5.0);     // 参考轨迹
    draw_path(pred_traj, 1.0f, 0.0f, 0.0f, 1.0f, 20.0);     // 预测轨迹
}