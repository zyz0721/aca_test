#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>

#include "ocp_core/OcpProblem.h"
#include "mpc_backend/AcadosWrapper.h"
#include "mpc_backend/ModelCompiler.h"
#include "mpc_common.h"
#include "reference_generator.h"
#include "steering_ik.h"

#include "robot_models/SwerveDynamics.h"
#include "robot_models/ChassisFullConstraints.h"
#include "robot_models/ChassisFullCost.h"

// 全局变量
MPCUIState g_mpc_ui_state;
MpcParams full_params;

static const mjModel* cached_model = nullptr;
static std::unique_ptr<MPCSolver> full_mpc_solver;
static std::unique_ptr<AcadosWrapper> full_wrapper_;
static std::unique_ptr<ReferenceGenerator> ref_generator;
static std::unique_ptr<SteeringIK> steering_ik_solver;

// 硬件索引
struct ChassisHWIDs {
    int base_qpos_adr = -1;
    int base_qvel_adr = -1;
    std::vector<int> drive_ctrl_idx;
    std::vector<int> steer_ctrl_idx;
    std::vector<int> steer_qpos_idx;
    ChassisHWIDs() : drive_ctrl_idx(4, -1), steer_ctrl_idx(4, -1), steer_qpos_idx(4, -1) {}
};
static ChassisHWIDs hw_ids;

// MPC频率控制
static double last_mpc_time = -1.0;
static const double MPC_DT = 0.02;

// 轨迹存储
static std::vector<std::vector<double>> g_ref_traj;
static std::vector<std::vector<double>> g_act_traj;
static std::vector<std::vector<double>> g_pred_traj;
static const size_t MAX_ACT_TRAJ_LEN = 1000;
static const size_t REF_GLOBAL_STEPS = 3000;

// 辅助函数
static double get_yaw_from_quaternion(double w, double x, double y, double z) {
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

static double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ============================================================
// Part 2: NMPC初始化函数 - 构建完整OCP问题
// ============================================================

void init_nmpc(const mjModel* m, mjData* d) {
    cached_model = m;

    // 绑定 MuJoCo MJCF节点
    int base_jnt = mj_name2id(m, mjOBJ_JOINT, "floating_base");
    if (base_jnt != -1) {
        hw_ids.base_qpos_adr = m->jnt_qposadr[base_jnt];
        hw_ids.base_qvel_adr = m->jnt_dofadr[base_jnt];
    } else {
        std::cerr << "[Warning] Could not find floating_base joint" << std::endl;
    }

    std::string drive_names[4] = {"Wheel_1_drive_joint", "Wheel_2_drive_joint",
                                  "Wheel_3_drive_joint", "Wheel_4_drive_joint"};
    std::string steer_names[4] = {"Wheel_1_direction_joint", "Wheel_2_direction_joint",
                                  "Wheel_3_direction_joint", "Wheel_4_direction_joint"};

    for (int i = 0; i < 4; ++i) {
        hw_ids.drive_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, drive_names[i].c_str());
        hw_ids.steer_ctrl_idx[i] = mj_name2id(m, mjOBJ_ACTUATOR, steer_names[i].c_str());
        int steer_jnt = mj_name2id(m, mjOBJ_JOINT, steer_names[i].c_str());
        if (steer_jnt != -1) hw_ids.steer_qpos_idx[i] = m->jnt_qposadr[steer_jnt];
    }

    std::cout << "[FullNMPC] MuJoCo joints bound." << std::endl;

    // 加载完整配置
    std::string config_path = "/home/galbot/galbot_ws/aca_test/src/nmpc_ctrl/config/nmpc_chassis_full_config.yaml";
    full_params.loadFromYaml(config_path);

    // ============================================================
    // 构建 OCP 问题 (参考 steer_nmpc.cpp setup_optimization)
    // ============================================================
    using S = StageSelector;
    using B = BoundConstraintData;

    OcpProblem ocp(full_params.solver_cfg);

    // 1. 设置动力学 (参考 SwerveDynamics)
    ocp.setDynamics<SwerveDynamics>(full_params.wheel_base, full_params.track_width);

    int nx = full_params.solver_cfg.nx;  // 6
    int nu = full_params.solver_cfg.nu;  // 3

    // 2. 初始状态约束 (X(:,0) == current_state)
    std::vector<int> all_idx(nx);
    for (int i = 0; i < nx; i++) all_idx[i] = i;
    std::vector<double> lo_init(nx, 0.0), hi_init(nx, 0.0);  // set_x0会覆写
    ocp.addBound(S::initial(), B{B::STATE, all_idx, lo_init, hi_init});

    // 3. 速度约束 (圆形约束: ||vx,vy||^2 <= max_vel^2)
    // acados不支持圆形约束，改用分解的矩形约束
    // 这里用边界约束近似
    ocp.addBound(S::pathAndTerminal(), B{B::STATE,
        {3, 4, 5},
        {-full_params.max_vel, -full_params.max_vel, -full_params.max_yaw_rate},
        {full_params.max_vel, full_params.max_vel, full_params.max_yaw_rate}});

    // 4. 加速度约束 (圆形约束: ||ax,ay||^2 <= max_acc^2)
    // 同样用矩形约束近似
    ocp.addBound(S::path(), B{B::INPUT,
        {0, 1, 2},
        {-full_params.max_acc, -full_params.max_acc, -full_params.max_yaw_acc},
        {full_params.max_acc, full_params.max_acc, full_params.max_yaw_acc}});

    // 5. 舵角投影约束 (扇形约束)
    // 参考 steer_nmpc.cpp 第114-124行
    ocp.addNonlinear<SteerProjectionConstraint>(S::pathAndTerminal(),
        full_params.wheel_base, full_params.track_width,
        full_params.steer_lim_max, full_params.steer_lim_min);

    // 6. 舵角变化率软约束 (参考 steer_nmpc.cpp 第127-137行)
    // 注意: 需要跨时刻约束，此处用软约束框架处理
    // 可选: 添加SteerRateSoftConstraint
    // ocp.addNonlinear<SteerRateSoftConstraint>(S::path(),
    //     SoftPenalty{true, full_params.soft_Z, full_params.soft_z},
    //     full_params.wheel_base, full_params.track_width,
    //     full_params.max_steer_rate, MPC_DT);

    // 7. 设置代价函数
    ocp.setCost<ChassisFullCost>(nx, nu);

    // 8. 构建
    full_mpc_solver = ocp.build();
    if (!full_mpc_solver) {
        std::cerr << "[Error] Full MPC Solver build failed!" << std::endl;
        return;
    }
    full_wrapper_ = ocp.takeWrapper();

    // 9. 初始化参考轨迹生成器和逆运动学
    ref_generator = std::make_unique<ReferenceGenerator>();
    // 转换为MpcParams格式用于ReferenceGenerator
    MpcParams ref_params;
    ref_params.ref_type = full_params.ref_type;
    ref_params.circle_r = full_params.circle_r;
    ref_params.circle_v = full_params.circle_v;
    ref_params.line_vx = full_params.line_vx;
    ref_params.fig8_a = full_params.fig8_a;
    ref_params.fig8_omega = full_params.fig8_omega;
    ref_params.solver_cfg = full_params.solver_cfg;
    ref_generator->init(ref_params, nx, nu);

    steering_ik_solver = std::make_unique<SteeringIK>();
    steering_ik_solver->init(ref_params);

    // 10. 冷启动初始化
    std::vector<double> init_state = full_params.x0;
    if (init_state.size() < nx) init_state.resize(nx, 0.0);
    std::vector<double> zero_u(nu, 0.0);
    double initial_dir[1] = {1.0};

    for (int i = 0; i <= full_mpc_solver->N(); i++) {
        full_mpc_solver->init_x(i, init_state.data());
        if (i < full_mpc_solver->N()) {
            full_mpc_solver->init_u(i, zero_u.data());
        }
        full_mpc_solver->set_online_parameter(i, initial_dir);
    }

    // 清空轨迹
    std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
    g_ref_traj.clear();
    g_act_traj.clear();
    g_pred_traj.clear();

    std::cout << "[FullNMPC] Initialized with N=" << full_params.solver_cfg.N
              << ", dt=" << MPC_DT << "s" << std::endl;
}

// ============================================================
// Part 3: 控制回调函数 - 状态提取部分
// ============================================================

// 提取当前底盘状态
// 包括: 世界坐标系位置(x,y), 航向角(yaw), 体坐标系速度(vx,vy), 角速度(omega)
static std::vector<double> extract_current_state(const mjModel* m, mjData* d) {
    std::vector<double> current_state(6, 0.0);

    if (hw_ids.base_qpos_adr != -1) {
        // 位置 (世界坐标系)
        current_state[0] = d->qpos[hw_ids.base_qpos_adr];      // x
        current_state[1] = d->qpos[hw_ids.base_qpos_adr + 1];  // y
        // 航向角 (从四元数提取)
        current_state[2] = get_yaw_from_quaternion(
            d->qpos[hw_ids.base_qpos_adr + 3],  // qw
            d->qpos[hw_ids.base_qpos_adr + 4],  // qx
            d->qpos[hw_ids.base_qpos_adr + 5],  // qy
            d->qpos[hw_ids.base_qpos_adr + 6]); // qz
    }

    if (hw_ids.base_qvel_adr != -1) {
        // 速度 (世界坐标系)
        current_state[3] = d->qvel[hw_ids.base_qvel_adr];      // vx_global
        current_state[4] = d->qvel[hw_ids.base_qvel_adr + 1];  // vy_global
        current_state[5] = d->qvel[hw_ids.base_qvel_adr + 5];  // dyaw
    }

    return current_state;
}

// 提取当前舵轮角度
static std::vector<double> extract_steer_angles(const mjModel* m, mjData* d) {
    std::vector<double> steer_angles(4, 0.0);
    for (int i = 0; i < 4; ++i) {
        if (hw_ids.steer_qpos_idx[i] != -1) {
            steer_angles[i] = d->qpos[hw_ids.steer_qpos_idx[i]];
        }
    }
    return steer_angles;
}

// 计算运动方向 (参考 steer_nmpc.cpp compute_forward_direction)
static int compute_motion_direction(const std::vector<double>& current_state,
                                    const RefState& ref_state) {
    double yaw = current_state[2];
    double vx_ref_global = ref_state.val[3];
    double vy_ref_global = ref_state.val[4];

    // 投影到体坐标系
    double v_body_x = vx_ref_global * std::cos(yaw) + vy_ref_global * std::sin(yaw);

    double v_eps = 0.01;  // 速度阈值
    if (v_body_x > v_eps) return 1;   // 前进
    if (v_body_x < -v_eps) return -1; // 后退
    return 1;  // 默认前进
}

// ============================================================
// Part 4: 控制回调函数 - MPC求解部分
// ============================================================

// 设置参考轨迹和在线参数
static void setup_mpc_references(double current_time, const std::vector<double>& current_state) {
    int N = full_mpc_solver->N();
    int nx = full_mpc_solver->get_x_dimension();
    int nu = full_mpc_solver->get_u_dimension();

    // 设置初始状态 (需要const_cast因为接口是非const指针)
    std::vector<double> state_copy = current_state;
    full_mpc_solver->set_x0(state_copy.data());

    // 计算当前运动方向
    auto ref_curr = ref_generator->at(current_time);
    int current_dir = compute_motion_direction(current_state, ref_curr);
    double p_val[1] = { static_cast<double>(current_dir) };

    // 循环设置每个 stage 的参考轨迹和参数
    for (int stage = 0; stage <= N; stage++) {
        double pred_time = current_time + stage * MPC_DT;
        auto ref = ref_generator->at(pred_time);

        // 处理 yaw 连续性 (参考 steer_nmpc.cpp 第243-245行)
        double dyaw = normalize_angle(ref.val[2] - current_state[2]);
        ref.val[2] = current_state[2] + dyaw;

        // 设置在线参数 (运动方向)
        full_mpc_solver->set_online_parameter(stage, p_val);

        // 设置参考轨迹
        if (stage < N) {
            // 阶段代价: yref = [x; u] (nx + nu)
            std::vector<double> yref(nx + nu, 0.0);
            for (int j = 0; j < nx; ++j) yref[j] = ref.val[j];
            for (int j = 0; j < nu; ++j) yref[nx + j] = ref.u_ref[j];
            full_mpc_solver->set_yref(stage, yref.data());
        } else {
            // 终端代价: yref_e = x (nx)
            std::vector<double> yref_e(nx, 0.0);
            for (int j = 0; j < nx; ++j) yref_e[j] = ref.val[j];
            full_mpc_solver->set_yref(N, yref_e.data());
        }
    }
}

// MPC求解并提取结果
static int solve_mpc(std::vector<double>& x1_out, double& solve_time_ms) {
    auto start = std::chrono::high_resolution_clock::now();
    int solve_status = full_mpc_solver->solve();
    auto end = std::chrono::high_resolution_clock::now();

    solve_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (solve_status == 0) {
        // 提取首步最优预测状态
        int nx = full_mpc_solver->get_x_dimension();
        x1_out.resize(nx, 0.0);
        full_mpc_solver->get_x(1, x1_out.data());
    }

    return solve_status;
}

// 提取预测轨迹用于渲染
static void extract_prediction_trajectory(std::vector<std::vector<double>>& pred_traj,
                                          std::vector<std::vector<double>>& ref_traj_out,
                                          double current_time, const std::vector<double>& current_state) {
    int N = full_mpc_solver->N();
    int nx = full_mpc_solver->get_x_dimension();

    pred_traj.clear();
    ref_traj_out.clear();

    // MPC预测轨迹
    for (int i = 0; i <= N; i++) {
        std::vector<double> pred_x(nx, 0.0);
        full_mpc_solver->get_x(i, pred_x.data());
        pred_traj.push_back({pred_x[0], pred_x[1], 0.12});  // z=0.12用于渲染
    }

    // 全局参考轨迹
    for (int i = 0; i < REF_GLOBAL_STEPS; i++) {
        auto ref = ref_generator->at(current_time + i * MPC_DT);
        ref_traj_out.push_back({ref.val[0], ref.val[1], 0.09});
    }
}

// ============================================================
// Part 5: 控制回调函数 - 逆运动学和执行部分
// ============================================================

// 将全局速度转换为体坐标系速度
static void global_to_body_velocity(const std::vector<double>& x1,
                                    const std::vector<double>& current_state,
                                    double& vx_body, double& vy_body, double& dyaw_target) {
    // 提取全局目标速度
    double vx_global_target = x1[3];
    double vy_global_target = x1[4];
    dyaw_target = x1[5];

    // 坐标变换: 全局 -> 体坐标系
    double yaw = current_state[2];
    double c = std::cos(yaw), s = std::sin(yaw);
    vx_body = c * vx_global_target + s * vy_global_target;
    vy_body = -s * vx_global_target + c * vy_global_target;
}

// 执行逆运动学解算并下发控制指令
static void execute_wheel_control(double vx_body, double vy_body, double dyaw_target,
                                  mjData* d, int solve_status) {
    WheelCmd wheel_cmds[NUM_WHEELS];

    if (solve_status == 0) {
        // MPC求解成功, 执行逆运动学
        steering_ik_solver->compute(vx_body, vy_body, dyaw_target, wheel_cmds, MPC_DT);
    } else {
        // 求解失败, 安全停止
        steering_ik_solver->compute(0.0, 0.0, 0.0, wheel_cmds, MPC_DT);
        std::printf("[FullNMPC] Warning: Solve failed with status %d\n", solve_status);
    }

    // 下发执行器指令
    for (int i = 0; i < 4; ++i) {
        if (hw_ids.drive_ctrl_idx[i] != -1) {
            d->ctrl[hw_ids.drive_ctrl_idx[i]] = wheel_cmds[i].drive_omega;
        }
        if (hw_ids.steer_ctrl_idx[i] != -1) {
            d->ctrl[hw_ids.steer_ctrl_idx[i]] = wheel_cmds[i].steer_angle;
        }
    }
}

// 更新轨迹存储
static void update_trajectory_storage(const std::vector<double>& current_state) {
    // 添加实际轨迹点
    if (hw_ids.base_qpos_adr != -1) {
        double x = current_state[0];
        double y = current_state[1];
        g_act_traj.push_back({x, y, 0.12});
        if (g_act_traj.size() > MAX_ACT_TRAJ_LEN) {
            g_act_traj.erase(g_act_traj.begin());
        }
    }
}

// ============================================================
// Part 6: 主控制回调函数和轨迹渲染
// ============================================================

// 主控制回调函数 (整合Part3-5)
void nmpc_control_callback(const mjModel* m, mjData* d) {
    if (!g_mpc_ui_state.enable_mpc) return;

    // 模型重新加载时自动重新初始化
    if (m != cached_model) {
        init_nmpc(m, d);
        cached_model = m;
    }

    if (!full_mpc_solver) return;

    double current_time = d->time;

    // 频率控制
    if (current_time - last_mpc_time < MPC_DT) {
        return;
    }
    last_mpc_time = current_time;

    // ========== Part 3: 状态提取 ==========
    std::vector<double> current_state = extract_current_state(m, d);
    std::vector<double> current_steer_angles = extract_steer_angles(m, d);

    // ========== Part 4: MPC求解 ==========
    int nx = full_mpc_solver->get_x_dimension();
    int nu = full_mpc_solver->get_u_dimension();

    // 设置参考轨迹和参数
    setup_mpc_references(current_time, current_state);

    // 求解MPC
    std::vector<double> x1;
    double solve_time_ms = 0.0;
    int solve_status = solve_mpc(x1, solve_time_ms);

    // 获取求解统计
    int sqp_iter = full_mpc_solver->get_sqp_iter();

    // 周期性打印求解信息
    static int dbg_count = 0;
    dbg_count++;
    if (dbg_count % 50 == 0) {
        std::cout << "[FullNMPC] N=" << full_mpc_solver->N()
                  << ", SQP_iter=" << sqp_iter
                  << ", solve_time=" << solve_time_ms << "ms"
                  << ", status=" << solve_status << std::endl;
    }

    // 提取预测轨迹用于渲染
    if (g_mpc_ui_state.show_trajectory && solve_status == 0) {
        std::vector<std::vector<double>> current_pred_traj;
        std::vector<std::vector<double>> current_ref_traj;
        extract_prediction_trajectory(current_pred_traj, current_ref_traj, current_time, current_state);

        // 更新实际轨迹
        update_trajectory_storage(current_state);

        // 传入全局绘图变量
        {
            std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
            g_pred_traj = current_pred_traj;
            g_ref_traj = current_ref_traj;
        }
    }

    // ========== Part 5: 逆运动学和执行 ==========
    double vx_body = 0.0, vy_body = 0.0, dyaw_target = 0.0;

    if (solve_status == 0 && x1.size() >= 6) {
        global_to_body_velocity(x1, current_state, vx_body, vy_body, dyaw_target);
    } else {
        // 求解失败时使用当前状态的速度作为目标（保持当前速度）
        vx_body = current_state[3] * std::cos(current_state[2]) + current_state[4] * std::sin(current_state[2]);
        vy_body = -current_state[3] * std::sin(current_state[2]) + current_state[4] * std::cos(current_state[2]);
        dyaw_target = current_state[5];
    }

    // 执行轮控
    execute_wheel_control(vx_body, vy_body, dyaw_target, d, solve_status);
}

// 轨迹渲染函数
void render_mpc_trajectory(mjvScene* scn) {
    if (!g_mpc_ui_state.show_trajectory) return;

    std::vector<std::vector<double>> act_traj;
    std::vector<std::vector<double>> pred_traj;
    std::vector<std::vector<double>> ref_traj;

    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        pred_traj = g_pred_traj;
        ref_traj = g_ref_traj;
        act_traj = g_act_traj;
    }

    // 绘制路径的lambda函数
    auto draw_path = [&](const std::vector<std::vector<double>>& traj,
                         float r, float g, float b, float a, double width) {
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

    // 绘制三条轨迹
    draw_path(act_traj, 0.0f, 0.5f, 1.0f, 1.0f, 5.0);   // 实际轨迹: 蓝色
    draw_path(ref_traj, 0.0f, 1.0f, 0.0f, 1.0f, 5.0);   // 参考轨迹: 绿色
    draw_path(pred_traj, 1.0f, 0.0f, 0.0f, 1.0f, 20.0); // 预测轨迹: 红色
}