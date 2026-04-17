#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <mutex>

// 引入 NMPC 框架头文件 
#include "ocp_core/OcpProblem.h"
#include "robot_models/ArmDynamics.h"

// 全局静态变量管理
static const mjModel* cached_model = nullptr;
static std::unique_ptr<MPCSolver> mpc_solver;
static std::unique_ptr<AcadosWrapper> mpc_wrapper;
static mjData* d_tmp = nullptr; // 用于运动学正解的临时数据结构

// 存储硬件在 MuJoCo 内存数组中的索引
struct HardwareIDs {
    int left_qpos_adr[7];
    int left_ctrl_id[7];
    int right_ctrl_id[7];
};
static HardwareIDs hw_ids;
static int ee_body_id = -1;

// MPC 运行频率控制
static double last_mpc_time = -1.0;
static const double MPC_DT = 0.02; // 50Hz = 0.02s
static std::vector<double> last_cmd_q(7, 0.0); // 缓存上一次的控制目标

// 新增轨迹存储
static std::vector<std::vector<double>> g_ref_traj; // 参考轨迹
static std::vector<std::vector<double>> g_act_traj; // 实际轨迹
static const size_t MAX_ACT_TRAJ_LEN = 300;         // 实际轨迹保留长度

MPCUIState g_mpc_ui_state;

// ======================= 轨迹参考生成器 =======================

// 辅助函数：获取特定时间的参考关节角位置
static std::vector<double> get_reference_q(double t) {
    std::vector<double> q = {0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0};
    q[0] = 0.5 * sin(1.0 * t);           
    q[1] = -1.0 - 0.2 * cos(1.0 * t);    
    q[3] = -1.57 + 0.8 * sin(1.0 * t);   
    return q;
}

// 辅助函数：获取特定时间的参考关节角速度
static std::vector<double> get_reference_dq(double t) {
    std::vector<double> dq(7, 0.0);
    dq[0] = 0.5 * 1.0 * cos(1.0 * t);
    dq[1] = 0.5 * 1.0 * sin(1.0 * t);
    dq[3] = 0.8 * 1.0 * cos(1.0 * t);
    return dq;
}

// ======================= 初始化 NMPC =======================
void init_nmpc(const mjModel* m, mjData* d) {
    std::cout << "\n[NMPC] Initializing Acados/CasADi Solver for MuJoCo..." << std::endl;

    // 清理旧的临时数据，重建新的辅助计算结构
    if (d_tmp) {
        mj_deleteData(d_tmp);
        d_tmp = nullptr;
    }
    d_tmp = mj_makeData(m);

    // 1. 获取模型中左右臂的内存地址 ID
    for (int i = 0; i < 7; ++i) {
        std::string left_name = "left_arm_joint" + std::to_string(i+1);
        std::string right_name = "right_arm_joint" + std::to_string(i+1);

        int left_jnt_id = mj_name2id(m, mjOBJ_JOINT, left_name.c_str());
        hw_ids.left_qpos_adr[i] = (left_jnt_id >= 0) ? m->jnt_qposadr[left_jnt_id] : -1;

        hw_ids.left_ctrl_id[i]  = mj_name2id(m, mjOBJ_ACTUATOR, left_name.c_str());
        hw_ids.right_ctrl_id[i] = mj_name2id(m, mjOBJ_ACTUATOR, right_name.c_str());
    }

    ee_body_id = mj_name2id(m, mjOBJ_BODY, "left_arm_end_effector_mount_link");

    // 2. 配置求解器
    SolverConfig cfg;
    cfg.nx = 7; cfg.nu = 7; cfg.N = 20; cfg.T = 1.0; cfg.hz = 50.0;
    // 权重：优先逼近目标角度 (Q)，同时限制速度不要太大 (R)
    cfg.W = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,  
              0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1}; 

    // 3. 构建 OCP 问题
    using S = StageSelector;
    using B = BoundConstraintData;
    OcpProblem ocp(cfg);
    ocp.setDynamics<ArmDynamics>();
    
    std::vector<int> all_idx = {0,1,2,3,4,5,6};
    std::vector<double> min_q(7, -3.14), max_q(7, 3.14);
    std::vector<double> min_dq(7, -1.0), max_dq(7, 1.0);
    
    ocp.addBound(S::pathAndTerminal(), B{B::STATE, all_idx, min_q, max_q});
    ocp.addBound(S::path(), B{B::INPUT, all_idx, min_dq, max_dq});
    
    // 4. 生成求解器
    mpc_solver = ocp.build();
    if (mpc_solver) {
        mpc_wrapper = ocp.takeWrapper(); // 拿走所有权防止被析构
        std::cout << "[NMPC] Solver built successfully!" << std::endl;
    } else {
        std::cerr << "[NMPC ERROR] Failed to build Acados solver!" << std::endl;
    }

    // 状态初始化
    last_mpc_time = d->time;
    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        g_act_traj.clear();
        g_ref_traj.clear();
        g_mpc_ui_state.predicted_ee_pos.clear();
    }
}

// 实时控制回调 每次 mj_step 会调用此函数
void nmpc_control_callback(const mjModel* m, mjData* d) {
    // UI 开关判断
    if (!g_mpc_ui_state.enable_mpc) {
        return; // UI 关闭 MPC 时直接返回
    }
    
    // 重新加载模型时自动重新初始化
    if (m != cached_model) {
        init_nmpc(m, d);
        cached_model = m;
    }

    if (!mpc_solver) return;

    // 1. 50Hz 运行一次 MPC 求解
    if (d->time - last_mpc_time >= MPC_DT) {
        last_mpc_time += MPC_DT; 
        double t = d->time;

        // --- A. 记录当前实际的末端位置 (用于绘制蓝色实际历史轨迹) ---
        if (ee_body_id >= 0 && g_mpc_ui_state.show_trajectory) {
            std::vector<double> current_ee_pos = {
                d->xpos[3 * ee_body_id + 0],
                d->xpos[3 * ee_body_id + 1],
                d->xpos[3 * ee_body_id + 2]
            };
            std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
            g_act_traj.push_back(current_ee_pos);
            if (g_act_traj.size() > MAX_ACT_TRAJ_LEN) {
                g_act_traj.erase(g_act_traj.begin()); // 保持队列长度
            }
        }

        // B. 从 MuJoCo 读取当前状态
        std::vector<double> current_q(7, 0.0);
        for (int i = 0; i < 7; ++i) {
            if (hw_ids.left_qpos_adr[i] >= 0) {
                current_q[i] = d->qpos[hw_ids.left_qpos_adr[i]];
            }
        }

        // C. 送入求解器求解 
        mpc_solver->set_x0(current_q.data());
        
        int N = 20; // cfg.N
        // 分步计算未来 N 步的参考目标并注入求解器
        for (int i = 0; i < N; i++) {
            auto step_q = get_reference_q(t + i * MPC_DT);
            auto step_dq = get_reference_dq(t + i * MPC_DT);
            std::vector<double> yref(14);
            std::copy(step_q.begin(), step_q.end(), yref.begin());
            std::copy(step_dq.begin(), step_dq.end(), yref.begin() + 7);
            mpc_solver->set_yref(i, yref.data());
        }
        auto term_q = get_reference_q(t + N * MPC_DT);
        mpc_solver->set_yref(N, term_q.data()); // 终端参考点

        if (mpc_solver->solve() == 0) {
            // 解析出下个时刻的期望状态作为控制位置指令
            std::vector<double> next_q(7);
            mpc_solver->get_x(1, next_q.data()); 
            
            for (int i = 0; i < 7; ++i) {
                last_cmd_q[i] = next_q[i]; // 更新缓存
            }

            // --- D. 提取未来预测位置和完整的全局参考位置，用于画线 ---
            if (g_mpc_ui_state.show_trajectory && ee_body_id >= 0 && d_tmp) {
                std::vector<std::vector<double>> current_pred_traj;
                std::vector<std::vector<double>> current_ref_traj;

                // (1) 获取预测状态的正解 (红色线，仅显示 N=20 步)
                int render_N = std::min(g_mpc_ui_state.horizon_steps, N); 
                for (int i = 0; i < render_N; i++) {
                    std::vector<double> pred_q(7);
                    mpc_solver->get_x(i, pred_q.data()); 

                    mju_copy(d_tmp->qpos, d->qpos, m->nq);
                    for(int j=0; j<7; j++) {
                        if (hw_ids.left_qpos_adr[j] >= 0)
                            d_tmp->qpos[hw_ids.left_qpos_adr[j]] = pred_q[j];
                    }
                    mj_kinematics(m, d_tmp); 
                    current_pred_traj.push_back({
                        d_tmp->xpos[3 * ee_body_id + 0],
                        d_tmp->xpos[3 * ee_body_id + 1],
                        d_tmp->xpos[3 * ee_body_id + 2]
                    });
                }

                // (2) 获取完整的期望参考轨迹 (绿色线)
                // 从当前时间 t 开始，往后预测一个完整的周期 (大约 2*pi ≈ 6.28秒，除以 0.02s 约 315步)
                // 这样你在画面中就会看到一个完整的期望"赛道"
                int REF_GLOBAL_STEPS = 315; 
                for (int i = 0; i < REF_GLOBAL_STEPS; i++) {
                    auto ref_q = get_reference_q(t + i * MPC_DT);
                    mju_copy(d_tmp->qpos, d->qpos, m->nq);
                    for(int j=0; j<7; j++) {
                        if (hw_ids.left_qpos_adr[j] >= 0)
                            d_tmp->qpos[hw_ids.left_qpos_adr[j]] = ref_q[j];
                    }
                    mj_kinematics(m, d_tmp);
                    current_ref_traj.push_back({
                        d_tmp->xpos[3 * ee_body_id + 0],
                        d_tmp->xpos[3 * ee_body_id + 1],
                        d_tmp->xpos[3 * ee_body_id + 2]
                    });
                }

                // 加锁更新给渲染线程
                {
                    std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
                    g_mpc_ui_state.predicted_ee_pos = current_pred_traj;
                    g_ref_traj = current_ref_traj;
                }
            }

        } else {
            // 求解失败处理，保持 last_cmd_q 不变
        }
    }

    // 2. 控制量下发 MuJoCo 
    for (int i = 0; i < 7; ++i) {
        double left_cmd = last_cmd_q[i];

        // 下发左臂位置
        if (hw_ids.left_ctrl_id[i] >= 0) {
            d->ctrl[hw_ids.left_ctrl_id[i]] = left_cmd;
        }
        // 右臂镜像动作
        double right_cmd = left_cmd;
        if (i == 0 || i == 1 || i == 3) {
            right_cmd = -left_cmd;
        }

        if (hw_ids.right_ctrl_id[i] >= 0) {
            d->ctrl[hw_ids.right_ctrl_id[i]] = right_cmd;
        }
    }
}

// MPC 轨迹画线函数
void render_mpc_trajectory(mjvScene* scn) {
    if (!g_mpc_ui_state.show_trajectory) return;

    std::vector<std::vector<double>> pred_traj;
    std::vector<std::vector<double>> ref_traj;
    std::vector<std::vector<double>> act_traj;
    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        pred_traj = g_mpc_ui_state.predicted_ee_pos;
        ref_traj  = g_ref_traj;
        act_traj  = g_act_traj;
    }

    // 内部 Lambda 函数：绘制线段序列
    auto draw_path = [&](const std::vector<std::vector<double>>& traj, float r, float g, float b, float a, double width) {
        if (traj.size() < 2) return;
        float rgba[4] = {r, g, b, a};
        for (size_t i = 0; i < traj.size() - 1; ++i) {
            if (scn->ngeom >= scn->maxgeom) break; // 场景几何体满则退出

            double pt1[3] = {traj[i][0], traj[i][1], traj[i][2]};
            double pt2[3] = {traj[i+1][0], traj[i+1][1], traj[i+1][2]};

            // 使用 mjGEOM_LINE 绘制连接线，宽度参数控制线条的粗细（像素级）
            mjv_initGeom(&scn->geoms[scn->ngeom], mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
            mjv_connector(&scn->geoms[scn->ngeom], mjGEOM_LINE, width, pt1, pt2);
            scn->ngeom++;
        }
    };

    // 1. 实际历史轨迹 - 蓝色 (Blue)，细线
    draw_path(act_traj, 0.0f, 0.5f, 1.0f, 1.0f, 5.0);
    
    // 2. 全局参考轨迹 - 绿色 (Green)，细线 (稍微调细，显得像一条轨道)
    draw_path(ref_traj, 0.0f, 1.0f, 0.0f, 1.0f, 5.0);
    
    // 3. MPC 预测轨迹 - 红色 (Red)，粗线 (指示当前的行动意图)
    draw_path(pred_traj, 1.0f, 0.0f, 0.0f, 1.0f, 20.0);
}
