#include "mujoco_nmpc_sim.h"
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

// 引入 NMPC 框架头文件 
#include "ocp_core/OcpProblem.h"
#include "robot_models/ArmDynamics.h"

// 全局静态变量管理
static const mjModel* cached_model = nullptr;
static std::unique_ptr<MPCSolver> mpc_solver;
static std::unique_ptr<AcadosWrapper> mpc_wrapper;

// 存储硬件在 MuJoCo 内存数组中的索引
struct HardwareIDs {
    int left_qpos_adr[7];
    int left_ctrl_id[7];
    int right_ctrl_id[7];
};
static HardwareIDs hw_ids;

// MPC 运行频率控制
static double last_mpc_time = -1.0;
static const double MPC_DT = 0.02; // 50Hz = 0.02s
static std::vector<double> last_cmd_q(7, 0.0); // 缓存上一次的控制目标

MPCUIState g_mpc_ui_state;

// ======================= 初始化 NMPC =======================
void init_nmpc(const mjModel* m, mjData* d) {
    std::cout << "\n[NMPC] Initializing Acados/CasADi Solver for MuJoCo..." << std::endl;

    // 1. 获取模型中左右臂的内存地址 ID
    for (int i = 0; i < 7; ++i) {
        std::string left_name = "left_arm_joint" + std::to_string(i+1);
        std::string right_name = "right_arm_joint" + std::to_string(i+1);

        int left_jnt_id = mj_name2id(m, mjOBJ_JOINT, left_name.c_str());
        hw_ids.left_qpos_adr[i] = (left_jnt_id >= 0) ? m->jnt_qposadr[left_jnt_id] : -1;

        hw_ids.left_ctrl_id[i]  = mj_name2id(m, mjOBJ_ACTUATOR, left_name.c_str());
        hw_ids.right_ctrl_id[i] = mj_name2id(m, mjOBJ_ACTUATOR, right_name.c_str());
    }

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

        // A. 从 MuJoCo 读取当前状态
        std::vector<double> current_q(7, 0.0);
        for (int i = 0; i < 7; ++i) {
            if (hw_ids.left_qpos_adr[i] >= 0) {
                current_q[i] = d->qpos[hw_ids.left_qpos_adr[i]];
            }
        }

        // B. 生成参考轨迹
        std::vector<double> target_q = {0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0};
        std::vector<double> target_dq(7, 0.0); 
        
        target_q[0] = 0.5 * sin(1.0 * t);           
        target_q[1] = -1.0 - 0.5 * cos(1.0 * t);    
        target_q[3] = -1.57 + 0.8 * sin(1.0 * t);   

        target_dq[0] = 0.5 * 1.0 * cos(1.0 * t);
        target_dq[1] = 0.5 * 1.0 * sin(1.0 * t);
        target_dq[3] = 0.8 * 1.0 * cos(1.0 * t);

        std::vector<double> yref(14);
        std::copy(target_q.begin(), target_q.end(), yref.begin());
        std::copy(target_dq.begin(), target_dq.end(), yref.begin() + 7);

        // C. 送入求解器求解 
        mpc_solver->set_x0(current_q.data());
        
        int N = 20; // cfg.N
        for (int i = 0; i < N; i++) mpc_solver->set_yref(i, yref.data());
        mpc_solver->set_yref(N, target_q.data()); // 终端参考

        if (mpc_solver->solve() == 0) {
            // 解析出下个时刻的期望状态作为控制位置指令
            std::vector<double> next_q(7);
            mpc_solver->get_x(1, next_q.data()); 
            
            for (int i = 0; i < 7; ++i) {
                last_cmd_q[i] = next_q[i]; // 更新缓存
            }

            // 提取未来 N 步的末端位置用于画线
            if (g_mpc_ui_state.show_trajectory) {
                static mjData* d_tmp = nullptr;
                if (!d_tmp) d_tmp = mj_makeData(m); // 创建一个临时 d 避免污染主物理循环

                // 轨迹末端
                static int ee_body_id = mj_name2id(m, mjOBJ_BODY, "left_arm_end_effector_mount_link");

                mju_copy(d_tmp->qpos, d->qpos, m->nq); // 拷贝当前状态

                std::vector<std::vector<double>> current_traj;
                int render_N = std::min(g_mpc_ui_state.horizon_steps, 20); // 你的 N

                for (int i = 0; i < render_N; i++) {
                    std::vector<double> pred_q(7);
                    mpc_solver->get_x(i, pred_q.data()); 

                    // 将预测的关节角写给 d_tmp
                    for(int j=0; j<7; j++) {
                        d_tmp->qpos[hw_ids.left_qpos_adr[j]] = pred_q[j];
                    }

                    // 计算当前预测 q 下的运动学
                    mj_kinematics(m, d_tmp);

                    // 获取末端 XYZ
                    std::vector<double> pos = {
                        d_tmp->xpos[3 * ee_body_id + 0],
                        d_tmp->xpos[3 * ee_body_id + 1],
                        d_tmp->xpos[3 * ee_body_id + 2]
                    };
                    current_traj.push_back(pos);
                }

                // 加锁更新给渲染线程
                {
                    std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
                    g_mpc_ui_state.predicted_ee_pos = current_traj;
                }
            }

        } else {
            // 求解失败处理，保持 last_cmd_q 不变
            // std::cerr << "[NMPC] Solve failed at t=" << t << std::endl;
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

    std::vector<std::vector<double>> traj;
    {
        std::lock_guard<std::mutex> lock(g_mpc_ui_state.mtx);
        traj = g_mpc_ui_state.predicted_ee_pos;
    }

    if (traj.size() < 2) return;

    float rgba[4] = {1.0f, 0.0f, 0.0f, 1.0f}; // 红色
    double width = 20.0; // 粗细

    for (size_t i = 0; i < traj.size() - 1; ++i) {
        if (scn->ngeom >= scn->maxgeom) break; // 场景几何体满则退出

        double pt1[3] = {traj[i][0], traj[i][1], traj[i][2]};
        double pt2[3] = {traj[i+1][0], traj[i+1][1], traj[i+1][2]};

        mjv_initGeom(&scn->geoms[scn->ngeom], mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
        mjv_connector(&scn->geoms[scn->ngeom], mjGEOM_LINE, width, pt1, pt2);
        scn->ngeom++;
    }
}
