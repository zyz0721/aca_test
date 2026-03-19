#pragma once
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <cstring>
#include <vector>

#include "ocp_core/SolverConfig.h"


// 舵轮约束
#define NUM_WHEELS 4
// ============================================================
// 参数结构体
// ============================================================
struct MpcParams {
    // 核心求解器配置
    SolverConfig solver_cfg;
    // // --- 求解器 ---
    // int    nx  = 6;
    // int    nu  = 3;
    // int    N   = 20;
    // double T   = 2.0;
    // double hz  = 50.0;
    // std::string algorithm;
    // int sqp_max_iter;

    // --- 参考轨迹 ---
    std::string ref_type = "circle";
    double circle_r = 2.0, circle_v = 1.0;
    double line_vx = 1.0;
    double fig8_a = 2.0, fig8_omega = 0.3;

    // --- 车辆几何 ---
    double wheel_base  = 0.494;
    double track_width = 0.494;
    double wheel_radius = 0.08;     // 轮子半径 (用于 hub 角速度)

    // --- 运动学限制 ---
    double max_vel      = 2.0;
    double max_acc      = 1.0;
    double max_yaw_rate = 1.5;
    double max_yaw_acc  = 2.0;

    // --- 舵轮约束 (在 IK 层执行) ---
    double steer_lim_min   = -M_PI / 2.0 * 1.5;
    double steer_lim_max   =  M_PI / 2.0 * 1.5;
    double max_steer_rate  = 5.0;       // rad/s
    double max_wheel_vel   = 1.0;       // m/s (轮面线速度)
    
    // 在线参数维度
    int np = 1;

    // --- 初始状态 ---
    std::vector<double> x0;

    // --- 轮子位置 (体坐标系, setup 时计算) ---
    double wheel_lx[NUM_WHEELS];
    double wheel_ly[NUM_WHEELS];

    // --- 模式 ---
    bool sim_mode = true;           // true=仿真, false=实车(订阅odom)
    std::string odom_topic = "/odom";
    std::string joint_cmd_topic = "/joint_commands";

    // --- 关节名称 (URDF 中定义的名字) ---
    // 顺序: FL, FR, RL, RR
    std::vector<std::string> steer_joint_names;
    std::vector<std::string> drive_joint_names;

    void load(ros::NodeHandle &nh) {
        nh.param("solver/nx", solver_cfg.nx, solver_cfg.nx);
        nh.param("solver/nu", solver_cfg.nu, solver_cfg.nu);
        nh.param("solver/N",  solver_cfg.N,  solver_cfg.N);
        nh.param("solver/T",  solver_cfg.T,  solver_cfg.T);
        nh.param("solver/hz", solver_cfg.hz, solver_cfg.hz);
        nh.param<std::string>("solver/algorithm", solver_cfg.algorithm, solver_cfg.algorithm);
        nh.param("solver/sqp_max_iter", solver_cfg.sqp_max_iter, solver_cfg.sqp_max_iter);


        nh.param<std::string>("reference/type", ref_type, "circle");
        nh.param("reference/radius",     circle_r,    2.0);
        nh.param("reference/speed",      circle_v,    1.0);
        nh.param("reference/line_vx",    line_vx,     1.0);
        nh.param("reference/fig8_a",     fig8_a,      2.0);
        nh.param("reference/fig8_omega", fig8_omega,  0.3);

        nh.param("vehicle/wheel_base",    wheel_base,    0.6);
        nh.param("vehicle/track_width",   track_width,   0.5);
        nh.param("vehicle/wheel_radius",  wheel_radius,  0.075);
        nh.param("vehicle/max_vel",       max_vel,       2.0);
        nh.param("vehicle/max_acc",       max_acc,       1.0);
        nh.param("vehicle/max_yaw_rate",  max_yaw_rate,  1.5);
        nh.param("vehicle/max_yaw_acc",   max_yaw_acc,   2.0);
        nh.param("vehicle/steer_lim_min", steer_lim_min, -M_PI/2.0);
        nh.param("vehicle/steer_lim_max", steer_lim_max,  M_PI/2.0);
        nh.param("vehicle/max_steer_rate",max_steer_rate, 5.0);
        nh.param("vehicle/max_wheel_vel", max_wheel_vel,  1.0);

        // 轮子位置 (矩形底盘 FL/FR/RL/RR)
        double lx = wheel_base / 2.0, ly = track_width / 2.0;
        wheel_lx[0] =  lx; wheel_ly[0] =  ly;
        wheel_lx[1] =  lx; wheel_ly[1] = -ly;
        wheel_lx[2] = -lx; wheel_ly[2] =  ly;
        wheel_lx[3] = -lx; wheel_ly[3] = -ly;

        // 读取动态数组
        if (!nh.getParam("cost/W", solver_cfg.W)) {
            solver_cfg.W.assign(solver_cfg.nx + solver_cfg.nu, 1.0); // 默认兜底
        }
        if (!nh.getParam("cost_terminal/W_e", solver_cfg.W_e)) {
            solver_cfg.W_e.assign(solver_cfg.nx, 1.0);
        }

        if (!nh.getParam("initial_state/x0", x0)) {
            x0.assign(solver_cfg.nx, 0.0);
        }

        nh.param("mode/sim", sim_mode, true);
        nh.param<std::string>("mode/odom_topic", odom_topic, "/odom");
        nh.param<std::string>("mode/joint_cmd_topic", joint_cmd_topic, "/joint_commands");

        // 关节名称 (默认值)
        std::string prefix = "";
        nh.param<std::string>("joints/prefix", prefix, "");
        steer_joint_names = {
            prefix + "fl_steer_joint", prefix + "fr_steer_joint",
            prefix + "rl_steer_joint", prefix + "rr_steer_joint"
        };
        drive_joint_names = {
            prefix + "fl_drive_joint", prefix + "fr_drive_joint",
            prefix + "rl_drive_joint", prefix + "rr_drive_joint"
        };

    }
};

struct RefState {
    std::vector<double> val;
    std::vector<double> u_ref;
    // 增加构造函数分配内存
    RefState() = default;
    RefState(int nx, int nu) {
        val.resize(nx, 0.0);
        u_ref.resize(nu, 0.0);
    }
};