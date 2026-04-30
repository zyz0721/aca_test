#pragma once
#include <string>
#include <cmath>
#include <cstring>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "ocp_core/SolverConfig.h"


// 舵轮约束
#define NUM_WHEELS 4

// ============================================================
// 完整NMPC代价权重结构体 (参考 steer_nmpc.hpp VehicleConfig)
// ============================================================
struct FullCostWeights {
    // 过程代价权重
    double Q_xy = 15.0;
    double Q_yaw = 5.0;
    double Q_vxy = 1.0;
    double Q_dyaw = 2.0;
    // 控制量平滑权重
    double R_axy = 0.05;
    double R_ddyaw = 0.05;
    // 终端代价权重
    double Q_term_xy = 40.0;
    double Q_term_yaw = 10.0;
    double Q_term_vxy = 10.0;
    double Q_term_dyaw = 10.0;
    // 舵角变化率软约束惩罚
    double W_steer_rate = 200.0;
};

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
    std::string ref_type = "figure8";
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

    // --- 软约束惩罚参数 ---
    double soft_Z = 200.0;  // L2 惩罚
    double soft_z = 100.0;  // L1 惩罚

    // --- 运动方向阈值 ---
    double v_eps = 0.01;    // 速度阈值，判断前进/后退

    // --- 代价权重（分离字段）---
    double Q_xy = 15.0;
    double Q_yaw = 5.0;
    double Q_vxy = 1.0;
    double Q_dyaw = 2.0;
    double R_axy = 0.05;
    double R_ddyaw = 0.05;
    double Q_term_xy = 40.0;
    double Q_term_yaw = 10.0;
    double Q_term_vxy = 10.0;
    double Q_term_dyaw = 10.0;

    // --- 初始状态 ---
    std::vector<double> x0;

    // --- 轮子位置 (体坐标系, setup 时计算) ---
    double wheel_lx[NUM_WHEELS];
    double wheel_ly[NUM_WHEELS];

    // --- 模式 ---
    bool sim_mode = false;           // true=仿真, false=实车(订阅odom)
    std::string odom_topic = "/odom";
    std::string joint_cmd_topic = "/joint_commands";

    // --- 关节名称 (URDF 中定义的名字) ---
    // 顺序: FL, FR, RL, RR
    std::vector<std::string> steer_joint_names;
    std::vector<std::string> drive_joint_names;

    // ============================================================
    // 无 ROS 依赖的 YAML 解析器
    // ============================================================
    void loadFromYaml(const std::string& yaml_path) {
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            
            // 1. 读取 Solver 参数
            if (config["solver"]) {
                if (config["solver"]["nx"]) solver_cfg.nx = config["solver"]["nx"].as<int>();
                if (config["solver"]["nu"]) solver_cfg.nu = config["solver"]["nu"].as<int>();
                if (config["solver"]["N"]) solver_cfg.N = config["solver"]["N"].as<int>();
                if (config["solver"]["T"]) solver_cfg.T = config["solver"]["T"].as<double>();
                if (config["solver"]["hz"]) solver_cfg.hz = config["solver"]["hz"].as<double>();
                if (config["solver"]["algorithm"]) solver_cfg.algorithm = config["solver"]["algorithm"].as<std::string>();
                if (config["solver"]["sqp_max_iter"]) solver_cfg.sqp_max_iter = config["solver"]["sqp_max_iter"].as<int>();
            }

            // 2. 读取 Reference 参数
            if (config["reference"]) {
                if (config["reference"]["type"]) ref_type = config["reference"]["type"].as<std::string>();
                if (config["reference"]["radius"]) circle_r = config["reference"]["radius"].as<double>();
                if (config["reference"]["speed"]) circle_v = config["reference"]["speed"].as<double>();
                if (config["reference"]["line_vx"]) line_vx = config["reference"]["line_vx"].as<double>();
                if (config["reference"]["fig8_a"]) fig8_a = config["reference"]["fig8_a"].as<double>();
                if (config["reference"]["fig8_omega"]) fig8_omega = config["reference"]["fig8_omega"].as<double>();
            }

            // 3. 读取 Vehicle 车辆几何参数
            if (config["vehicle"]) {
                if (config["vehicle"]["wheel_base"]) wheel_base = config["vehicle"]["wheel_base"].as<double>();
                if (config["vehicle"]["track_width"]) track_width = config["vehicle"]["track_width"].as<double>();
                if (config["vehicle"]["wheel_radius"]) wheel_radius = config["vehicle"]["wheel_radius"].as<double>();
            }

            // 计算四轮位置
            double lx = wheel_base / 2.0;
            double ly = track_width / 2.0;
            wheel_lx[0] =  lx; wheel_ly[0] = -ly; // 左前
            wheel_lx[1] =  lx; wheel_ly[1] =  ly; // 右前
            wheel_lx[2] = -lx; wheel_ly[2] = -ly; // 左后
            wheel_lx[3] = -lx; wheel_ly[3] =  ly; // 右后

            // 4. 读取 Kinematics 运动学限制
            if (config["kinematics"]) {
                if (config["kinematics"]["max_vel"]) max_vel = config["kinematics"]["max_vel"].as<double>();
                if (config["kinematics"]["max_acc"]) max_acc = config["kinematics"]["max_acc"].as<double>();
                if (config["kinematics"]["max_yaw_rate"]) max_yaw_rate = config["kinematics"]["max_yaw_rate"].as<double>();
                if (config["kinematics"]["max_yaw_acc"]) max_yaw_acc = config["kinematics"]["max_yaw_acc"].as<double>();
            }

            // 5. 读取 Steering 舵轮参数
            if (config["steering"]) {
                if (config["steering"]["steer_lim_min"]) steer_lim_min = config["steering"]["steer_lim_min"].as<double>();
                if (config["steering"]["steer_lim_max"]) steer_lim_max = config["steering"]["steer_lim_max"].as<double>();
                if (config["steering"]["max_steer_rate"]) max_steer_rate = config["steering"]["max_steer_rate"].as<double>();
                if (config["steering"]["max_wheel_vel"]) max_wheel_vel = config["steering"]["max_wheel_vel"].as<double>();
            }

            // 6. 读取 Motion Direction 运动方向阈值
            if (config["motion_direction"]) {
                if (config["motion_direction"]["v_eps"]) v_eps = config["motion_direction"]["v_eps"].as<double>();
            }

            // 7. 读取 Cost 权重矩阵
            if (config["cost"] && config["cost"]["W"]) {
                solver_cfg.W = config["cost"]["W"].as<std::vector<double>>();
            } else {
                solver_cfg.W.assign(solver_cfg.nx + solver_cfg.nu, 1.0); // 兜底
            }

            if (config["cost_terminal"] && config["cost_terminal"]["W_e"]) {
                solver_cfg.W_e = config["cost_terminal"]["W_e"].as<std::vector<double>>();
            } else {
                solver_cfg.W_e.assign(solver_cfg.nx, 1.0); // 兜底
            }

            // 解析软约束惩罚参数
            if (config["soft_penalty"] && config["soft_penalty"]["steer_rate"]) {
                if (config["soft_penalty"]["steer_rate"]["Z"]) soft_Z = config["soft_penalty"]["steer_rate"]["Z"].as<double>();
                if (config["soft_penalty"]["steer_rate"]["z"]) soft_z = config["soft_penalty"]["steer_rate"]["z"].as<double>();
            }

            // 解析分离的代价权重
            if (config["cost"]) {
                if (config["cost"]["Q_xy"]) Q_xy = config["cost"]["Q_xy"].as<double>();
                if (config["cost"]["Q_yaw"]) Q_yaw = config["cost"]["Q_yaw"].as<double>();
                if (config["cost"]["Q_vxy"]) Q_vxy = config["cost"]["Q_vxy"].as<double>();
                if (config["cost"]["Q_dyaw"]) Q_dyaw = config["cost"]["Q_dyaw"].as<double>();
                if (config["cost"]["R_axy"]) R_axy = config["cost"]["R_axy"].as<double>();
                if (config["cost"]["R_ddyaw"]) R_ddyaw = config["cost"]["R_ddyaw"].as<double>();
            }
            if (config["cost_terminal"]) {
                if (config["cost_terminal"]["Q_term_xy"]) Q_term_xy = config["cost_terminal"]["Q_term_xy"].as<double>();
                if (config["cost_terminal"]["Q_term_yaw"]) Q_term_yaw = config["cost_terminal"]["Q_term_yaw"].as<double>();
                if (config["cost_terminal"]["Q_term_vxy"]) Q_term_vxy = config["cost_terminal"]["Q_term_vxy"].as<double>();
                if (config["cost_terminal"]["Q_term_dyaw"]) Q_term_dyaw = config["cost_terminal"]["Q_term_dyaw"].as<double>();
            }

            // 8. 读取初始状态
            if (config["initial_state"] && config["initial_state"]["x0"]) {
                x0 = config["initial_state"]["x0"].as<std::vector<double>>();
            } else {
                x0.assign(solver_cfg.nx, 0.0);
            }

            // 9. 读取其他模式信息(兼容用)
            if (config["mode"]) {
                if (config["mode"]["sim"]) sim_mode = config["mode"]["sim"].as<bool>();
                if (config["mode"]["odom_topic"]) odom_topic = config["mode"]["odom_topic"].as<std::string>();
            }

            std::string prefix = "";
            if (config["joints"] && config["joints"]["prefix"]) {
                prefix = config["joints"]["prefix"].as<std::string>();
            }
            // steer_joint_names = {
            //     prefix + "fl_steer_joint", prefix + "fr_steer_joint",
            //     prefix + "rl_steer_joint", prefix + "rr_steer_joint"
            // };
            // drive_joint_names = {
            //     prefix + "fl_drive_joint", prefix + "fr_drive_joint",
            //     prefix + "rl_drive_joint", prefix + "rr_drive_joint"
            // };

            std::cout << "[MpcParams] Successfully loaded configuration from: " << yaml_path << std::endl;
        } 
        catch (const YAML::Exception& e) {
            std::cerr << "[MpcParams] Error parsing YAML file: " << e.what() << std::endl;
            // 初始化默认的四轮坐标以防崩溃
            double lx = wheel_base / 2.0;
            double ly = track_width / 2.0;
            wheel_lx[0] =  lx; wheel_ly[0] =  ly;
            wheel_lx[1] =  lx; wheel_ly[1] = -ly;
            wheel_lx[2] = -lx; wheel_ly[2] =  ly;
            wheel_lx[3] = -lx; wheel_ly[3] = -ly;
        }
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