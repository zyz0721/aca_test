#pragma once

#include <casadi/casadi.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <utility> 
#include <algorithm> 
#include <chrono>
// 车辆配置参数结构体
struct VehicleConfig {
    // 1. 几何参数 (单位: m)
    double wheel_base = 0.6;  // 轴距
    double track_width = 0.5; // 轮距
    
    // 2. 运动学限制
    double max_vel = 2.0;          // 最大线速度 (m/s)
    double max_acc = 1.0;          // 最大加速度 (m/s^2)
    double max_yaw_rate = 1.5;     // 最大角速度 (rad/s)
    double max_yaw_acc = 2.0;      // 最大角加速度 (rad/s^2)
    
    // 3. 舵轮参数
    double steer_lim_min = -M_PI / 2.0;  // 舵轮转向角约束
    double steer_lim_max = M_PI / 2.0;
    double max_steer_rate = 5.0;   // 最大舵角变化率 (rad/s)
    double max_wheel_vel = 1.0; // 最大轮速 (m/s)
    
    // 4. 权重参数 
    double Q_xy = 0.5;     // 位置权重 
    double Q_yaw = 5.0;    // 航向权重
    double Q_vxy = 0.5;     // 速度跟踪权重 
    double Q_dyaw = 0.5;
    // 控制量平滑权重
    double R_axy = 0.6;     
    double R_ddyaw = 0.6;  
    // 终端权重 (Terminal Cost)
    double Q_term_xy = 10.0;
    double Q_term_yaw = 10.0;
    double Q_term_vxy = 10.0;
    double Q_term_ddyaw = 10.0;
    // 舵角转向速率软约束权重
    double W_steer_rate = 200;
    
    // 轮子位置相对车中心的坐标 顺序: FL, FR, RL, RR
    std::vector<std::pair<double, double>> wheel_positions;

    VehicleConfig() {
        // 初始化轮子位置 (假设矩形底盘)
        double l_x = wheel_base / 2.0;
        double l_y = track_width / 2.0;
        wheel_positions = {
            {l_x, l_y},   // FL
            {l_x, -l_y},  // FR
            {-l_x, l_y},  // RL
            {-l_x, -l_y}  // RR
        };
    }
};
// 优化结果结构体
struct MpcResult {
    // 平滑后的轨迹 [N, 6] 
    // Format: x, y, yaw, vx, vy, dyaw
    std::vector<std::vector<double>> ref_traj;
    std::vector<std::vector<double>> result_traj;
    
    // 控制输出 [N, 4] 体坐标系下的(vx, vy, dyaw, dt)
    std::vector<std::vector<double>> out_control;
    
    int status = -1;
    double opt_time_ms = 0.0;
};
class SteeringNMPC {
public:
    /**
     * @brief 构造函数
     * @param N 预测步长 (Horizon Length)
     * @param dt 采样时间 (s)
     * @param config 车辆参数配置
     */
    SteeringNMPC(int N, double dt, const VehicleConfig& config);
    ~SteeringNMPC() = default;
    void reset();

    /**
     * @brief 轨迹平滑接口
     * @param ref_traj 参考轨迹 [N+1, 6] (x, y, yaw, vx, vy, dyaw)
     * @return MpcResult 结果结构体
     */
    MpcResult control(const std::vector<std::vector<double>>& ref_traj,
                            const std::vector<double>& current_state);

private:
    std::size_t N_;             // 预测时域
    double dt_;         // 时间步长
    VehicleConfig cfg_; // 车辆参数
    // CasADi 对象
    casadi::Opti opti_;
    
    // 决策变量 (State) - 维度: 6 x (N+1)
    // 0:x, 1:y, 2:yaw, 3:vx, 4:vy, 5:dyaw (Global Frame)
    casadi::MX X_; 
    // 决策变量 (Control) - 维度: 3 x N
    // 0:ax, 1:ay, 2:ddyaw (Global Frame)
    casadi::MX U_;

    // 参数 (Parameters)
    casadi::MX P_curr_; // 当前状态参数
    casadi::MX P_ref_;  // 参考轨迹参数
    
    casadi::MX P_direction_; // 运动方向
    casadi::MX Slacks_; // 软约束松弛参数

    // 暖启动缓存
    std::vector<double> last_u_sol_;
    std::vector<double> last_x_sol_;
    bool has_solution_ = false;

    // 运动方向
    int motion_direction_ = 0;
    std::vector<double> motion_direction_vec_;

    // 构建函数
    void setup_optimization();
    // 执行优化
    casadi::OptiSol execute_optimization();
    // 设置参考轨迹
    void set_reference_trajectory(const std::vector<std::vector<double>>& ref_traj,const std::vector<double>& current_state);
    // 计算全局坐标系下的轮速
    casadi::MX get_global_wheel_velocity(const casadi::MX& state, std::size_t wheel_idx);

    std::pair<double, double> compute_wheel_vel_numeric(
        const std::vector<double>& state, std::size_t wheel_idx);
    // 计算运动方向
    int compute_forward_direction(const std::vector<std::vector<double>>& ref_traj);
};

