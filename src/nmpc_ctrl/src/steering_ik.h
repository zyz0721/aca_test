#pragma once
#include "mpc_common.h"
#include <cmath>
#include <algorithm>

// ============================================================
// 单个舵轮的命令
// ============================================================
struct WheelCmd {
    double steer_angle;    // 舵角 [rad] (绕 z 轴)
    double drive_vel;      // 轮毂线速度 [m/s]
    double drive_omega;    // 轮毂角速度 [rad/s] (= drive_vel / wheel_radius)
};

struct WheelState {
    double steer_angle;    // 实际舵角 [rad]
    double drive_omega;    // 实际轮毂角速度 [rad/s]
};

// ============================================================
// 四舵轮逆运动学
//
// 输入: 体坐标系速度 (vx_body, vy_body, omega)
// 输出: 4 个轮子的 (舵角, 轮速)
//
// 轮子编号: 0=FL, 1=FR, 2=RL, 3=RR
//
// 体坐标系下第 i 个轮子的速度:
//   wx_i = vx_body - omega * ly_i
//   wy_i = vy_body + omega * lx_i
//
// 舵角: theta_i = atan2(wy_i, wx_i)
// 轮速: v_i = sqrt(wx_i^2 + wy_i^2)
//
// 特性:
//   1. 舵角钳位到 [steer_lim_min, steer_lim_max]
//   2. 舵角速率限制 (相对上一次)
//   3. 轮速限制
//   4. 速度接近零时保持上次舵角 (避免抖动)
//   5. 支持后退: 如果舵角超限, 翻转 180° + 反转轮速
// ============================================================
class SteeringIK {
public:
    void init(const MpcParams &p) {
        p_ = p;
        for (int i = 0; i < NUM_WHEELS; i++)
            last_steer_[i] = 0.0;
    }

    // 主接口: body vel → 4 个轮子命令
    void compute(double vx_body, double vy_body, double omega,
                 WheelCmd out[NUM_WHEELS], double dt)
    {
        for (int i = 0; i < NUM_WHEELS; i++) {
            double lx = p_.wheel_lx[i];
            double ly = p_.wheel_ly[i];

            // --- 体坐标系下轮速分量 ---
            double wx = vx_body - omega * ly;
            double wy = vy_body + omega * lx;
            double speed = std::sqrt(wx * wx + wy * wy);

            // --- 舵角 ---
            double steer;
            if (speed < 1e-3) {
                // 速度太小, 保持上次舵角
                steer = last_steer_[i];
            } else {
                steer = std::atan2(wy, wx);
            }

            // --- 如果舵角超限, 尝试翻转 (±180°, 轮速反向) ---
            double drive = speed;
            steer = normalize_steer(steer, drive);

            // --- 舵角速率限制 ---
            double max_delta = p_.max_steer_rate * dt;
            double delta = steer - last_steer_[i];
            // 处理角度跳变
            while (delta >  M_PI) delta -= 2.0 * M_PI;
            while (delta < -M_PI) delta += 2.0 * M_PI;
            if (std::fabs(delta) > max_delta) {
                delta = (delta > 0) ? max_delta : -max_delta;
            }
            steer = last_steer_[i] + delta;

            // --- 轮速限制 ---
            if (p_.max_wheel_vel > 0 && std::fabs(drive) > p_.max_wheel_vel) {
                drive = (drive > 0) ? p_.max_wheel_vel : -p_.max_wheel_vel;
            }

            // --- 输出 ---
            out[i].steer_angle = steer;
            out[i].drive_vel   = drive;
            out[i].drive_omega = (p_.wheel_radius > 0) ? drive / p_.wheel_radius : 0.0;

            last_steer_[i] = steer;
        }
    }

    // 重置 (切换轨迹时)
    void reset() {
        for (int i = 0; i < NUM_WHEELS; i++)
            last_steer_[i] = 0.0;
    }

private:
    MpcParams p_;
    double last_steer_[NUM_WHEELS] = {};

    // 将舵角归一化到限制范围内
    // 如果原始角度超限, 翻转 180° 并反转轮速
    double normalize_steer(double &steer, double &drive) {
        // 先归到 [-PI, PI)
        while (steer >  M_PI) steer -= 2.0 * M_PI;
        while (steer < -M_PI) steer += 2.0 * M_PI;

        // 如果在限制范围内, 直接返回
        if (steer >= p_.steer_lim_min && steer <= p_.steer_lim_max)
            return steer;

        // 尝试翻转 180°
        double flip = steer + ((steer > 0) ? -M_PI : M_PI);
        if (flip >= p_.steer_lim_min && flip <= p_.steer_lim_max) {
            steer = flip;
            drive = -drive;  // 轮速反向
            return steer;
        }

        // 两个都不行, 钳位到最近极限
        if (steer < p_.steer_lim_min) steer = p_.steer_lim_min;
        if (steer > p_.steer_lim_max) steer = p_.steer_lim_max;
        return steer;
    }
};

class SteeringOdom {
public:
    void init(const MpcParams &p) {
        p_ = p;
        x_ = 0; y_ = 0; yaw_ = 0;
    }

    // 输入: 4轮实际状态, dt
    // 输出: 更新内部位姿, 返回全局速度
    void update(const WheelState actual[NUM_WHEELS], double dt) {
        // FK: 4轮 → body vel (最小二乘)
        double w_num = 0, w_den = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            double v = actual[i].drive_omega * p_.wheel_radius;
            double sa = actual[i].steer_angle;
            double wx = v * cos(sa);  // 体坐标系 x 分量
            double wy = v * sin(sa);  // 体坐标系 y 分量
            double lx = p_.wheel_lx[i];
            double ly = p_.wheel_ly[i];
            double r2 = lx * lx + ly * ly;
            if (r2 > 1e-6) {
                w_num += lx * wy - ly * wx;
                w_den += r2;
            }
        }
        // $v_{iy} \cdot l_x - v_{ix} \cdot l_y = \omega \cdot (l_x^2 + l_y^2)$。
        omega_ = (w_den > 1e-6) ? w_num / w_den : 0.0;

        double sum_vx = 0, sum_vy = 0;
        for (int i = 0; i < NUM_WHEELS; i++) {
            double v = actual[i].drive_omega * p_.wheel_radius;
            double sa = actual[i].steer_angle;
            sum_vx += v * cos(sa) + omega_ * p_.wheel_ly[i];
            sum_vy += v * sin(sa) - omega_ * p_.wheel_lx[i];
        }
        vx_body_ = sum_vx / NUM_WHEELS;
        vy_body_ = sum_vy / NUM_WHEELS;

        // body → global
        double c = cos(yaw_), s = sin(yaw_);
        vx_global_ = c * vx_body_ - s * vy_body_;
        vy_global_ = s * vx_body_ + c * vy_body_;

        // 积分位姿
        x_   += vx_global_ * dt;
        y_   += vy_global_ * dt;
        yaw_ += omega_ * dt;
    }

    // 返回 MPC 需要的 state[6]
    void get_state(std::vector<double> & state) const {
        if (state.size() < 6) state.resize(6);
        state[0] = x_;
        state[1] = y_;
        state[2] = yaw_;
        state[3] = vx_global_;
        state[4] = vy_global_;
        state[5] = omega_;
    }

    void set_pose(double x, double y, double yaw) {
        x_ = x; y_ = y; yaw_ = yaw;
    }

private:
    MpcParams p_;
    double x_ = 0, y_ = 0, yaw_ = 0;
    double vx_body_ = 0, vy_body_ = 0, omega_ = 0;
    double vx_global_ = 0, vy_global_ = 0;
};