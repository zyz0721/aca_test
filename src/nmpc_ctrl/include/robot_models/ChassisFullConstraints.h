#pragma once

#include "ocp_core/ConstraintBase.h"
#include <cmath>
#include <vector>

/**
 * @brief 舵角投影约束 (扇形约束)
 *
 * 约束舵轮速度向量在允许的扇形区域内。
 * 参考 steer_nmpc.cpp 第114-124行
 *
 * 数学表达:
 *   dir * proj >= dir^2 * norm * cos(half_angle)
 *   其中 proj = v_body · u_mid (速度向量在扇形中线方向的投影)
 */
class SteerProjectionConstraint : public NonlinearConstraintBase {
public:
    SteerProjectionConstraint(double wheelbase, double trackwidth,
                              double max_steer_angle, double min_steer_angle)
        : max_steer_angle_(max_steer_angle), min_steer_angle_(min_steer_angle) {
        angle_mid_ = (max_steer_angle_ + min_steer_angle_) / 2.0;
        angle_half_ = (max_steer_angle_ - min_steer_angle_) / 2.0;
        cos_half_ = std::cos(angle_half_);
        cos_mid_ = std::cos(angle_mid_);
        sin_mid_ = std::sin(angle_mid_);

        // FL, FR, RL, RR
        px_ = {wheelbase / 2.0, wheelbase / 2.0, -wheelbase / 2.0, -wheelbase / 2.0};
        py_ = {-trackwidth / 2.0, trackwidth / 2.0, -trackwidth / 2.0, trackwidth / 2.0};
    }

    std::string getName() const override { return "steer_projection"; }
    int getNumConstraints() const override { return wheel_num_; }

    void getBounds(std::vector<double>& lh, std::vector<double>& uh) const override {
        lh.assign(wheel_num_, -1e-5);
        uh.assign(wheel_num_, 1e6);
    }

    casadi::MX computeConstraint(const casadi::MX& x, const casadi::MX& u,
                                 const casadi::MX& p) const override {
        casadi::MX yaw = x(2);
        casadi::MX vx_g = x(3);
        casadi::MX vy_g = x(4);
        casadi::MX dyaw = x(5);
        casadi::MX dir = p(0);

        casadi::MX c_yaw = cos(yaw);
        casadi::MX s_yaw = sin(yaw);

        // 全局速度转体坐标系
        casadi::MX vx_b = c_yaw * vx_g + s_yaw * vy_g;
        casadi::MX vy_b = -s_yaw * vx_g + c_yaw * vy_g;

        double EPS_NORM = 1e-6;
        casadi::MX h = casadi::MX::zeros(wheel_num_, 1);

        for (size_t i = 0; i < wheel_num_; i++) {
            casadi::MX vbx_i = vx_b - dyaw * py_[i];
            casadi::MX vby_i = vy_b + dyaw * px_[i];

            casadi::MX proj = vbx_i * cos_mid_ + vby_i * sin_mid_;
            casadi::MX nsq = vbx_i * vbx_i + vby_i * vby_i + EPS_NORM;
            casadi::MX vel_norm = sqrt(nsq);

            h(i) = dir * proj - dir * dir * vel_norm * cos_half_;
        }
        return h;
    }

private:
    std::vector<double> px_, py_;
    size_t wheel_num_ = 4;
    double max_steer_angle_, min_steer_angle_;
    double angle_mid_, angle_half_, cos_half_, cos_mid_, sin_mid_;
};

/**
 * @brief 舵角变化率软约束
 *
 * 限制相邻时刻舵轮速度向量的夹角变化率。
 * 参考 steer_nmpc.cpp 第127-137行
 *
 * 使用松弛变量: dot_raw + s >= dot_limit, s >= 0
 */
class SteerRateSoftConstraint : public NonlinearConstraintBase {
public:
    SteerRateSoftConstraint(double wheelbase, double trackwidth,
                            double max_steer_rate, double dt)
        : cos_rate_limit_(std::cos(max_steer_rate * dt)) {
        px_ = {wheelbase / 2.0, wheelbase / 2.0, -wheelbase / 2.0, -wheelbase / 2.0};
        py_ = {-trackwidth / 2.0, trackwidth / 2.0, -trackwidth / 2.0, trackwidth / 2.0};
    }

    std::string getName() const override { return "steer_rate_soft"; }
    int getNumConstraints() const override { return 2 * wheel_num_; }  // 每轮2个约束

    void getBounds(std::vector<double>& lh, std::vector<double>& uh) const override {
        // dot_raw + s >= dot_limit -> lh=0, uh=inf
        // s >= 0 -> lh=0, uh=inf
        lh.assign(2 * wheel_num_, 0.0);
        uh.assign(2 * wheel_num_, 1e6);
    }

    casadi::MX computeConstraint(const casadi::MX& x, const casadi::MX& u,
                                 const casadi::MX& p) const override {
        // 注: 舵角变化率需要相邻两时刻状态，此处返回占位符
        // 实际在OcpProblem构建时特殊处理
        return casadi::MX::zeros(2 * wheel_num_, 1);
    }

    double getCosRateLimit() const { return cos_rate_limit_; }
    size_t getWheelNum() const { return wheel_num_; }
    const std::vector<double>& getPx() const { return px_; }
    const std::vector<double>& getPy() const { return py_; }

private:
    std::vector<double> px_, py_;
    size_t wheel_num_ = 4;
    double cos_rate_limit_;
};