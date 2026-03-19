#pragma once

#include "ocp_core/ConstraintBase.h"

using namespace casadi;
class SwerveConstraints: public NonlinearConstraintBase
{
public:
	SwerveConstraints(double wheelbase, double trackwidth, double max_steer_angle, double min_steer_angle){
		max_steer_angle_ = max_steer_angle;
		min_steer_angle_ = min_steer_angle;
		angle_mid_ = (max_steer_angle_ + min_steer_angle_) / 2.0;
		angle_half_ = (max_steer_angle_ - min_steer_angle_) / 2.0;
		// FL FR RL RR
		px_ = {wheelbase / 2.0,  wheelbase / 2.0, -wheelbase / 2.0,  -wheelbase / 2.0};
		py_ = {trackwidth / 2.0, -trackwidth / 2.0, -trackwidth / 2.0,  trackwidth / 2.0};

	};

	std::string getName() const override {
		return "steer_nonlinear_constr";
	};

	// 获取非线性约束的个数
	int getNumConstraints() const override {
		return wheel_num_;
	};
    void getBounds(std::vector<double>& lh, std::vector<double>& uh) const override {
        lh.assign(4, -1e-5);
        uh.assign(4,  1e6);
    }

	// 创建非线性约束
	casadi::MX computeConstraint(const casadi::MX& x, const casadi::MX& u, const casadi::MX& p) const override {
		// 定义非线性约束
		// 提取状态
        casadi::MX yaw  = x(2);
        casadi::MX vx_g = x(3);
        casadi::MX vy_g = x(4);
        casadi::MX dyaw = x(5);
		// 每次solve前，传入的direction (1, -1, 0)
		casadi::MX dir = p(0);

		casadi::MX c_yaw = cos(yaw);
        casadi::MX s_yaw = sin(yaw);

		casadi::MX c_mid = cos(angle_mid_);
		casadi::MX s_mid = sin(angle_mid_);

		casadi::MX c_half = cos(angle_half_);

		// 将全局坐标系下的线速度，旋转到车体坐标系 (Body Frame)
        // [vx_b, vy_b]^T = R(-yaw) * [vx_g, vy_g]^T
        casadi::MX vx_b =  c_yaw * vx_g + s_yaw * vy_g;
        casadi::MX vy_b = -s_yaw * vx_g + c_yaw * vy_g;
		double EPS_NORM = 1e-6;

		casadi::MX h = casadi::MX::zeros(N_constraints_, 1);

		for (std::size_t i = 0; i < wheel_num_; i++) {
			// 体坐标系轮速
			casadi::MX vbx_i = vx_b - dyaw * py_[i];
			casadi::MX vby_i = vy_b + dyaw * px_[i];

			// 根据 dir 翻转速度向量，使得舵轮始终保持在最小偏转角范围内
            casadi::MX v_wx_dir = dir * vbx_i;
            casadi::MX v_wy_dir = dir * vby_i;

			// 投影 & 模长
			casadi::MX proj = vbx_i * c_mid + vby_i * s_mid;
			casadi::MX nsq  = vbx_i * vbx_i + vby_i * vby_i + EPS_NORM;
			casadi::MX vel_sqrt = pow(nsq, 0.5);

			// 约束值
			h(i) = dir * proj - dir * dir * vel_sqrt * c_half;
		}
		return h;
	};


private: 
	std::vector<double> px_, py_;       // 存储轮子在body系的位置

	std::size_t wheel_num_ = 4;
	std::size_t N_constraints_ = 4;     // 非线性约束数量

	double angle_half_, angle_mid_;
	double max_steer_angle_, min_steer_angle_;
};

