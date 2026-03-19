#pragma once

#include "ocp_core/CostBase.h"

/**
 * 线性跟踪代价: 残差 y = [x; u], 终端残差 y_e = x
 * isLinear() = true → 框架自动使用 LINEAR_LS 模式
 */
class SwerveCost: public CostBase {

public: 
	SwerveCost(int nx, int nu) : nx_(nx), nu_(nu) {}
	casadi::MX computeStageCostResidual(const casadi::MX& x, const casadi::MX& u) const override {
		return casadi::MX::vertcat({x,u});
	}

	casadi::MX computeTerminalCostResidual(const casadi::MX& x) const override {
		return x;
	}

	std::string getCostName() const override { return "chassis_tracking_cost"; }
    int getNumStageResiduals() const override { return nx_ + nu_; }
	int getNumTerminalResiduals() const override { return nx_; }
	bool isLinear() const override {return true;}

	std::vector<double> getVx() const override {
		int ny = nx_ + nu_;
		std::vector<double> Vx(ny * nx_, 0.0);
		for (int j = 0; j < nx_; j++) Vx[j + j * ny] = 1.0;
		return Vx;
	}

	std::vector<double> getVu() const override {
		int ny = nx_ + nu_;
		std::vector<double> Vu(ny * nu_, 0.0);
		for (int j = 0; j < nu_; j++) Vu[(nx_ + j) + j * ny] = 1.0;
		return Vu;
	}

	std::vector<double> getVxe() const override {
		std::vector<double> Vxe(nx_ * nx_, 0.0);
		for (int j = 0; j < nx_; j++) Vxe[j + j * nx_] = 1.0;
		return Vxe;
	}

private:
	int nx_;
    int nu_;
};