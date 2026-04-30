#pragma once

#include "ocp_core/CostBase.h"
#include <cmath>
#include <vector>

/**
 * @brief 完整底盘NMPC代价函数
 *
 * 参考 steer_nmpc.cpp 第156-186行，包含:
 * 1. 过程轨迹偏差代价 (位置、航向、速度)
 * 2. 控制量平滑代价 (加速度)
 * 3. 终端代价
 *
 * 使用 1-cos(yaw_err) 处理航向角周期性
 */
class ChassisFullCost : public CostBase {
public:
    struct Weights {
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
    };

    ChassisFullCost(int nx, int nu)
        : nx_(nx), nu_(nu) {}

    std::string getCostName() const override { return "chassis_full_cost"; }
    int getNumStageResiduals() const override { return nx_ + nu_; }
    int getNumTerminalResiduals() const override { return nx_; }

    // 非线性代价 (1-cos形式)
    bool isLinear() const override { return false; }

    casadi::MX computeStageCostResidual(const casadi::MX& x, const casadi::MX& u) const override {
        casadi::MX residual = casadi::MX::zeros(nx_ + nu_, 1);

        // 状态误差
        residual(0) = x(0);  // x_err
        residual(1) = x(1);  // y_err
        residual(2) = 1.0 - cos(x(2));  // yaw_err: 1-cos形式
        residual(3) = x(3);  // vx_err
        residual(4) = x(4);  // vy_err
        residual(5) = x(5);  // dyaw_err

        // 控制量
        residual(6) = u(0);  // ax
        residual(7) = u(1);  // ay
        residual(8) = u(2);  // ddyaw

        return residual;
    }

    casadi::MX computeTerminalCostResidual(const casadi::MX& x) const override {
        casadi::MX residual = casadi::MX::zeros(nx_, 1);

        residual(0) = x(0);
        residual(1) = x(1);
        residual(2) = 1.0 - cos(x(2));  // yaw_err: 1-cos形式
        residual(3) = x(3);
        residual(4) = x(4);
        residual(5) = x(5);

        return residual;
    }

    std::vector<double> getW() const {
        int ny = nx_ + nu_;
        std::vector<double> W(ny * ny, 0.0);
        W[0] = weights_.Q_xy;
        W[1 + ny] = weights_.Q_xy;
        W[2 + 2*ny] = weights_.Q_yaw;
        W[3 + 3*ny] = weights_.Q_vxy;
        W[4 + 4*ny] = weights_.Q_vxy;
        W[5 + 5*ny] = weights_.Q_dyaw;
        W[6 + 6*ny] = weights_.R_axy;
        W[7 + 7*ny] = weights_.R_axy;
        W[8 + 8*ny] = weights_.R_ddyaw;
        return W;
    }

    std::vector<double> getWe() const {
        std::vector<double> We(nx_ * nx_, 0.0);
        We[0] = weights_.Q_term_xy;
        We[1 + nx_] = weights_.Q_term_xy;
        We[2 + 2*nx_] = weights_.Q_term_yaw;
        We[3 + 3*nx_] = weights_.Q_term_vxy;
        We[4 + 4*nx_] = weights_.Q_term_vxy;
        We[5 + 5*nx_] = weights_.Q_term_dyaw;
        return We;
    }

    void setWeights(const Weights& w) { weights_ = w; }

private:
    int nx_, nu_;
    Weights weights_;
};