#pragma once
#include "ocp_core/SystemDynamicsBase.h"

class SwerveDynamics: public SystemDynamicsBase {

public: 
    SwerveDynamics(double wheelbase, double trackwidth)
            : L(wheelbase), W(trackwidth) {}

    std::string getModelName() const override { return "swerve_chassis";}

    int getNu() const override { return Nu_;}          // 状态: [px, py, yaw, vx, vy, dyaw]
    int getNx() const override { return Nx_;}          // 输入: [ddx, ddy, ddyaw]

    // 定义动力学方程
    casadi::MX computeFlowMap (const casadi::MX& x, const casadi::MX& u) const override {
        // x: [px, py, yaw, vx, vy, vyaw]
        // u: [ddx, ddy, ddyaw]
        casadi::MX xdot = casadi::MX::zeros(Nx_, 1);
        xdot(0) = x(3);
        xdot(1) = x(4);
        xdot(2) = x(5);
        
        xdot(3) = u(0);
        xdot(4) = u(1);
        xdot(5) = u(2);
        return xdot;
    };

private:
    double L, W;
    int Nx_ = 6;
    int Nu_ = 3;
};
