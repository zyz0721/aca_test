#pragma once
#include "ocp_core/SystemDynamicsBase.h"

// 极简 7 自由度运动学模型
// 状态 x: 7个关节角度 [q1, q2, q3, q4, q5, q6, q7]
// 控制 u: 7个关节速度 [dq1, dq2, dq3, dq4, dq5, dq6, dq7]
class ArmDynamics : public SystemDynamicsBase {
public:
    std::string getModelName() const override { return "left_arm_kinematics"; }
    int getNx() const override { return 7; } 
    int getNu() const override { return 7; } 

    // 改为匹配 SystemDynamicsBase 的 computeFlowMap 和 casadi::MX
    casadi::MX computeFlowMap(const casadi::MX& x, const casadi::MX& u) const override {
        // 运动学模型：位置的导数就是速度。方程：dx/dt = u
        return u; 
    }
};