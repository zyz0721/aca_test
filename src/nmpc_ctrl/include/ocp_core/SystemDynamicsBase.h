#pragma once
#include <casadi/casadi.hpp>
#include <string>

class SystemDynamicsBase {
public: 
    virtual ~SystemDynamicsBase() = default;

    // 上层需要实现的纯虚函数，即前向计算图
    // 输入符号变量x和u，返回符号变量xdot
    virtual casadi::MX computeFlowMap(const casadi::MX& x, const casadi::MX& u) const = 0;

    // 模型名字，用于.so的命名
    virtual std::string getModelName() const = 0;

    // 状态维度 (默认从computeFlowMap推断，子类可override)
    virtual int getNx() const = 0; //-1 = 未实现，使用全局NX

    // 输入维度
    virtual int getNu() const = 0; // -1 = 未实现，使用全局NU


};