#pragma once
#include <casadi/casadi.hpp>
#include <string>


/**
 * @brief 代价函数基类
 *
 * 支持两种 acados 代价模式:
 *   - LINEAR_LS:     残差 y = [x; u]，代价 = ||y - y_ref||^2_W
 *   - NONLINEAR_LS:  残差 y = f(x,u)，代价 = ||y - y_ref||^2_W
 *
 * 当残差就是 [x; u] 时（如 SwerveCost），等价于 LINEAR_LS，
 * 框架自动检测并选择更高效的模式。
 */

class CostBase {
  
public:
    virtual ~CostBase() = default;
  
    /**
     * @brief 计算运行阶段的代价 (Stage Cost) L(x, u)
     * acados 中常用的 NONLINEAR_LS 模式需要你提供非线性的残差向量 y(x,u)
     * 最终代价将是 0.5 * (y - y_ref)^T * W * (y - y_ref)
     * 这里我们直接返回残差向量 y(x,u)
     */
    /// 阶段代价的残差向量 y(x, u)
    virtual casadi::MX computeStageCostResidual(const casadi::MX &x, const casadi::MX &u) const = 0;

    /// 终端代价的残差向量 y_e(x)
    virtual casadi::MX computeTerminalCostResidual(const casadi::MX &x) const = 0;

    virtual std::string getCostName() const = 0;
    virtual int getNumStageResiduals() const = 0;
    virtual int getNumTerminalResiduals() const = 0;
    
    /// 是否为线性残差（框架自动选择 LINEAR_LS / NONLINEAR_LS）
    /// 默认 false（保守选择 NONLINEAR_LS），子类可 override
    virtual bool isLinear() const {return false;}

    virtual std::vector<double> getVx() const { return {}; }
    virtual std::vector<double> getVu() const { return {}; }
    virtual std::vector<double> getVxe() const { return {}; }

};