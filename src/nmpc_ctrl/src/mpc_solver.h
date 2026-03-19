#pragma once
#include "ocp_core/SolverConfig.h"
#include <cmath>
#include <memory>

extern "C" {
    #include "acados_c/ocp_nlp_interface.h"
    #include "acados_c/external_function_interface.h"
    #include "acados/utils/math.h"
    #include "acados/utils/external_function_generic.h"
}

#include "mpc_backend/AcadosWrapper.h"
#include "ocp_core/CostBase.h"
#include "ocp_core/ConstraintBase.h"


/**
 * @brief 抽象求解策略接口
 */
class SolverStrategy {
public:
    virtual ~SolverStrategy() = default;
    virtual void applyPlan(ocp_nlp_plan_t* plan) const = 0;
    virtual void applyOptions(ocp_nlp_config* config, void* opts) const = 0;
};

/**
 * @brief RTI (Real-Time Iteration) 策略 - 每次执行单次QP求解，速度最快
 */
class RtiStrategy : public SolverStrategy {
public:
    void applyPlan(ocp_nlp_plan_t* plan) const override;
    void applyOptions(ocp_nlp_config* config, void* opts) const override;
};


/**
 * @brief Full SQP 策略 - 多次迭代直至满足KKT容差，适合高非线性
 */
class FullSqpStrategy : public SolverStrategy {
public:
    explicit FullSqpStrategy(int max_iter) : max_iter_(max_iter) {}
    void applyPlan(ocp_nlp_plan_t* plan) const override;
    void applyOptions(ocp_nlp_config* config, void* opts) const override;
private:
    int max_iter_;
};

/**
 * @brief 策略生成工厂
 */
class SolverStrategyFactory {
public:
    static std::unique_ptr<SolverStrategy> create(const SolverConfig& cfg);
};


/**
 * @brief 通用 MPC 求解器
 *
 * 完全通过 OcpConstraints 描述约束，不含任何机器人相关的硬编码。
 * 更换机器人只需更换 OcpConstraints 的配置，此类无需修改。
 */
class MPCSolver {
public:
    explicit MPCSolver(const SolverConfig& cfg)
        : cfg_(cfg), N_(cfg.N), nx_(cfg.nx), nu_(cfg.nu) {}

    ~MPCSolver();
    /**
     * @param wrapper      JIT 函数绑定
     * @param ocp_constr   OCP 约束完整描述（线性边界 + 非线性）
     * @param strategy     求解策略
     */
    bool setup(AcadosWrapper& wrapper, const OcpConstraints& ocp_constr, const CostBase* cost, const SolverStrategy& strategy);

    void set_x0(double *x0);
    void set_yref(int stage, double *yref);
    void set_online_parameter(int stage, const double *p);  // 设置在线参数 (传入 dir)

    int  solve();         

    void get_u(int stage, double *u);
    void get_x(int stage, double *x);
    void init_x(int stage, double *x);
    void init_u(int stage, double *u);
    int get_x_dimension() const { return nx_; }
    int get_u_dimension() const { return nu_; }
    int  N() const { return N_; }

private:
    // MpcParams p_;
    int N_;
    int nx_, nu_;
    SolverConfig cfg_;
    // 保存约束描述的副本，供 set_dimensions / set_constraints 使用
    // const OcpConstraints* constr_ = nullptr;


    ocp_nlp_plan_t *plan_   = nullptr;
    ocp_nlp_config *config_ = nullptr;
    ocp_nlp_dims   *dims_   = nullptr;
    ocp_nlp_in     *in_     = nullptr;
    ocp_nlp_out    *out_    = nullptr;
    ocp_nlp_solver *solver_ = nullptr;
    void           *opts_   = nullptr;

    // const CostBase* cost_ = nullptr;

    void set_dimensions(const OcpConstraints& constr);
    void set_constraints(AcadosWrapper& wrapper, const OcpConstraints& constr);
    void set_dynamics(AcadosWrapper& wrapper);
    void set_cost(AcadosWrapper& wrapper, const CostBase* cost, const OcpConstraints& constr);
};