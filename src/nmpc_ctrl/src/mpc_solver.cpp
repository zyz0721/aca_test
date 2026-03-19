#include "mpc_solver.h"
#include <unistd.h>
#include <fcntl.h>


// ============================================================
// 求解策略实现
// ============================================================
void RtiStrategy::applyPlan(ocp_nlp_plan_t* plan) const {
    plan->nlp_solver = SQP_RTI;
    plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
}

void RtiStrategy::applyOptions(ocp_nlp_config* config, void* opts) const {
    // RTI 默认单次迭代，可在此配置特殊选项
}

void FullSqpStrategy::applyPlan(ocp_nlp_plan_t* plan) const {
    plan->nlp_solver = SQP;   // 启用完整的 SQP
    plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
}
void FullSqpStrategy::applyOptions(ocp_nlp_config* config, void* opts) const {
    ocp_nlp_solver_opts_set(config, opts, "max_iter", (void*)&max_iter_);
    // 如果需要设置收敛容差 tol_stat, tol_eq, tol_ineq, tol_comp 也可以在这里加
}

std::unique_ptr<SolverStrategy> SolverStrategyFactory::create(const SolverConfig& cfg) {
    if (cfg.algorithm == "SQP") {
        return std::make_unique<FullSqpStrategy>(cfg.sqp_max_iter);
    }
    // 默认回退到 RTI
    return std::make_unique<RtiStrategy>();
}


// ============================================================
// 基本接口
// ============================================================

MPCSolver::~MPCSolver() {
    if (solver_) {
        ocp_nlp_solver_destroy(solver_);
        solver_ = nullptr;
    }
    if (opts_) {
        ocp_nlp_solver_opts_destroy(opts_);
        opts_ = nullptr;
    }
    if (out_) {
        ocp_nlp_out_destroy(out_);
        out_ = nullptr;
    }
    if (in_) {
        ocp_nlp_in_destroy(in_);
        in_ = nullptr;
    }
    if (dims_) {
        ocp_nlp_dims_destroy(dims_);
        dims_ = nullptr;
    }
    if (config_) {
        ocp_nlp_config_destroy(config_);
        config_ = nullptr;
    }
    if (plan_) {
        ocp_nlp_plan_destroy(plan_);
        plan_ = nullptr;
    }
    // 注意：cost_ 和 constr_ 的生命周期由上层 OcpProblem 管理（使用了 shared_ptr / 栈内存），此处无需 delete。
}


void MPCSolver::set_x0(double *x0) {
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(config_, dims_, in_, out_, 0, "ubx", x0);
}
void MPCSolver::set_yref(int i, double *yref) { ocp_nlp_cost_model_set(config_, dims_, in_, i, "yref", yref);}
void MPCSolver::set_online_parameter(int stage, const double *p) { ocp_nlp_in_set(config_, dims_, in_, stage, "parameter_values", (void*)p);}
int MPCSolver::solve() {
    // ★ acados SQP_RTI 内部用 printf 硬编码打印 QP 子问题状态
    //   print_level=0 也压不住。临时重定向 stdout → /dev/null
    fflush(stdout);
    int saved_fd = dup(STDOUT_FILENO);
    int devnull = open("/dev/null", O_WRONLY);
    dup2(devnull, STDOUT_FILENO);
    close(devnull);

    int status = ocp_nlp_solve(solver_, in_, out_);

    fflush(stdout);
    dup2(saved_fd, STDOUT_FILENO);
    close(saved_fd);

    return status;
}

void MPCSolver::get_u(int i, double *u) { ocp_nlp_out_get(config_, dims_, out_, i, "u", u);}
void MPCSolver::get_x(int i, double *x) { ocp_nlp_out_get(config_, dims_, out_, i, "x", x);}
void MPCSolver::init_x(int i, double *x) { ocp_nlp_out_set(config_, dims_, out_, in_, i, "x", x);}
void MPCSolver::init_u(int i, double *u) { ocp_nlp_out_set(config_, dims_, out_, in_, i, "u", u);}

// ============================================================
// setup
// ============================================================
bool MPCSolver::setup(AcadosWrapper& wrapper, const OcpConstraints& ocp_constr, const CostBase* cost, const SolverStrategy& strategy) {

    // constr_ = &ocp_constr;
    // cost_   = cost;

    printf("[SETUP] 1. Creating NLP plan...\n");
    // 初始化一个非线性规划（NLP）求解器的布局方案
    plan_ = ocp_nlp_plan_create(N_);                                    // N_时域长度，会定义一个结构体，定义打算采用使用什么算法组件

    strategy.applyPlan(plan_);

    bool is_linear_cost = (cost == nullptr || cost->isLinear());

    for (int i = 0; i <= N_; i++) {
        plan_->nlp_cost[i] = is_linear_cost ? LINEAR_LS : NONLINEAR_LS; // 设置代价函数类型为线性最小二乘 $\|Ax - b\|^2_W$
        plan_->nlp_constraints[i] = BGH;                                // 状态方程类型为BGH，B-Bounds， G-一般线性约束，H-非线性约束
        if (i < N_) {
            plan_->nlp_dynamics[i] = CONTINUOUS_MODEL;                  // 系统动力学是以连续微分方程
            plan_->sim_solver_plan[i].sim_solver = ERK;                 // 显示龙格库塔积分
        }
    }

    printf("[SETUP] 2. Creating config and dims...\n");
    // 根据填入的 plan_，生成具体的配置对象
    config_ = ocp_nlp_config_create(*plan_);                       

    // 创建维度对象，调用ocp_nlp_dims_set定义变量维度，包括状态以及输入
    dims_ = ocp_nlp_dims_create(config_);                          
    set_dimensions(ocp_constr);                                 

    in_  = ocp_nlp_in_create(config_, dims_);
    out_ = ocp_nlp_out_create(config_, dims_);

    printf("[SETUP] 3. Setting constraints and binding JIT functions...\n");
    set_constraints(wrapper, ocp_constr);
    printf("[SETUP] 4. Setting dynamics and binding JIT functions...\n");
    set_dynamics(wrapper);
    printf("[SETUP] 5. Setting costs...\n");
    set_cost(wrapper, cost, ocp_constr);

    printf("[SETUP] 6. Creating solver options...\n");
    // 配置优化器参数
    opts_ = ocp_nlp_solver_opts_create(config_, dims_);                        
    double lev_mar = cfg_.levenberg_marquardt;
    ocp_nlp_solver_opts_set(config_, opts_, "levenberg_marquardt", &lev_mar); // 算法优化计算时，需要求解Hessian矩阵的逆矩阵，在对角线加上一个正数，防止出现数值爆炸-类似于阻尼最小二乘
    int print_level = cfg_.print_level;
    ocp_nlp_solver_opts_set(config_, opts_, "print_level", &print_level);     // 关闭内部打印
    // 引入特定的策略
    strategy.applyOptions(config_, opts_);

    printf("[SETUP] 7. Allocating internal memory for solver (Danger Zone)...\n");
    // 根据定义的dims_,在内存开辟内存池，存放每次循环所需的KKT矩阵、梯度向量等  
    solver_ = ocp_nlp_solver_create(config_, dims_, opts_, in_);               
    if (!solver_) {
        printf("[SETUP]  solver_create returned NULL!\n");
        return false;
    }
    printf("[SETUP] 8. Precomputing KKT matrices...\n");
    ocp_nlp_precompute(solver_, in_, out_);                                 // 将矩阵中一些不变的，提前进行计算，

    printf("MPCSolver setup OK (nx=%d, nu=%d, N=%d).\n", nx_, nu_, N_);
    return true;
}

// 使用 nx_/nu_ 成员变量代替全局宏 NX/NU
void MPCSolver::set_dimensions(const OcpConstraints& constr) {
    std::vector<int> nx_a(N_+1), nu_a(N_+1), nz_a(N_+1, 0), ns_a(N_+1, 0), np_a(N_+1, cfg_.np);
    for (int i = 0; i <= N_; i++) {
        nx_a[i] = nx_;
        nu_a[i] = (i < N_) ? nu_ : 0;

        int nsh; 
        std::vector<int> dummy_idx; 
        std::vector<double> dummy_d;

        constr.getNonlinearSoftInfoForStage(i, N_, nsh, dummy_idx, dummy_d, dummy_d, dummy_d, dummy_d);
        ns_a[i] = nsh; // 当前框架优先将松弛变量赋给非线性软约束
    }
    ocp_nlp_dims_set_opt_vars(config_, dims_, "nx", nx_a.data());
    ocp_nlp_dims_set_opt_vars(config_, dims_, "nu", nu_a.data());
    ocp_nlp_dims_set_opt_vars(config_, dims_, "nz", nz_a.data());
    ocp_nlp_dims_set_opt_vars(config_, dims_, "ns", ns_a.data());
    ocp_nlp_dims_set_opt_vars(config_, dims_, "np", np_a.data());

    for (int i = 0; i < N_; i++) {
        int v;
        v = nx_; ocp_nlp_dims_set_dynamics(config_, dims_, i, "nx", &v);
        v = nu_; ocp_nlp_dims_set_dynamics(config_, dims_, i, "nu", &v);
        v = 0;   ocp_nlp_dims_set_dynamics(config_, dims_, i, "nz", &v);
    }
    for (int i = 0; i <= N_; i++) {
        int ny = (i < N_) ? (nx_ + nu_) : nx_;
        ocp_nlp_dims_set_cost(config_, dims_, i, "ny", &ny);
    }

    for (int i = 0; i <= N_; i++) {
        std::vector<int> bx_idx; std::vector<double> bx_lb, bx_ub;
        constr.getStateBoundsForStage(i, N_, bx_idx, bx_lb, bx_ub);
        int nbx = (int)bx_idx.size();

        std::vector<int> bu_idx; std::vector<double> bu_lb, bu_ub;
        constr.getInputBoundsForStage(i, N_, bu_idx, bu_lb, bu_ub);
        int nbu = (int)bu_idx.size();

        auto lc = constr.getLinearForStage(i, N_, nx_, nu_);
        int ng = lc.ng;
        int nh = constr.getNhForStage(i, N_);

        printf("[DIMS] stage %d: nbx=%d nbu=%d ng=%d nh=%d\n", i, nbx, nbu, ng, nh);
        ocp_nlp_dims_set_constraints(config_, dims_, i, "nbx", &nbx);
        ocp_nlp_dims_set_constraints(config_, dims_, i, "nbu", &nbu);
        ocp_nlp_dims_set_constraints(config_, dims_, i, "ng",  &ng);
        ocp_nlp_dims_set_constraints(config_, dims_, i, "nh",  &nh);

        int nsh; std::vector<int> dummy_idx; std::vector<double> dummy_d;
        constr.getNonlinearSoftInfoForStage(i, N_, nsh, dummy_idx, dummy_d, dummy_d, dummy_d, dummy_d);
        if (nsh > 0) {
            ocp_nlp_dims_set_constraints(config_, dims_, i, "nsh", &nsh);
        }
    }
}

// ============================================================
// set_constraints — 全部从 OcpConstraints 读取
// ============================================================
void MPCSolver::set_constraints(AcadosWrapper& wrapper, const OcpConstraints& constr) {
    for (int i = 0; i <= N_; i++) {
        std::vector<int> bx_idx; std::vector<double> bx_lb, bx_ub;
        constr.getStateBoundsForStage(i, N_, bx_idx, bx_lb, bx_ub);
        if (!bx_idx.empty()) {
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "idxbx", bx_idx.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "lbx", bx_lb.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "ubx", bx_ub.data());
        }

        std::vector<int> bu_idx; std::vector<double> bu_lb, bu_ub;
        constr.getInputBoundsForStage(i, N_, bu_idx, bu_lb, bu_ub);
        if (!bu_idx.empty()) {
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "idxbu", bu_idx.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "lbu", bu_lb.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "ubu", bu_ub.data());
        }

        auto lc = constr.getLinearForStage(i, N_, nx_, nu_);
        if (lc.ng > 0) {
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "C", lc.C.data());
            if (i < N_ && !lc.D.empty())
                ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "D", lc.D.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "lg", lc.lg.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "ug", lc.ug.data());
        }

        int nh = constr.getNhForStage(i, N_);
        if (nh > 0) {
            auto nls = constr.getNonlinearForStage(i, N_);
            std::vector<double> lh, uh;
            for (const auto& c : nls) {
                std::vector<double> sl, su;
                c->getBounds(sl, su);
                lh.insert(lh.end(), sl.begin(), sl.end());
                uh.insert(uh.end(), su.begin(), su.end());
            }
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "lh", lh.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "uh", uh.data());
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i,
                "nl_constr_h_fun", wrapper.getHFun(i));
            ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i,
                "nl_constr_h_fun_jac", wrapper.getHFunJacUx(i));
            
            // 挂载软约束索引
            int nsh; std::vector<int> idxsh; std::vector<double> dummy_d;
            constr.getNonlinearSoftInfoForStage(i, N_, nsh, idxsh, dummy_d, dummy_d, dummy_d, dummy_d);
            if (nsh > 0) {
                ocp_nlp_constraints_model_set(config_, dims_, in_, out_, i, "idxsh", idxsh.data());
            }
        }
    }
}

void MPCSolver::set_dynamics(AcadosWrapper& wrapper) {
    double Ts = cfg_.T / N_;
    for (int i = 0; i < N_; i++) {
        ocp_nlp_in_set(config_, dims_, in_, i, "Ts", &Ts);
        ocp_nlp_dynamics_model_set(config_, dims_, in_, i, "expl_ode_fun",  wrapper.getExplOdeFun(i));
        ocp_nlp_dynamics_model_set(config_, dims_, in_, i, "expl_vde_forw", wrapper.getExplVdeForw(i));
    }
}

// ★ 代价函数使用 nx_/nu_ 和 cfg_.W/W_e，不再依赖宏
// ==================== src/mpc_solver.cpp ====================
void MPCSolver::set_cost(AcadosWrapper& wrapper, const CostBase* cost, const OcpConstraints& constr) {
    int ny = cost ? cost->getNumStageResiduals() : nx_ + nu_;
    int nye = cost ? cost->getNumTerminalResiduals() : nx_;

    bool is_linear_cost = (cost == nullptr || cost->isLinear());
    printf("[MPC Cost] is_linear_cost = %s, ny = %d\n", is_linear_cost ? "TRUE" : "FALSE", ny);

    // --- 阶段权重 W ---
    std::vector<double> W(ny * ny, 0.0);
    for (int j = 0; j < ny && j < (int)cfg_.W.size(); j++) W[j + j * ny] = cfg_.W[j];

    // --- 终端权重 We ---
    std::vector<double> We(nye * nye, 0.0);
    for (int j = 0; j < nye && j < (int)cfg_.W_e.size(); j++) We[j + j * nye] = cfg_.W_e[j];

    // --- 构造默认的 Vx, Vu, Vxe 映射矩阵 (当没有自定义 cost 时使用) ---
    // Vx: ny × nx, identity 上半部分
    std::vector<double> default_Vx(ny * nx_, 0.0);
    for (int j = 0; j < nx_; j++) default_Vx[j + j * ny] = 1.0;

    // Vu: ny × nu, identity 下半部分
    std::vector<double> default_Vu(ny * nu_, 0.0);
    for (int j = 0; j < nu_; j++) default_Vu[(nx_ + j) + j * ny] = 1.0;

    // Vxe: nye × nx, identity
    std::vector<double> default_Vxe(nye * nx_, 0.0);
    for (int j = 0; j < nx_; j++) default_Vxe[j + j * nye] = 1.0;

    for (int i = 0; i <= N_; i++) {
        if (i < N_) {
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "W", W.data());
        
            if (is_linear_cost) {
                std::vector<double> Vx = cost ? cost->getVx() : default_Vx;
                std::vector<double> Vu = cost ? cost->getVu() : default_Vu;
                ocp_nlp_cost_model_set(config_, dims_, in_, i, "Vx", Vx.data());
                ocp_nlp_cost_model_set(config_, dims_, in_, i, "Vu", Vu.data());
            } else {
                // NONLINEAR_LS 模式，绑定由 AcadosWrapper 暴露出的 JIT 函数
                ocp_nlp_cost_model_set(config_, dims_, in_, i, "nl_cost_y_fun", wrapper.getCostYFun(i));
                ocp_nlp_cost_model_set(config_, dims_, in_, i, "nl_cost_y_fun_jac_ut_xt", wrapper.getCostYJac(i));
            }
        } else {

            // 终端节点
            ocp_nlp_cost_model_set(config_, dims_, in_, N_, "W", We.data());
            
            if (is_linear_cost) {
                std::vector<double> Vxe = cost ? cost->getVxe() : default_Vxe;
                ocp_nlp_cost_model_set(config_, dims_, in_, N_, "Vx", Vxe.data());
            } else {
                ocp_nlp_cost_model_set(config_, dims_, in_, N_, "nl_cost_y_fun", wrapper.getCostYEFun(N_));
                ocp_nlp_cost_model_set(config_, dims_, in_, N_, "nl_cost_y_fun_jac_x", wrapper.getCostYEJac(N_));
            }
        }
        // 挂载松弛变量的代价权重
        int nsh; std::vector<int> idxsh; std::vector<double> Zl, Zu, zl, zu;
        constr.getNonlinearSoftInfoForStage(i, N_, nsh, idxsh, Zl, Zu, zl, zu);
        if (nsh > 0) {
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "Zl", Zl.data());
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "Zu", Zu.data());
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "zl", zl.data());
            ocp_nlp_cost_model_set(config_, dims_, in_, i, "zu", zu.data());
        }
    }


}