#pragma once

#include <memory>
#include <vector>
#include <string>
#include <numeric>

#include "ocp_core/SolverConfig.h"
#include "ocp_core/SystemDynamicsBase.h"
#include "ocp_core/ConstraintBase.h"
#include "ocp_core/ConstraintComposer.h"
#include "ocp_core/CostBase.h"
#include "mpc_backend/AcadosWrapper.h"
#include "mpc_backend/ModelCompiler.h"
#include "mpc_solver.h"


/**
 * @brief 统一的 OCP 问题构建器
 *
 * 用户只需通过此类定义优化问题，然后 build() 得到可用的求解器。
 * 底层的 CasADi 代码生成、GCC 编译、acados 配置全部自动完成。
 *
 * @code
 *   OcpProblem ocp(params);
 *
 *   // 动力学（必须）
 *   ocp.setDynamics<SwerveDynamics>(wheel_base, track_width);
 *
 *   // 约束（按需添加，每个约束指定作用阶段）
 *   using S = StageSelector;
 *   using B = BoundConstraintData;
 *
 *   ocp.addBound(S::initial(),         B{B::STATE, {0,1,2,3,4,5}, lo6, hi6});
 *   ocp.addBound(S::pathAndTerminal(), B{B::STATE, {3,4,5}, {-v,-v,-w}, {v,v,w}});
 *   ocp.addBound(S::path(),            B{B::INPUT, {0,1,2}, {-a,-a,-α}, {a,a,α}});
 *
 *   ocp.addNonlinear<SwerveConstraints>(S::pathAndTerminal(), wb, tw, smax, smin);
 *   // ocp.addNonlinear<ObstacleConstraint>(S::range(5, 15), 0.5);
 *
 *   // 代价（可选，默认使用 LINEAR_LS + identity Vx/Vu）
 *   // ocp.setCost<SwerveCost>();
 *
 *   // 构建
 *   auto solver = ocp.build();
 * @endcode
 */

class OcpProblem {
public:
	explicit OcpProblem(const SolverConfig& cfg) :cfg_(cfg) {}

	// 动力学
	template <typename T, typename... Args>
	void setDynamics(Args&&... args) {
		dynamics_ = std::make_shared<T>(std::forward<Args>(args)...);
	}

	void setDynamics(std::shared_ptr<SystemDynamicsBase> d) {
		dynamics_ = std::move(d);
        cfg_.nx = dynamics_->getNx();
        cfg_.nu = dynamics_->getNu();
        
        if(cfg_.nx <= 0 || cfg_.nu <= 0) {
            throw std::runtime_error("SystemDynamics must return valid nx and nu > 0");
        }
	}

	// 统一的addXxx(stages, data) 接口
	void addBound(StageSelector stages, BoundConstraintData data) {
        constr_.addBound(std::move(stages), std::move(data));
    }
	void addLinear(StageSelector stages, LinearConstraintData data) {
        constr_.addLinear(std::move(stages), std::move(data));
    }

    /// 模板版本：带软约束配置
    template<typename T, typename... Args>
    void addNonlinear(StageSelector stages, SoftPenalty penalty, Args&&... args) {
        constr_.addNonlinear(std::move(stages),
            std::make_shared<T>(std::forward<Args>(args)...), penalty);
    }


    /// 模板版本：兼容之前的硬约束调用
    template<typename T, typename... Args>
    void addNonlinear(StageSelector stages, Args&&... args) {
        constr_.addNonlinear(std::move(stages),
            std::make_shared<T>(std::forward<Args>(args)...), SoftPenalty{false});
    }
    void addNonlinear(StageSelector stages, std::shared_ptr<NonlinearConstraintBase> c, SoftPenalty penalty = SoftPenalty{}) {
        constr_.addNonlinear(std::move(stages), std::move(c), penalty);
    }
	// 代价
	template<typename T, typename... Args>
    void setCost(Args&&... args) {
        cost_ = std::make_shared<T>(std::forward<Args>(args)...);
    }

	std::unique_ptr<MPCSolver> build() { 
		if(!dynamics_) {
			std::cerr << "Error: dynamics not set" << std::endl;
			return nullptr;
		}

		int nx = dynamics_->getNx();
		int nu = dynamics_->getNu();
		int np = cfg_.np;
        if (nx > 0) cfg_.nx = nx;
        if (nu > 0) cfg_.nu = nu;

        if (constr_.hasNonlinear() && !constr_.isNonlinearUniformAcrossStages(cfg_.N)) {
            printf("Non-uniform nonlinear constraints across stages not yet supported. "
                          "All stages 1..N must have the same nh. \n");
            return nullptr;
        }

        // 注入全状态bound, 框架接管初始状态约束，set_x0() 会覆写这些 bound
        {
            std::vector<int> all_idx(nx);
            std::iota(all_idx.begin(), all_idx.end(), 0); 
            std::vector<double> lo(nx, 0.0), hi(nx, 0.0); // 初始值为 0, set_x0 会覆写
            constr_.addBound(StageSelector::initial(), BoundConstraintData{BoundConstraintData::STATE, all_idx, lo, hi});
        }

		// --- 组合非线性约束用于 JIT ---
        auto all_nl = constr_.getAllNonlinearConstraints();
        NonlinearConstraintBase* nl_for_jit = nullptr;
        std::shared_ptr<NonlinearConstraintBase> nl_holder;
	    if (all_nl.size() == 1) {
            nl_holder = all_nl[0];
            nl_for_jit = nl_holder.get();
        } else if (all_nl.size() > 1) {
            auto composer = std::make_shared<ConstraintComposer>();
            for (auto& c : all_nl) composer->add(c);
            nl_holder = composer;
            nl_for_jit = nl_holder.get();
        }

        bool has_nl_cost = (cost_ != nullptr && !cost_->isLinear());

	    // --- JIT 编译 ---
        std::cout << "[OcpProblem] Compiling model (nx= " << nx << "," 
				  << "nu=" << nu << ", np=" << np << std::endl;
        std::string so_path;
        try {
            so_path = ModelCompiler::compileAll(*dynamics_, nl_for_jit, cost_.get(), nx, nu, np);
        } catch (const std::exception& e) {
            printf("[OcpProblem] Compile failed: %s\n", e.what());
            return nullptr;
        }

	    // --- 加载 .so ---
        try {
            wrapper_ = std::make_unique<AcadosWrapper>(cfg_.N, np);
            wrapper_->loadLibrary(so_path, nl_for_jit != nullptr);
        } catch (const std::exception& e) {
            printf("[OcpProblem] Load failed: %s\n", e.what());
            return nullptr;
        }

        // --- 创建求解器 ---
        auto solver = std::make_unique<MPCSolver>(cfg_);
        auto strategy = SolverStrategyFactory::create(cfg_);
        if (!solver->setup(*wrapper_, constr_, cost_.get(), *strategy)) {
            printf("[OcpProblem] Solver setup failed!\n");
            return nullptr;
        }

		printf("[OcpProblem] Build OK. %zu bound groups, %zu linear groups, %zu nonlinear groups.\n",
                 constr_.getNumBoundGroups(), 
                 constr_.getNumLinearGroups(),
                 constr_.getNumNonlinearGroups());
        return solver;
	}
    std::unique_ptr<AcadosWrapper> takeWrapper() { return std::move(wrapper_); }
    const OcpConstraints& constraints() const { return constr_; }
private:
    SolverConfig cfg_;
    std::shared_ptr<SystemDynamicsBase> dynamics_;
    std::shared_ptr<CostBase> cost_;
    OcpConstraints constr_;
    std::unique_ptr<AcadosWrapper> wrapper_;
};