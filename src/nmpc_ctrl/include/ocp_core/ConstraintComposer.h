#pragma once
#include "ocp_core/ConstraintBase.h"
#include <vector>
#include <memory>
#include <numeric>

/**
 * @brief 非线性约束组合器
 *
 * 将多个 NonlinearConstraintBase 子类自动拼接为一个联合约束。
 * 本身也是 NonlinearConstraintBase，对 ModelCompiler 完全透明。
 *
 * 用法:
 *   ConstraintComposer composer;
 *   composer.add(std::make_shared<SwerveConstraints>(...));
 *   composer.add(std::make_shared<ObstacleConstraint>(...));
 *   ocp_constr.setNonlinearConstraint(
 *       std::make_shared<ConstraintComposer>(composer));
 */


class ConstraintComposer: public NonlinearConstraintBase { 

public:

 	ConstraintComposer() = default;
	void add(std::shared_ptr<NonlinearConstraintBase> constraint) {
		constraints_.push_back(std::move(constraint));
	}

	// 自动拼接所有约束的h向量

	casadi::MX computeConstraint(const casadi::MX& x, const casadi::MX&u, const casadi::MX& p) const override 
	{
		std::vector<casadi::MX> h_list;
		for (const auto& constraint : constraints_) {
			h_list.push_back(constraint->computeConstraint(x, u, p));
		}
		// 进行拼接
		return casadi::MX::vertcat(h_list);
	}

	std::string getName() const override {
        return "composed";
    }
	int getNumConstraints() const override {
		int total = 0;
		for (const auto& constraint : constraints_) {
			total += constraint->getNumConstraints();
		}
		return total;
	}

    /// 自动拼接所有子约束的边界
    void getBounds(std::vector<double>& lh, std::vector<double>& uh) const override {
		lh.clear();
		uh.clear();
		for (const auto& constraint : constraints_) {
			std::vector<double> sl, su;
			constraint->getBounds(sl, su);
			lh.insert(lh.end(), sl.begin(), sl.end());
			uh.insert(uh.end(), su.begin(), su.end());
		}
    }

private:
	std::vector<std::shared_ptr<NonlinearConstraintBase>> constraints_;

};