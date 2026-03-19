#pragma once
#include <string>
#include <vector>

//包含 acados 的外部函数定义
extern "C" {
	#include "acados_c/external_function_interface.h"
}

class  AcadosWrapper {

public: 
	AcadosWrapper(int N, int np);
	~AcadosWrapper();

	void loadLibrary(const std::string& so_path, bool has_nonlinear_constraints = true, bool has_nonlinear_cost = false);

	// 获取特定 stage 的 acados 函数指针封装
    external_function_param_casadi* getExplOdeFun(int stage) { return &expl_ode_fun_[stage]; }
    external_function_param_casadi* getExplVdeForw(int stage) { return &expl_vde_forw_[stage]; }
    external_function_param_casadi* getHFun(int stage) { return &h_fun_[stage]; }
    external_function_param_casadi* getHFunJacUx(int stage) { return &h_fun_jac_u_x_[stage]; }

	external_function_param_casadi* getCostYFun(int stage) { return &cost_y_fun_[stage]; }
    external_function_param_casadi* getCostYJac(int stage) { return &cost_y_fun_jac_ut_xt_[stage]; }
    external_function_param_casadi* getCostYEFun(int stage) { return &cost_y_e_fun_[stage]; }
    external_function_param_casadi* getCostYEJac(int stage) { return &cost_y_e_fun_jac_x_[stage]; }

	external_function_param_casadi* getHEFun() { return &h_e_fun_; }
	external_function_param_casadi* getHEFunJacX() { return &h_e_fun_jac_x_; }

private:

	// 将指定名称的 CasADi C 函数绑定到 acados 的 struct 上
    void bindCasadiFunction(external_function_param_casadi& ext_fun, const std::string& fun_name);

	int N_;
	int np_;
	bool has_nl_ = false;
	void* handle_ = nullptr;

	std::vector<external_function_param_casadi> expl_ode_fun_;
	std::vector<external_function_param_casadi> expl_vde_forw_;
	std::vector<external_function_param_casadi> h_fun_;
	std::vector<external_function_param_casadi> h_fun_jac_u_x_;


	bool has_nl_cost_ = false;
    std::vector<external_function_param_casadi> cost_y_fun_;
    std::vector<external_function_param_casadi> cost_y_fun_jac_ut_xt_;
    std::vector<external_function_param_casadi> cost_y_e_fun_;
    std::vector<external_function_param_casadi> cost_y_e_fun_jac_x_;

	external_function_param_casadi h_e_fun_;
	external_function_param_casadi h_e_fun_jac_x_;
};
