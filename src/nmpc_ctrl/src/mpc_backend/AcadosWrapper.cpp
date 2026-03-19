#include "mpc_backend/AcadosWrapper.h"
#include <dlfcn.h>
#include <stdexcept>
#include <iostream>
#include <cstring>   // 使用memset

AcadosWrapper::AcadosWrapper(int N, int np): N_(N), np_(np) {
    // 动力学只有0-N-1
    expl_ode_fun_.resize(N_);
    expl_vde_forw_.resize(N_);
    // 约束和代价不仅有0-N-1，还有第N个终端阶段
    h_fun_.resize(N_ + 1);
    h_fun_jac_u_x_.resize(N_ + 1);

    // 初始化代价函数数组
    cost_y_fun_.resize(N_ + 1);
    cost_y_fun_jac_ut_xt_.resize(N_ + 1);
    cost_y_e_fun_.resize(N_ + 1);
    cost_y_e_fun_jac_x_.resize(N_ + 1);
}

AcadosWrapper::~AcadosWrapper() {
    // 释放 acados 内部为参数分配的内存
    for (int i = 0; i < N_; i++) {
        external_function_param_casadi_free(&expl_ode_fun_[i]);
        external_function_param_casadi_free(&expl_vde_forw_[i]);
    }
    if (has_nl_) {
        for (int i = 0; i < N_ + 1; i++) {
            external_function_param_casadi_free(&h_fun_[i]);
            external_function_param_casadi_free(&h_fun_jac_u_x_[i]);
        }
    }

    if (has_nl_cost_) {
        for (int i = 0; i < N_; i++) {
            external_function_param_casadi_free(&cost_y_fun_[i]);
            external_function_param_casadi_free(&cost_y_fun_jac_ut_xt_[i]);
        }
        external_function_param_casadi_free(&cost_y_e_fun_[N_]);
        external_function_param_casadi_free(&cost_y_e_fun_jac_x_[N_]);
    }

    // 卸载动态链接库
    if (handle_) dlclose(handle_);
}

void AcadosWrapper::loadLibrary(const std::string& so_path, bool has_nonlinear_constraints, bool has_nonlinear_cost) {
    has_nl_ = has_nonlinear_constraints;
    has_nl_cost_ = has_nonlinear_cost;   // 保存状态

    handle_ = dlopen(so_path.c_str(), RTLD_LAZY);
    if (!handle_) {
        throw std::runtime_error("Failed to load JIT model library: " + std::string(dlerror()));
    }
    printf("[AcadosWrapper] dlopen OK. N=%d, np=%d \n", N_, np_);

    // 动力学函数
    for (int i = 0; i < N_; i++) {
        bindCasadiFunction(expl_ode_fun_[i],  "expl_ode_fun");
        bindCasadiFunction(expl_vde_forw_[i], "expl_vde_forw");
    }
    // 非线性约束函数(可选)
    if (has_nl_) {
        for (int i = 0; i < N_+1; i++) {
            bindCasadiFunction(h_fun_[i],         "h_fun");
            bindCasadiFunction(h_fun_jac_u_x_[i], "h_fun_jac_u_x");
        }
    }

    // 非线性代价函数绑定
    if (has_nl_cost_) {
        for (int i = 0; i < N_; i++) {
            bindCasadiFunction(cost_y_fun_[i], "cost_y_fun");
            bindCasadiFunction(cost_y_fun_jac_ut_xt_[i], "cost_y_fun_jac_ut_xt");
        }
        // 终端代价
        bindCasadiFunction(cost_y_e_fun_[N_], "cost_y_e_fun");
        bindCasadiFunction(cost_y_e_fun_jac_x_[N_], "cost_y_e_fun_jac_rt"); // 注意这里对齐编译生成的名字
    }


    printf("[AcadosWrapper] All stages loaded successfully. \n");
}
void AcadosWrapper::bindCasadiFunction(external_function_param_casadi& ext_fun,
                                        const std::string& fun_name) {
    std::memset(&ext_fun, 0, sizeof(external_function_param_casadi));

    typedef int (*casadi_fun_t)(const double**, double**, int*, double*, void*);
    typedef int (*casadi_work_t)(int*, int*, int*, int*);
    typedef const int* (*casadi_sparsity_t)(int);
    typedef int (*casadi_n_in_t)(void);
    typedef int (*casadi_n_out_t)(void);

    ext_fun.casadi_fun          = (casadi_fun_t)      dlsym(handle_, fun_name.c_str());
    ext_fun.casadi_work         = (casadi_work_t)     dlsym(handle_, (fun_name + "_work").c_str());
    ext_fun.casadi_n_in         = (casadi_n_in_t)     dlsym(handle_, (fun_name + "_n_in").c_str());
    ext_fun.casadi_n_out        = (casadi_n_out_t)    dlsym(handle_, (fun_name + "_n_out").c_str());
    ext_fun.casadi_sparsity_in  = (casadi_sparsity_t) dlsym(handle_, (fun_name + "_sparsity_in").c_str());
    ext_fun.casadi_sparsity_out = (casadi_sparsity_t) dlsym(handle_, (fun_name + "_sparsity_out").c_str());

    // 全部 6 个指针的 NULL 检查
    if (!ext_fun.casadi_fun)          throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name);
    if (!ext_fun.casadi_work)         throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name + "_work");
    if (!ext_fun.casadi_n_in)         throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name + "_n_in");
    if (!ext_fun.casadi_n_out)        throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name + "_n_out");
    if (!ext_fun.casadi_sparsity_in)  throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name + "_sparsity_in");
    if (!ext_fun.casadi_sparsity_out) throw std::runtime_error("[AcadosWrapper] dlsym failed: " + fun_name + "_sparsity_out");

    // external_function_param_casadi_create 的第三个参数 opts_ 不能传 nullptr！
    // calculate_size() 内部会直接解引用 opts_->with_global_data，
    // 传 nullptr 会导致段错误。
    // 必须传一个默认初始化（全零）的 external_function_opts 结构体。
    external_function_opts opts;
    std::memset(&opts, 0, sizeof(external_function_opts));
    external_function_param_casadi_create(&ext_fun, np_, &opts);
}