#include "mpc_backend/ModelCompiler.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <functional>
#include <iostream>
#include <cstdlib>

using namespace casadi;


// 创建持久化缓存目录
static void createCacheDir(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        mkdir(path.c_str(), 0777); // 目录不存在则创建
    }
}

// 检查文件是否存在
static bool fileExists(const std::string& name) {
    struct stat buffer;   
    return (stat(name.c_str(), &buffer) == 0); 
}


std::string ModelCompiler::compileAll(const SystemDynamicsBase& dynamics, 
									  const NonlinearConstraintBase* constraints,
                                      const CostBase* cost, 
									  int nx, int nu, int np)
{ 
	MX x = MX::sym("x", nx);
	MX u = MX::sym("u", nu);
	MX p = MX::sym("p", np);  //运行时的参数，比如运动方向

	// 动力学方程与变分方程（VDE）计算图建立
	MX flow_map = dynamics.computeFlowMap(x, u);
	Function expl_ode("expl_ode_fun", {x, u, p}, {flow_map});

	// 变分方程(VDE) 这里待会看下！！！
	MX Sx = MX::sym("Sx", nx, nx);
	MX Su = MX::sym("Su", nx, nu);
	MX jac_x = jacobian(flow_map, x);
	MX jac_u = jacobian(flow_map, u);
	MX Sx_dot = mtimes(jac_x, Sx);
	MX Su_dot = mtimes(jac_x, Su) + jac_u;
	// in: [x, Sx, Su, u, p]   out: [f, Sx_dot, Su_dot]
    Function expl_vde_forw("expl_vde_forw", {x, Sx, Su, u, p}, {flow_map, Sx_dot, Su_dot});

	// 设置缓存目录
    std::string cache_dir = "/var/tmp/nmpc_cache";
    createCacheDir(cache_dir);
    std::string base_name = dynamics.getModelName() + "_cg";

	// 设置代码生成选项
    Dict opts;
    opts["casadi_real"] = "double";
    opts["casadi_int"] = "int";

    // 合并到一个 C 文件中
    CodeGenerator gen(base_name, opts);
    gen.add(expl_ode);
    gen.add(expl_vde_forw);

    // ========== 非线性约束（可选） ==========
    if (constraints != nullptr) {
		// 非线性图计算图构建
        MX h = constraints->computeConstraint(x, u, p);
        Function h_fun("h_fun", {x, u, p}, {h});
		// Acados 要求的约束雅可比函数
    	// 注意：acados 要求对 [u; x] 联合求导，并且输出的是雅可比的转置 (Transpose)
        MX ux = MX::vertcat({u, x});
        MX jac_ux = jacobian(h, ux);
		// in: [x, u, p]  out: [h, J_ux^T]
        Function h_fun_jac_u_x("h_fun_jac_u_x", {x, u, p}, {h, jac_ux.T()});

        gen.add(h_fun);
        gen.add(h_fun_jac_u_x);

        // ========== 终端阶段约束（不依赖 u）==========
        MX u_zero = MX::zeros(nu, 1);
        MX h_e = constraints->computeConstraint(x, u_zero, p);
        Function h_e_fun("h_e_fun", {x, p}, {h_e});
        MX jac_h_e_x = jacobian(h_e, x);
        Function h_e_fun_jac_x("h_e_fun_jac_x", {x, p}, {h_e, jac_h_e_x.T()});
        gen.add(h_e_fun);
        gen.add(h_e_fun_jac_x);
    }
    // 非线性代价函数生成
    if (cost != nullptr && !cost->isLinear()) {
        // 阶段代价
        MX y = cost->computeStageCostResidual(x, u);
        Function cost_y_fun("cost_y_fun", {x, u, p}, {y});
        
        // Acados 要求的形式: [y, J_ux^T]
        MX ux = MX::vertcat({u, x});
        MX jac_y_ux = jacobian(y, ux);
        Function cost_y_fun_jac_ut_xt("cost_y_fun_jac_ut_xt", {x, u, p}, {y, jac_y_ux.T()});

        // 终端代价 (只依赖 x)
        MX ye = cost->computeTerminalCostResidual(x);
        Function cost_y_e_fun("cost_y_e_fun", {x, p}, {ye});
        
        MX jac_ye_x = jacobian(ye, x);
        Function cost_y_e_fun_jac_x("cost_y_e_fun_jac_rt", {x, p}, {ye, jac_ye_x.T()});

        gen.add(cost_y_fun);
        gen.add(cost_y_fun_jac_ut_xt);
        gen.add(cost_y_e_fun);
        gen.add(cost_y_e_fun_jac_x);
    }



    // 先将 C 代码生成到缓存目录
    gen.generate(cache_dir + "/");
    std::string c_file_path = cache_dir + "/" + base_name + ".c";

    // 读取生成的 C 代码并计算 Hash 
    // (这完美反映了 nx, nu, np 以及所有底层数学公式的任何变更)
    std::ifstream ifs(c_file_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("[ModelCompiler] Failed to read generated C file.");
    }
    std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    ifs.close();

    std::hash<std::string> hasher;
    size_t hash_val = hasher(content);

    std::string so_file_path = cache_dir + "/lib_" + dynamics.getModelName() + "_" + std::to_string(hash_val) + ".so";

    // AOT 缓存命中检查
    if (fileExists(so_file_path)) {
        std::cout << "[ModelCompiler] AOT Hit! Found cached model hash: " << hash_val << std::endl;
        std::cout << "[ModelCompiler] Skipping GCC compilation. Direct dlopen." << std::endl;
        return so_file_path;
    }
    
    // JIT 编译回退 (缓存未命中或初次运行)
    std::cout << "[ModelCompiler] JIT Compiling generated C code (Hash: " << hash_val << ")..." << std::endl;
    std::string cmd = "gcc -fPIC -shared -O3 " + c_file_path + " -o " + so_file_path + " -lm";
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        throw std::runtime_error("[ModelCompiler] GCC JIT Compilation failed! Ensure 'gcc' is installed.");
    }
    std::cout << "[ModelCompiler] Successfully compiled to " << so_file_path << std::endl;

    return so_file_path;
}