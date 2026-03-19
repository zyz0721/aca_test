#pragma once
#include <string>
#include "ocp_core/SystemDynamicsBase.h"
#include "ocp_core/ConstraintBase.h"
#include "ocp_core/CostBase.h"

class ModelCompiler{

public:
	/**
     * @brief 提取模型并生成编译动态链接库 .so
     * @param dynamics 动力学派生类实例
     * @param constraints 约束派生类实例
     * @param nx 状态维度
     * @param nu 控制输入维度
     * @param np 运行时在线参数维度 (比如你的 dir, np=1)
     * @return std::string 生成的 .so 文件的绝对路径
     */
	static std::string compileAll(const SystemDynamicsBase& dynamics, 
								  const NonlinearConstraintBase* constraints,
                                          const CostBase* cost, 
								  int nx, int nu, int np);

};
