启动脚本

cd catkin_ws

roslaunch nmpc_ctrl gazebo_mpc.launch



### 核心思想

绕过 acados 的 Python codegen 管线，在纯 C++ 中完成"模型定义 → 符号图构建 → JIT 编译 → 求解器绑定"的全流程。

### 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                        编译期 (启动时一次性)                          │
│                                                                     │
│  SwerveDynamics ──┐                                                 │
│  SwerveConstraints ──→ ModelCompiler::compileAll()                  │
│  (SwerveCost)     ──┘       │                                       │
│                        CasADi C++ API                               │
│                        构建符号计算图                                 │
│                             │                                       │
│                        CodeGenerator                                │
│                        生成 jit_cg_model.c                           │
│                             │                                       │
│                        gcc -fPIC -shared -O3                        │
│                        → libjit_cg_model.so                         │
│                             │                                       │
│                   AcadosWrapper::loadLibrary()                      │
│                   dlopen + dlsym × 6 指针 × 4 函数 × (N+1) stages   │
│                   external_function_param_casadi_create()            │
│                             │                                       │
│                   MPCSolver::setup()                                 │
│                   ocp_nlp_* C API 配置维度/代价/约束/动力学           │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                     运行期 (每个控制周期)                             │
│                                                                     │
│  set_x0 → set_yref → set_online_parameter → solve → get_u/get_x    │
└─────────────────────────────────────────────────────────────────────┘
```

### 三层抽象

| 层级 | 组件 | 职责 |
|------|------|------|
| **模型层** | `SystemDynamicsBase`, `ConstraintBase`, `CostBase` | 用户只需实现纯虚函数，用 CasADi MX 定义符号表达式 |
| **编译层** | `ModelCompiler`, `AcadosWrapper` | 符号图 → C 代码 → .so → dlsym 绑定到 acados 结构体 |
| **求解层** | `MPCSolver` | 配置 acados ocp_nlp C 接口，执行 SQP_RTI |
