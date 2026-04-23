#include "galbot/singorix_wbcs/controller/galbot_s1/steer_nmpc.hpp"

using namespace std;
using namespace casadi;
SteeringNMPC::SteeringNMPC(int N, double dt, const VehicleConfig& config)
    : N_(static_cast<std::size_t>(N)), dt_(dt), cfg_(config) 
{
    // 初始化
    last_u_sol_.resize(N_ * 3, 0.0);
    last_x_sol_.resize((N_ + 1) * 6, 0.0);
    has_solution_ = false;
    // 建立一次优化
    setup_optimization();
    std::cout<< "NMPC param: "<<"N = "<< N_<<"  dt:  "<<dt_ <<std::endl;    
}

void SteeringNMPC::setup_optimization() 
{
    // 初始化 Opti 实例
    opti_ = Opti();
    // 决策变量
    X_ = opti_.variable(6, N_ + 1);  // 状态变量：x, y, yaw, vx, vy, dyaw
    U_ = opti_.variable(3, N_);      // 控制变量：ax, ay, ddyaw(线加速度,角加速度)

    std::size_t num_wheels = cfg_.wheel_positions.size();
    // 参数
    P_curr_ = opti_.parameter(6, 1); // 当前状态       
    P_ref_ = opti_.parameter(6, N_ + 1); // 参考轨迹
    P_direction_ = opti_.parameter(1, N_); // 运动方向参数
    Slacks_ = opti_.variable(num_wheels, N_);

    // 系统动力学 
    // 前向积分
    for (std::size_t k = 0; k < N_; ++k) {
        // RK4积分
        // MX k1 = f(X_(Slice(), k), U_(Slice(), k));
        // MX k2 = f(X_(Slice(), k) + dt_ / 2.0 * k1, U_(Slice(), k));
        // MX k3 = f(X_(Slice(), k) + dt_ / 2.0 * k2, U_(Slice(), k));
        // MX k4 = f(X_(Slice(), k) + dt_ * k3, U_(Slice(), k));
        // MX x_next = X_(Slice(), k) + dt_ / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4);
        // opti_.subject_to(X_(Slice(), k + 1) == x_next);

        // 欧拉前向积分
        MX pos = X_(Slice(0, 3), k); 
        MX vel = X_(Slice(3, 6), k);
        MX acc = U_(Slice(), k);     
        // 位移更新: p_next = p + v*t + 0.5*a*t^2
        MX pos_next = pos + vel * dt_ + 0.5 * acc * dt_ * dt_;
        // 速度更新: v_next = v + a*t
        MX vel_next = vel + acc * dt_;

        opti_.subject_to(X_(Slice(), k + 1) == MX::vertcat({pos_next, vel_next}));
    }

    // 设置约束
    // A. 初始状态约束
    opti_.subject_to(X_(Slice(), 0) == P_curr_);
    // B. 车体运动学硬约束
    for (std::size_t k = 0; k < N_; ++k) {
        // 加速度限制[这里用的平方根，转成线性MPC时不这样写]
        opti_.subject_to(MX::sumsqr(U_(Slice(0, 2), k)) <= pow(cfg_.max_acc, 2));
        // 角加速度限制
        opti_.subject_to(opti_.bounded(-cfg_.max_yaw_acc, U_(2, k), cfg_.max_yaw_acc));
    }    
    for (std::size_t k = 1; k <= N_; ++k) {
        // 速度限制
        opti_.subject_to(MX::sumsqr(X_(Slice(3, 5), k)) <= pow(cfg_.max_vel, 2));
        // 角速度限制
        opti_.subject_to(opti_.bounded(-cfg_.max_yaw_rate, X_(5, k), cfg_.max_yaw_rate));
    }

    // C. 四舵轮特定约束 
    double eps = 1e-6; // 数值稳定微小量

    // 扇形限制参数
    // 计算扇形的"中线角度" 
    double ang_mid = (cfg_.steer_lim_max + cfg_.steer_lim_min) / 2.0;
    double ang_half = (cfg_.steer_lim_max - cfg_.steer_lim_min) / 2.0;
    // 舵角速度约束
    double cos_rate = std::cos(cfg_.max_steer_rate * dt_);
    double cos_rate_sq = cos_rate * cos_rate; 

    for (std::size_t k = 0; k < N_; ++k) {
        // 提取 Yaw 角计算旋转矩阵
        MX yaw_curr = X_(2, k);
        MX yaw_next = X_(2, k+1);
        
        // 绕z轴旋转的逆矩阵(Global -> Body)
        // R^T = [ cos  sin]
        //       [-sin  cos]
        MX Rt_curr = MX::vertcat({
            MX::horzcat({ cos(yaw_curr), sin(yaw_curr)}),
            MX::horzcat({-sin(yaw_curr), cos(yaw_curr)})
        });
        MX Rt_next = MX::vertcat({
            MX::horzcat({ cos(yaw_next), sin(yaw_next)}),
            MX::horzcat({-sin(yaw_next), cos(yaw_next)})
        });

        MX dir_k = P_direction_(0, k);

        for (std::size_t i = 0; i < num_wheels; ++i) {
            // 获取全局轮速
            MX v_global_curr = get_global_wheel_velocity(X_(Slice(), k), i); // 当前周期
            MX v_global_next = get_global_wheel_velocity(X_(Slice(), k+1), i); //下一周期
            
            // 1. 轮速线速度限制 (全局坐标系下计算)
            // opti_.subject_to(MX::sumsqr(v_global_curr) <= pow(cfg_.max_wheel_vel, 2));

            // 舵轮速度 (转到体坐标系下)
            MX v_body_curr = MX::mtimes(Rt_curr, v_global_curr);
            MX v_body_next = MX::mtimes(Rt_next, v_global_next);

            // 2. 舵角极限约束
            /*****************舵角极限超过±90，只能用这种点积来做，没法用叉积****************/
            // [步骤A] 投影计算
            // 计算速度向量在"扇形中线"方向上的投影长度
            // 几何意义: v dot u_mid
            MX v_proj = v_body_next(0) * std::cos(ang_mid) + v_body_next(1) * std::sin(ang_mid);
            // [步骤B] 模长计算
            MX v_norm = MX::sqrt(MX::sumsqr(v_body_next) + eps);
            // [步骤C] 施加投影约束
            // 禁止指向正后方那一小块死区
            opti_.subject_to(dir_k * v_proj >= dir_k * dir_k * v_norm * std::cos(ang_half) - 1e-5);
            
            // 3. 舵角变化率约束
            MX dot_raw = v_body_curr(0) * v_body_next(0) + v_body_curr(1) * v_body_next(1);
            // MX norm_sq_curr = v_body_curr(0)*v_body_curr(0) + v_body_curr(1)*v_body_curr(1);
            // MX norm_sq_next = v_body_next(0)*v_body_next(0) + v_body_next(1)*v_body_next(1);

            MX norm_curr = MX::sqrt(MX::sumsqr(v_body_curr) + eps);
            MX norm_next = MX::sqrt(MX::sumsqr(v_body_next) + eps);

            MX dot_limit = cos_rate * norm_curr * norm_next;
            MX s = Slacks_(i, k);
            opti_.subject_to(dot_raw + s >= dot_limit);
            opti_.subject_to(s >= 0);

            //软约束
            // MX term_actual = MX::sq(dot_raw);               
            // MX term_limit  = cos_rate_sq * norm_sq_curr * norm_sq_next;
            // 计算违约量 violation
            // 如果 limit > actual，说明 cos(夹角)^2 太小 -> 夹角太大 -> 违约
            // 使用 fmax(0, x) 截断，只惩罚违约的部分
            // MX violation = MX::fmax(0.0, term_limit - term_actual);
            // 累积惩罚 (使用 sumsqr 保证光滑可导，利于 Ipopt 求解)
            // J_steer_rate_soft += cfg_.W_steer_rate * MX::sumsqr(violation);
            // 硬约束
            // 假设 max_steer_rate * dt < 90度 (通常只有几度) 
            // 施加约束 物理含义: |cos(theta)| >= |cos(limit)|
            // opti_.subject_to(MX::sq(dot_raw) >= cos_rate_sq * norm_sq_curr * norm_sq_next);
        }
    }

    // 6. 目标函数 (Cost Function)
    MX J = 0;
    // A 过程轨迹偏差
    for (std::size_t k = 0; k < N_; ++k) {
        // 状态轨迹偏差
        MX err = X_(Slice(), k) - P_ref_(Slice(), k);
        J += cfg_.Q_xy * MX::sumsqr(err(0));
        J += cfg_.Q_xy * MX::sumsqr(err(1));
        // 航向误差 (使用 1-cos 处理周期性)
        J += cfg_.Q_yaw * MX::sumsqr(1.0 - cos(err(2))); 
        J += cfg_.Q_vxy * MX::sumsqr(err(3));
        J += cfg_.Q_vxy * MX::sumsqr(err(4));
        J += cfg_.Q_dyaw * MX::sumsqr(err(5));
    }    
    // B 控制量平滑
    for (std::size_t k = 0; k < N_; ++k) {
        J += cfg_.R_axy * MX::sumsqr(U_(0, k));
        J += cfg_.R_axy * MX::sumsqr(U_(1, k));
        J += cfg_.R_ddyaw * MX::sumsqr(U_(2, k));
    }
    // C 终端代价
    MX term_err = X_(Slice(), N_) - P_ref_(Slice(), N_);    
    J += cfg_.Q_term_xy * (MX::sumsqr(term_err(0)) + MX::sumsqr(term_err(1)));  // 终端位置
    J += cfg_.Q_term_yaw * MX::sumsqr(1.0 - cos(term_err(2)));                  // 终端航向
    J += cfg_.Q_term_vxy * (MX::sumsqr(term_err(3)) + MX::sumsqr(term_err(4))); // 终端线速度
    J += cfg_.Q_term_ddyaw * MX::sumsqr(term_err(5));                           // 终端角速度

    // D 加入软约束
    // J += J_steer_rate_soft;
    J += cfg_.W_steer_rate * MX::sumsqr(Slacks_);

    opti_.minimize(J);

    // 配置求解器选项
    Dict p_opts;
    p_opts["expand"] = true; // 加速计算图，减少Ipopt对casadi表达式处理时间
    p_opts["print_time"] = 0; // 不输出casadi内部用时
    
    Dict s_opts;
    s_opts["max_iter"] = 100; // 最大迭代次数
    s_opts["print_level"] = 0; // 静默模式
    s_opts["sb"] = "yes"; // 减少打印信息
    s_opts["max_cpu_time"] = dt_ * N_; // 最大CPU时间

    // 收敛容差
    s_opts["tol"] = 1e-3;      // 主收敛容差
    s_opts["acceptable_tol"] = 1e-2; // 可接收收敛容差
    s_opts["acceptable_iter"] = 2; // 2次迭代满足acceptable_tol 返回

    // L-BFGS 拟牛顿法，不计算精确 Hessian
    s_opts["hessian_approximation"] = "limited-memory";
    // 开启热启动
    s_opts["warm_start_init_point"] = "yes"; 
    s_opts["mu_init"] = 0.001;       // 热启动时，初始 barrier 参数给小一点
    s_opts["warm_start_bound_push"] = 1e-9;
    s_opts["warm_start_mult_bound_push"] = 1e-9;

    // 线性求解器
    s_opts["linear_solver"] = "ma27";
    // 算法策略优化
    s_opts["mu_strategy"] = "adaptive";
    
    opti_.solver("ipopt", p_opts, s_opts);

}
// 执行优化
casadi::OptiSol SteeringNMPC::execute_optimization() 
{
    return opti_.solve();
}

// 根据车体状态计算单个轮子的全局速度矢量 (vx_global, vy_global)
MX SteeringNMPC::get_global_wheel_velocity(const MX& state, std::size_t wheel_idx) 
{
    // 当前状态[全局坐标系]
    MX yaw = state(2);
    MX vx = state(3);
    MX vy = state(4);
    MX dyaw = state(5);

    // 体坐标系下轮子位置
    MX wheel_pos_body = MX::vertcat({cfg_.wheel_positions[wheel_idx].first, cfg_.wheel_positions[wheel_idx].second});

    // 计算旋转矩阵对时间的导数 dR/dt
    MX dR = MX::vertcat({
        MX::horzcat({-sin(yaw), -cos(yaw)}),
        MX::horzcat({cos(yaw), -sin(yaw)})
    }) * dyaw;
    // 全局坐标系下车体速度
    MX v_body_global = MX::vertcat({vx, vy});
    MX v_wheel_global = v_body_global + MX::mtimes(dR, wheel_pos_body); // MX::mtimes：矩阵乘法接口
    
    return v_wheel_global; // 返回 2x1 向量
}

std::pair<double, double> SteeringNMPC::compute_wheel_vel_numeric(
        const std::vector<double>& state, std::size_t wheel_idx) 
{
    double yaw = state[2];
    double vx = state[3];
    double vy = state[4];
    double dyaw = state[5];
    
    double lx = cfg_.wheel_positions[wheel_idx].first;
    double ly = cfg_.wheel_positions[wheel_idx].second;
    
    // dR/dt = [-sin -cos; cos -sin] * dyaw
    double dR00 = -sin(yaw) * dyaw; double dR01 = -cos(yaw) * dyaw;
    double dR10 = cos(yaw) * dyaw;  double dR11 = -sin(yaw) * dyaw;
    
    double vx_w = vx + (dR00 * lx + dR01 * ly);
    double vy_w = vy + (dR10 * lx + dR11 * ly);
    
    return {vx_w, vy_w};
}
void SteeringNMPC::set_reference_trajectory(const std::vector<std::vector<double>>& ref_traj,
                                            const std::vector<double>& current_state) 
{
    if (ref_traj.size() != static_cast<std::size_t>(N_ + 1)) {
        throw std::runtime_error("Input trajectory length does not match MPC horizon N");
    }

    // 1. 设置当前状态参数 (P_curr) 这是当前状态
    opti_.set_value(P_curr_, current_state);

    // 2. 设置参考轨迹参数 (P_ref)
    // 将 std::vector 转换为 DM (6 x N+1)
    DM ref_dm(6, N_ + 1);
    for (std::size_t k = 0; k <= N_; ++k) {
        for (std::size_t i = 0; i < 6; ++i) ref_dm(i, k) = ref_traj[k][i];
    }
    opti_.set_value(P_ref_, ref_dm);

    // 3. 设置热启动初值
    if (has_solution_) {
        DM x_init = DM::zeros(6, N_ + 1);
        for (std::size_t k = 0; k < N_; ++k)
            for (std::size_t i = 0; i < 6; ++i)
                x_init(i, k) = last_x_sol_[(k + 1) * 6 + i];
        // 终端
        for (std::size_t i = 0; i < 6; ++i){
            double val_N   = last_x_sol_[N_ * 6 + i];       // 上次最后一个点
            double val_N_1 = last_x_sol_[(N_ - 1) * 6 + i]; // 上次倒数第二个点
            x_init(i, N_)  = val_N + (val_N - val_N_1);
        }            
        opti_.set_initial(X_, x_init);

        DM u_init = DM::zeros(3, N_);
        for (std::size_t k = 0; k < N_ - 1; ++k)
            for (std::size_t i = 0; i < 3; ++i)
                u_init(i, k) = last_u_sol_[(k + 1) * 3 + i];
        for (std::size_t i = 0; i < 3; ++i)
            u_init(i, N_ - 1) = last_u_sol_[(N_ - 1) * 3 + i];
        opti_.set_initial(U_, u_init);
    } else {
        // 直接用参考轨迹冷启动
        opti_.set_initial(X_, ref_dm);
        opti_.set_initial(U_, 0.0);
        opti_.set_initial(opti_.lam_g(), 0); // 强制重置对偶变量
    }
    opti_.set_initial(Slacks_, 0.0);
}
// mpc控制
MpcResult SteeringNMPC::control(const std::vector<std::vector<double>>& ref_traj,
                                const std::vector<double>& current_state) 
{
    MpcResult result;
    result.ref_traj = ref_traj;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool optimization_success = false;

    try {
        if (ref_traj.size() != static_cast<std::size_t>(N_ + 1)) {
            throw std::runtime_error("Initial guess length mismatch");
        }
        motion_direction_ = compute_forward_direction(ref_traj);
        // std::cout<<" motion_direction_: " << motion_direction_ <<std::endl;
        // 运动方向不单调
        if(-2 == motion_direction_){
            throw std::runtime_error("reference trajectory is not valid");
        }
        else{
            motion_direction_vec_.assign(N_,static_cast<double>(motion_direction_));
            opti_.set_value(P_direction_, motion_direction_vec_);
        }
        /*****************打印参考轨迹***************/
        // if(!has_solution_){
        //     std::cout<<"init state:"<<std::endl;
        //     for(int i = 0;i<current_state.size();i++)
        //         std::cout<<current_state[i]<<" ";
        //     std::cout<<std::endl;

        //     std::cout << "ref_traj: " << std::endl;

        //     for(int i = 0; i < ref_traj.size(); i++)
        //     {
        //         for(int j = 0; j < 6; j++)
        //         {
        //             std::cout<< ref_traj[i][j] << " ";
        //         }
        //         std::cout<<std::endl;
        //     }
        // }
        
        
        // A. 设置参考轨迹
        set_reference_trajectory(ref_traj,current_state);
        // B. 执行求解 
        casadi::OptiSol sol = execute_optimization();
        // C. 检验
        std::string status = sol.stats().at("return_status");
        if (status == "Solve_Succeeded" || status == "Solved_To_Acceptable_Level") {
            optimization_success = true;            
            // 提取结果用于保存
            DM x_val = sol.value(X_);
            DM u_val = sol.value(U_);
            
            last_x_sol_ = std::vector<double>(x_val);
            last_u_sol_ = std::vector<double>(u_val);
            has_solution_ = true;
        } else {
            std::cerr << "[MPC] Status: " << status << ". Using Initial Guess." << std::endl;
            optimization_success = false;
            has_solution_ = false;
        }

    } catch (const std::exception& e) {
        std::cerr << "[MPC] Crash: " << e.what() << ". Using Initial Guess." << std::endl;
        optimization_success = false;
        has_solution_ = false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.opt_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    // std::cout<<"nmpc excute time: "<<result.opt_time_ms<<" ms"<<std::endl;

    result.status = optimization_success;
    // 4. 数据填充
    result.result_traj.reserve(N_); 
    result.out_control.reserve(N_);
    std::size_t num_wheels = 4;

    if (optimization_success) {
        // CasADi 列优先: last_x_sol_ [k*6 + i]
        for (std::size_t k = 0; k <= N_; ++k) {
            std::vector<double> state_row(14, 0.0);
            for (std::size_t i = 0; i < 6; ++i) {
                state_row[i] = last_x_sol_[k * 6 + i]; 
            }            
            // 轮子状态
            std::vector<double> base_state(6);
            for(std::size_t i=0; i<6; ++i) base_state[i] = state_row[i];

            for (std::size_t w = 0; w < num_wheels; ++w) {
                auto v_wheel = compute_wheel_vel_numeric(base_state, w);
                state_row[6 + w] = v_wheel.first;
                state_row[10 + w] = v_wheel.second;
            }
            result.result_traj.push_back(state_row);
        }

        for (std::size_t k = 0; k <= N_; ++k) {
            std::vector<double> ctrl_row(4, 0.0);
            // 取出第 k+1 步的全局速度 (优化器算出来的期望速度)
            double vx_global_next = last_x_sol_[k * 6 + 3];
            double vy_global_next = last_x_sol_[k * 6 + 4];
            double omega_next     = last_x_sol_[k * 6 + 5]; // 角速度            
            
            // 坐标变换：全局速度 -> 体坐标系速度
            double yaw_next = last_x_sol_[k * 6 + 2];
            double vx_body =  cos(yaw_next) * vx_global_next + sin(yaw_next) * vy_global_next;
            double vy_body = -sin(yaw_next) * vx_global_next + cos(yaw_next) * vy_global_next;

            ctrl_row[0] = vx_body; 
            ctrl_row[1] = vy_body;
            ctrl_row[2] = omega_next;
            ctrl_row[3] = dt_;
            
            result.out_control.push_back(ctrl_row);

            // std::cout<< last_x_sol_[k * 6 + 0]<<" "
            //          << last_x_sol_[k * 6 + 1]<<" " 
            //          << last_x_sol_[k * 6 + 2]<<" "
            //          << last_x_sol_[k * 6 + 3]<<" "
            //          << last_x_sol_[k * 6 + 4]<<" "
            //          << last_x_sol_[k * 6 + 5]<<" " <<std::endl;
        }
        // std::cout<<"output roll horzon"<<std::endl;
    } else {
        // --- 失败：降级Initial Guess ---
        for (std::size_t k = 0; k <= N_; ++k) {
            std::vector<double> state_row(14, 0.0);
            int dim_in = static_cast<int>(ref_traj[k].size()); // 强制转换避免警告
            
            for (int i = 0; i < std::min(6, dim_in); ++i) {
                state_row[i] = ref_traj[k][i];
            }
            
            std::vector<double> base_state(6, 0.0);
            for(int i=0; i<std::min(6, dim_in); ++i) base_state[i] = state_row[i];

            for (std::size_t w = 0; w < num_wheels; ++w) {
                auto v_wheel = compute_wheel_vel_numeric(base_state, w);
                state_row[6 + w] = v_wheel.first;
                state_row[10 + w] = v_wheel.second;
            }
            result.result_traj.push_back(state_row);
        }

        for (std::size_t k = 0; k <= N_; ++k) {
            result.out_control.push_back({0.0, 0.0, 0.0, dt_});
        }
    }

    return result;
}
// 计算前进方向
int SteeringNMPC::compute_forward_direction(const std::vector<std::vector<double>>& ref_traj)
{
    int first_direction = 0; 
    const double v_eps = 1e-2; 

    for (size_t k = 0; k < ref_traj.size(); ++k) {
        // ref_traj 结构为: [x, y, yaw, vx_global, vy_global, dyaw]
        double yaw = ref_traj[k][2];
        double vx_global = ref_traj[k][3];
        double vy_global = ref_traj[k][4];

        // 投影到体坐标系
        double v_body_x = vx_global * std::cos(yaw) + vy_global * std::sin(yaw);
        // std::cout << "v_body_x: " << v_body_x << std::endl;
        // 判断当前点的方向
        int current_dir = 0;
        if (v_body_x > v_eps) {
            current_dir = 1;  // 前进
        } else if (v_body_x < -v_eps) {
            current_dir = -1; // 后退
        }
        // else: 速度太小，认为是静止，不改变方向判定

        if (current_dir != 0) {
            if (first_direction == 0) {
                first_direction = current_dir;
            } 
            // 如果当前方向与初始方向不一致，说明中间有反向
            else if (current_dir != first_direction) {
                // 直接返回 0
                return -2; 
            }
        }
    }

    return first_direction;
}
// 重置时，清理热启动
void SteeringNMPC::reset()
{
    last_u_sol_.resize(N_ * 3, 0.0);
    last_x_sol_.resize((N_ + 1) * 6, 0.0);
    has_solution_ = false;

    std::cout<<"reset"<<std::endl;
}

