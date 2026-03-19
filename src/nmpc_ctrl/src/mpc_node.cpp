// ============================================================
// mpc_node.cpp — 使用统一 OCP 框架的示例
//
// 底层的 CasADi 编译、acados 配置全部由框架自动完成。
// ============================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// ★ 框架头文件
#include "ocp_core/OcpProblem.h"

// ★ 机器人特定的模型文件
#include "robot_models/SwerveDynamics.h"
#include "robot_models/SwerveContraints.h"
// #include "robot_models/ObstacleConstraint.h"

#include "mpc_visualizer.h"
#include "reference_generator.h"
#include "steering_ik.h"
static double quat_to_yaw(double qx, double qy, double qz, double qw) {
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

static void global_to_body(double yaw, double vx_g, double vy_g,
                           double &vx_b, double &vy_b) {
    double c = cos(yaw), s = sin(yaw);
    vx_b =  c * vx_g + s * vy_g;
    vy_b = -s * vx_g + c * vy_g;
}

static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static bool solution_valid(MPCSolver &solver) {
    int nx = solver.get_x_dimension();
    int nu = solver.get_u_dimension();
    std::vector<double> buf(nx);
    for (int i = 0; i <= solver.N(); i++) {
        solver.get_x(i, buf.data());
        for (int k = 0; k < nx; k++)
            if (!std::isfinite(buf[k])) return false;
    }
    std::vector<double> u(nu);
    for (int i = 0; i < solver.N(); i++) {
        solver.get_u(i, u.data());
        for (int k = 0; k < nu; k++)
            if (!std::isfinite(u[k])) return false;
    }
    return true;
}

// ============================================================
static void warm_start(MPCSolver &solver, const ReferenceGenerator &ref_gen,
                       std::vector<double>& state, const MpcParams &p)
{
    double dt_h = p.solver_cfg.T / p.solver_cfg.N;
    std::vector<double> guess_x = state;
    std::vector<double> zero_u(p.solver_cfg.nu, 0.0);
    for (int k = 0; k < p.solver_cfg.nx; k++) guess_x[k] = state[k];

    for (int i = 0; i <= p.solver_cfg.N; i++) {
        solver.init_x(i, guess_x.data());
        if (i < p.solver_cfg.N) {
            solver.init_u(i, zero_u.data());
        }
        guess_x[0] += guess_x[3] * dt_h;
        guess_x[1] += guess_x[4] * dt_h;
        guess_x[2] += guess_x[5] * dt_h;

        RefState ref = ref_gen.at(i * dt_h);
        if (i < p.solver_cfg.N) {
            std::vector<double> yref(p.solver_cfg.nx + p.solver_cfg.nu);
            // 进行拼接
            std::copy(ref.val.begin(), ref.val.end(), yref.begin());
            std::copy(ref.u_ref.begin(), ref.u_ref.end(), yref.begin() + p.solver_cfg.nx);
            solver.set_yref(i, yref.data());
        } else {
            solver.set_yref(i, ref.val.data());
        }
    }
    solver.set_x0(state.data());

    for (int w = 0; w < 5; w++) {
        int st = solver.solve();
        printf("[WARM] iter %d: status=%d", w, st);
        if (st == 0) break;
        if (!solution_valid(solver)) break;
    }
}

// 移位热启动，将上一步计算的最优控制输入作为当前时刻迭代的初始
static void shift_warm_start(MPCSolver &solver, int N) {
    // 目的：减少迭代次数，避免局部最优，保证实时性
    std::vector<double> x_buf(solver.get_x_dimension());
    std::vector<double> u_buf(solver.get_u_dimension());
    for (int i = 0; i < N; i++) {
        solver.get_x(i + 1, x_buf.data()); // 解析出上一次的x[i+1]
        solver.init_x(i, x_buf.data());    // 作为当前的x[i]
    }
    solver.get_x(N, x_buf.data());         // 解析出上一次迭代的x[N]
    solver.init_x(N, x_buf.data());        // 当前时刻的末端状态保持与上一次的x[N]一致

    // 控制变量的移位，控制量比状态量少一个
    for (int i = 0; i < N - 1; i++) {
        solver.get_u(i + 1, u_buf.data());
        solver.init_u(i, u_buf.data());
    }
    solver.get_u(N - 1, u_buf.data());
    solver.init_u(N - 1, u_buf.data());
}

// ============================================================
class MpcNode {
public:
    MpcNode(ros::NodeHandle &nh) : nh_(nh) {
        params_.load(nh_);
        ik_.init(params_);

        // ===========================================================
        // ★★★ OCP 问题定义 — 用户只需修改这一段 ★★★
        // ===========================================================
        using S = StageSelector;
        using B = BoundConstraintData;

        OcpProblem ocp(params_.solver_cfg);
        // 1. 动力学
        ocp.setDynamics<SwerveDynamics>(params_.wheel_base, params_.track_width);

        // 2. 边界约束
        // stage 0 的初始约束由框架自动管理，无需手动添加
        ocp.addBound(S::pathAndTerminal(), B{B::STATE,                 // 路径+终端: 速度限制
            {3, 4, 5},
            {-params_.max_vel, -params_.max_vel, -params_.max_yaw_rate},
            { params_.max_vel,  params_.max_vel,  params_.max_yaw_rate}});

        ocp.addBound(S::path(), B{B::INPUT,                            // 路径: 加速度限制
            {0, 1, 2},
            {-params_.max_acc, -params_.max_acc, -params_.max_yaw_acc},
            { params_.max_acc,  params_.max_acc,  params_.max_yaw_acc}});

        // 3. 非线性约束
        ocp.addNonlinear<SwerveConstraints>(S::pathAndTerminal(),      // 舵轮约束
            params_.wheel_base, params_.track_width,
            params_.steer_lim_max, params_.steer_lim_min);
        
        // 4. 构建
        solver_ = ocp.build();
        nx_ = solver_->get_x_dimension(); 
        nu_ = solver_->get_u_dimension();

        state_.resize(nx_, 0.0);
        // 将 params_.x0 中的初始值安全地拷贝进去，拷贝长度取两者中较小的一个
        int copy_len = std::min(nx_, (int)params_.x0.size());
        std::copy(params_.x0.begin(), params_.x0.begin() + copy_len, state_.begin());


        ref_gen_.init(params_, nx_, nu_);
        if (!solver_) { ros::shutdown(); return; }
        wrapper_ = ocp.takeWrapper();

        viz_ = std::make_unique<MpcVisualizer>(nh_, params_);

        if (params_.sim_mode) {
            printf("=== SIM MODE ===\n");
            odom_received_ = true;
        } else {
            printf("=== GAZEBO MODE: odom=%s ===\n", params_.odom_topic.c_str());
            odom_sub_ = nh_.subscribe(params_.odom_topic, 1, &MpcNode::odom_cb, this);
            odom_received_ = false;
        }

        std::string steer_names[] = {"fl_steer", "fr_steer", "rl_steer", "rr_steer"};
        std::string drive_names[] = {"fl_drive", "fr_drive", "rl_drive", "rr_drive"};
        for (int i = 0; i < NUM_WHEELS; i++) {
            steer_pub_[i] = nh_.advertise<std_msgs::Float64>(
                "/" + steer_names[i] + "_controller/command", 1);
            drive_pub_[i] = nh_.advertise<std_msgs::Float64>(
                "/" + drive_names[i] + "_controller/command", 1);
        }

        double initial_dir[1] = {1.0};
        for (int i = 0; i <= params_.solver_cfg.N; i++) {
            solver_->set_online_parameter(i, initial_dir);
        }

        warm_start(*solver_, ref_gen_, state_, params_);
        printf("Ready! N=%d T=%.1f hz=%.0f ref=%s\n",
                 params_.solver_cfg.N, params_.solver_cfg.T, params_.solver_cfg.hz, params_.ref_type.c_str());

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / params_.solver_cfg.hz), &MpcNode::control_loop, this);
    }

private:
    ros::NodeHandle nh_;
    MpcParams params_;
    SolverConfig cfg_;
    std::unique_ptr<MPCSolver> solver_;
    ReferenceGenerator ref_gen_;
    SteeringIK ik_;
    std::unique_ptr<MpcVisualizer> viz_;
    std::unique_ptr<AcadosWrapper> wrapper_;

    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    ros::Publisher steer_pub_[NUM_WHEELS];
    ros::Publisher drive_pub_[NUM_WHEELS];
    
    std::vector<double> state_;
    int nx_, nu_;
    double sim_time_ = 0.0;
    double track_time_  = 0.0;

    bool odom_received_ = false;
    bool first_solve_ = true;
    int step_count_ = 0;

    tf2_ros::TransformBroadcaster tf_br_;
    WheelCmd last_good_wheels_[NUM_WHEELS] = {};
    bool has_good_solution_ = false;
    double last_shift_time_ = 0.0;

    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
        state_[0] = msg->pose.pose.position.x;
        state_[1] = msg->pose.pose.position.y;
        state_[2] = quat_to_yaw(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        // Gazebo p3d 插件的 twist 已经是全局坐标系速度
        state_[3] = msg->twist.twist.linear.x;
        state_[4] = msg->twist.twist.linear.y;
        state_[5] = msg->twist.twist.angular.z;

        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = msg->header.stamp;
        tfs.header.frame_id = "map";
        tfs.child_frame_id = "base_link";
        tfs.transform.translation.x = msg->pose.pose.position.x;
        tfs.transform.translation.y = msg->pose.pose.position.y;
        tfs.transform.translation.z = msg->pose.pose.position.z;
        tfs.transform.rotation = msg->pose.pose.orientation;
        tf_br_.sendTransform(tfs);
        odom_received_ = true;
    }

    void control_loop(const ros::TimerEvent &) {
        if (!odom_received_) {
            ROS_WARN_THROTTLE(2.0, "waiting for odom...");
            return;
        }

        double dt = 1.0 / params_.solver_cfg.hz;
        double dt_h = params_.solver_cfg.T / params_.solver_cfg.N;

        if (sim_time_ - last_shift_time_ >= dt_h) {
            shift_warm_start(*solver_, params_.solver_cfg.N);
            last_shift_time_ += dt_h;
        }

        solver_->set_x0(state_.data());

        for (int i = 0; i <= params_.solver_cfg.N; i++) {
            RefState ref = ref_gen_.at(sim_time_ + i * dt_h);
            double dyaw = normalize_angle(ref.val[2] - state_[2]);
            ref.val[2] = state_[2] + dyaw;
            if (i < params_.solver_cfg.N) {
                std::vector<double> yref(nx_ + nu_);
                std::copy(ref.val.begin(), ref.val.end(), yref.begin());
                std::copy(ref.u_ref.begin(), ref.u_ref.end(), yref.begin() + nx_);
                solver_->set_yref(i, yref.data());
            } else {
                solver_->set_yref(i, ref.val.data());
            }
        }

        double vx_global_ref = solver_->N() > 0 ? ref_gen_.at(sim_time_).val[3] : 0.0;
        double vy_global_ref = solver_->N() > 0 ? ref_gen_.at(sim_time_).val[4] : 0.0;
        double yaw_ref = state_[2]; 
        // 投影到体坐标系看 X 速度
        double v_body_x = vx_global_ref * std::cos(yaw_ref) + vy_global_ref * std::sin(yaw_ref);
        double current_dir = 1.0; // 默认前进
        if (v_body_x < -1e-2) {
            current_dir = -1.0;   // 后退
        }
        // 核心修改点：不再调用全局 C 函数，通过 Wrapper 统一注入符号参数
        double p_val[1] = { current_dir };
        for (int i = 0; i <= params_.solver_cfg.N; i++) {
            solver_->set_online_parameter(i, p_val);
        }

        auto start = std::chrono::high_resolution_clock::now();
        int status = solver_->solve();
        auto end = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // status=0 求解成功; status=4 检查解有效性; 其他=失败
        bool use_solution = false;
        if (status == 0) {
            use_solution = true;
        } else {
            use_solution = solution_valid(*solver_);
        }

        WheelCmd wheels[NUM_WHEELS];
        if (use_solution) {
            std::vector<double> x1(nx_);
            // 获取第 1 步的预测状态作为当前的执行目标
            // x1 = [x, y, yaw, vx_global, vy_global, dyaw]
            solver_->get_x(1, x1.data()); 
            double vx_global_target = x1[3];
            double vy_global_target = x1[4];
            double dyaw_target      = x1[5];

            // 转换到机器人体坐标系
            double vx_body, vy_body;
            global_to_body(state_[2], vx_global_target, vy_global_target, vx_body, vy_body);

            // 输入体坐标系速度
            ik_.compute(vx_body, vy_body, dyaw_target, wheels, dt);

            for (int i = 0; i < NUM_WHEELS; i++) {
                last_good_wheels_[i] = wheels[i];
            }
            has_good_solution_ = true;
        } else {
            // RAII 与安全性兜底：求解失败时，进行速度衰减而非保持全速
            printf("Solver failed (st=%d), using last good solution.", status);
            if (has_good_solution_) {
                for (int i = 0; i < NUM_WHEELS; i++) {
                    wheels[i] = last_good_wheels_[i];
                    wheels[i].drive_vel   *= 0.8;  // 衰减线速度
                    wheels[i].drive_omega *= 0.8;  // 衰减角速度
                    last_good_wheels_[i] = wheels[i];
                }
            } else {
                for (int i = 0; i < NUM_WHEELS; i++) {
                    wheels[i] = {0.0, 0.0, 0.0};
                }
            }
            ROS_WARN_THROTTLE(1.0, "[MPC] Solver failed (st=%d), applying deceleration.", status);
        }

        publish_joint_cmd(wheels);

        if (params_.sim_mode) {
            std::vector<double> u0(nu_, 0.0);
            if (use_solution) solver_->get_u(0, u0.data());
            state_[0] += state_[3] * dt + 0.5 * u0[0] * dt * dt;
            state_[1] += state_[4] * dt + 0.5 * u0[1] * dt * dt;
            state_[2] += state_[5] * dt + 0.5 * u0[2] * dt * dt;
            state_[3] += u0[0] * dt;
            state_[4] += u0[1] * dt;
            state_[5] += u0[2] * dt;
        }
        sim_time_ += dt;
        step_count_++;

        viz_->publish(state_, *solver_, ref_gen_, wheels);

        // 只保留 0.5 秒一次的状态日志
        ROS_INFO_THROTTLE(0.5,
            "t=%.1f sol_t=%.3fms st=%d step=%d | pos=[%.2f,%.2f] yaw=%.1fdeg vel=[%.2f,%.2f,%.2f]",
            sim_time_, duration.count() / 1e3, status, step_count_,
            state_[0], state_[1], state_[2]*180/M_PI,
            state_[3], state_[4], state_[5]);
    }

    void publish_joint_cmd(const WheelCmd wheels[NUM_WHEELS]) {
        for (int i = 0; i < NUM_WHEELS; i++) {
            std_msgs::Float64 steer_msg, drive_msg;
            // ("Current wheel %d: steer_angle = %f, drive_omega = %f", i, wheels[i].steer_angle, wheels[i].drive_omega);
            steer_msg.data = wheels[i].steer_angle;
            drive_msg.data = wheels[i].drive_omega;
            steer_pub_[i].publish(steer_msg);
            drive_pub_[i].publish(drive_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh("~");
    MpcNode node(nh);
    ros::spin();
    return 0;
}