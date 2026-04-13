#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "ocp_core/OcpProblem.h"
#include "robot_models/ArmDynamics.h"

std::vector<double> current_q(7, 0.0);
bool state_received = false;

// 订阅关节真实状态
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string name = msg->name[i];
        if (name.find("left_arm_joint") != std::string::npos) {
            int idx = name.back() - '1'; // 将 "left_arm_joint1" 映射为索引 0
            if(idx >= 0 && idx < 7) {
                current_q[idx] = msg->position[i];
            }
        }
    }
    state_received = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_mpc_node");
    ros::NodeHandle nh("~");

    // 1. 配置求解器
    SolverConfig cfg;
    cfg.nx = 7; cfg.nu = 7; cfg.N = 20; cfg.T = 1.0; cfg.hz = 50.0;
    // 权重：优先逼近目标角度 (Q)，同时限制速度不要太大 (R)
    cfg.W = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,  // Q: 角度误差惩罚
              0.1,  0.1,  0.1,  0.1,  0.1,  0.1,  0.1}; // R: 速度大小惩罚

    // 2. 构建 OCP 问题
    using S = StageSelector;
    using B = BoundConstraintData;
    OcpProblem ocp(cfg);
    ocp.setDynamics<ArmDynamics>();
    
    // 关节角度限制 (-3.14 to 3.14) 和 关节速度限制 (-1.0 to 1.0)
    std::vector<int> all_idx = {0,1,2,3,4,5,6};
    std::vector<double> min_q(7, -3.14), max_q(7, 3.14);
    std::vector<double> min_dq(7, -1.0), max_dq(7, 1.0);
    
    ocp.addBound(S::pathAndTerminal(), B{B::STATE, all_idx, min_q, max_q});
    ocp.addBound(S::path(), B{B::INPUT, all_idx, min_dq, max_dq});
    
    auto solver = ocp.build();
    if (!solver) return -1;

    // 3. ROS 通信设置
    ros::Subscriber sub = nh.subscribe("/joint_states_merged", 1, jointStateCallback);
    ros::Publisher pubs[7];
    for(int i=0; i<7; ++i) {
        pubs[i] = nh.advertise<std_msgs::Float64>("/left_arm_joint" + std::to_string(i+1) + "_position_controller/command", 1);
    }

    // 4. 设定目标动作 (让手臂抬起来，例如 joint2 和 joint4 弯曲)
    std::vector<double> target_q = {0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0};
    std::vector<double> target_dq(7, 0.0); // 目标速度为 0
    std::vector<double> yref(14);
    std::copy(target_q.begin(), target_q.end(), yref.begin());
    std::copy(target_dq.begin(), target_dq.end(), yref.begin() + 7);

    ros::Rate rate(cfg.hz);
    while (ros::ok()) {
        ros::spinOnce();
        if (state_received) {
            // 设置当前初始状态
            solver->set_x0(current_q.data());
            
            // 设置未来预测域的参考目标
            for (int i = 0; i < cfg.N; i++) solver->set_yref(i, yref.data());
            solver->set_yref(cfg.N, target_q.data()); // Terminal cost reference

            // Acados 求解！
            if (solver->solve() == 0) {
                std::vector<double> next_q(7);
                // 获取第一步预测的角度，下发给底层 PID 控制器
                solver->get_x(1, next_q.data()); 
                
                for(int i=0; i<7; ++i) {
                    std_msgs::Float64 cmd;
                    cmd.data = next_q[i];
                    pubs[i].publish(cmd);
                }
                ROS_INFO_THROTTLE(1.0, "Arm MPC running! Joint2 target: %.2f", next_q[1]);
            }
        }
        rate.sleep();
    }
    return 0;
}