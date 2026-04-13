#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "ocp_core/OcpProblem.h"
#include "robot_models/ArmDynamics.h"

std::vector<double> current_q(7, 0.0);
bool state_received = false;

// 全局存储整个机器人的当前关节状态，用于虚影匹配
std::vector<std::string> current_joint_names;
std::vector<double> current_joint_positions;

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

    // 声明左臂和右臂的 Publisher
    ros::Publisher left_pubs[7];
    ros::Publisher right_pubs[7];
    for(int i=0; i<7; ++i) {
        left_pubs[i] = nh.advertise<std_msgs::Float64>("/left_arm_joint" + std::to_string(i+1) + "_position_controller/command", 1);
        right_pubs[i] = nh.advertise<std_msgs::Float64>("/right_arm_joint" + std::to_string(i+1) + "_position_controller/command", 1);
    }

    // 用于发布预测虚影状态的 Publisher
    ros::Publisher pred_js_pub = nh.advertise<sensor_msgs::JointState>("/mpc_arm_predict/joint_states", 1);

    // 4. 设定目标动作 (让手臂抬起来，例如 joint2 和 joint4 弯曲)
    std::vector<double> target_q = {0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0};
    std::vector<double> target_dq(7, 0.0); // 目标速度为 0
    std::vector<double> yref(14);
    // std::copy(target_q.begin(), target_q.end(), yref.begin());
    // std::copy(target_dq.begin(), target_dq.end(), yref.begin() + 7);

    ros::Rate rate(cfg.hz);

    // 记录初始时间
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (state_received) {

            double t = (ros::Time::now() - start_time).toSec();
            
            // 让左臂做平滑的正弦波摆动 (基础值 + 偏置)
            target_q[0] = 0.5 * sin(1.0 * t);           // Joint 1: 左右摆动
            target_q[1] = -1.0 - 0.5 * cos(1.0 * t);    // Joint 2: 上下抬举 (基础角度-1.0)
            target_q[3] = -1.57 + 0.8 * sin(1.0 * t);   // Joint 4: 肘部弯曲 (基础角度-1.57)

            // 如果配置了速度惩罚，建议同时给出参考速度 (位置的导数)
            target_dq[0] = 0.5 * 1.0 * cos(1.0 * t);
            target_dq[1] = 0.5 * 1.0 * sin(1.0 * t);
            target_dq[3] = 0.8 * 1.0 * cos(1.0 * t);

            // 更新到 yref 数组中 (前7个是位置，后7个是速度)
            std::copy(target_q.begin(), target_q.end(), yref.begin());
            std::copy(target_dq.begin(), target_dq.end(), yref.begin() + 7);

            // 设置当前初始状态
            solver->set_x0(current_q.data());
            
            // 设置未来预测域的参考目标
            for (int i = 0; i < cfg.N; i++) solver->set_yref(i, yref.data());
            solver->set_yref(cfg.N, target_q.data()); // Terminal cost reference

            // Acados 求解
            if (solver->solve() == 0) {
                std::vector<double> solver_result_q(7);
                // 获取求解出来的下一步状态
                solver->get_x(1, solver_result_q.data()); 
                
                for (int i = 0; i < 7; ++i) {
                    // 下发左臂指令
                    std_msgs::Float64 cmd_left;
                    cmd_left.data = solver_result_q[i];
                    left_pubs[i].publish(cmd_left);

                    // ============ 右臂镜像动作 ============
                    std_msgs::Float64 cmd_right;
                    // 关节1(肩部左右旋转) 和 关节2(肩部前后俯仰) 取负号，实现完美的对称镜像运动
                    if (i == 0 || i == 1 || i == 3) {
                        cmd_right.data = -cmd_left.data;
                    } else {
                        cmd_right.data = cmd_left.data;
                    }
                    // 下发右臂指令
                    right_pubs[i].publish(cmd_right);
                }

                // 发布预测虚影
                sensor_msgs::JointState pred_msg;
                pred_msg.header.stamp = ros::Time::now();
                // 拷贝真实的机器人整体状态 (让虚影躯干对齐实车)
                pred_msg.name = current_joint_names;
                pred_msg.position = current_joint_positions;

                // 提取未来第 N 步（时域最末端）的状态作为虚影
                std::vector<double> pred_q_ghost(7);
                solver->get_x(cfg.N, pred_q_ghost.data()); 

                // 在全状态列表中，把双臂替换成预测的姿态
                for (size_t i = 0; i < pred_msg.name.size(); ++i) {
                    std::string name = pred_msg.name[i];
                    if (name.find("left_arm_joint") != std::string::npos) {
                        int idx = name.back() - '1';
                        if (idx >= 0 && idx < 7) pred_msg.position[i] = pred_q_ghost[idx];
                    } else if (name.find("right_arm_joint") != std::string::npos) {
                        int idx = name.back() - '1';
                        if (idx >= 0 && idx < 7) {
                            if (idx == 0 || idx == 1) pred_msg.position[i] = -pred_q_ghost[idx];
                            else pred_msg.position[i] = pred_q_ghost[idx];
                        }
                    }
                }
                pred_js_pub.publish(pred_msg);

            }
        }
        rate.sleep();
    }
    return 0;
}