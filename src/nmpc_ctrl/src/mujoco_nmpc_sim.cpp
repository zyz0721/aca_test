#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

// 包含你的 NMPC Solver 头文件 (根据实际路径调整)
#include "mpc_solver.h" 

// ==========================================
// 全局变量设置
// ==========================================
mjModel* m = NULL;
mjData* d = NULL;

// 控制频率设置
const double MPC_FREQ = 50.0;           // NMPC 运行频率 (Hz)
const double MPC_DT = 1.0 / MPC_FREQ;
double last_mpc_time = 0.0;

// 执行器(Actuator) ID 保存
int drive_act_ids[4];
int steer_act_ids[4];

// 关节(Joint) ID 保存，用于读取状态
int steer_joint_ids[4];
int drive_joint_ids[4];

// TODO: 定义你的全局 MPC 求解器指针
// MpcSolver* nmpc_solver = nullptr;
// std::vector<double> current_control(8, 0.0); // 缓存 NMPC 的输出 (4个速度，4个角度)

// ==========================================
// 辅助函数：四元数转 Yaw (偏航角)
// ==========================================
double get_yaw_from_quaternion(double qw, double qx, double qy, double qz) {
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

// ==========================================
// 初始化执行器和关节的 ID
// ==========================================
void init_mujoco_ids() {
    // 根据 galbot_S1_v1.0.xml 中的名称获取 ID
    for (int i = 0; i < 4; ++i) {
        std::string drive_act_name = "Wheel_" + std::to_string(i+1) + "_drive_joint";
        std::string steer_act_name = "Wheel_" + std::to_string(i+1) + "_direction_joint";
        std::string steer_jnt_name = "Wheel_" + std::to_string(i+1) + "_direction_joint";
        std::string drive_jnt_name = "Wheel_" + std::to_string(i+1) + "_drive_joint";

        drive_act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, drive_act_name.c_str());
        steer_act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, steer_act_name.c_str());
        
        steer_joint_ids[i] = mj_name2id(m, mjOBJ_JOINT, steer_jnt_name.c_str());
        drive_joint_ids[i] = mj_name2id(m, mjOBJ_JOINT, drive_jnt_name.c_str());

        if (drive_act_ids[i] == -1 || steer_act_ids[i] == -1) {
            std::cerr << "Warning: Could not find actuator for Wheel " << i+1 << std::endl;
        }
    }
}

// ==========================================
// 保持上身关节不动
// ==========================================
void hold_upper_body() {
    // 遍历所有执行器，如果不是轮子的执行器，则将其控制量设为初始状态 (qpos[0])
    for (int i = 0; i < m->nu; ++i) {
        bool is_wheel_actuator = false;
        for (int w = 0; w < 4; ++w) {
            if (i == drive_act_ids[w] || i == steer_act_ids[w]) {
                is_wheel_actuator = true;
                break;
            }
        }
        
        if (!is_wheel_actuator) {
            // 获取该执行器控制的关节 ID
            int trnid = m->actuator_trnid[i * 2]; // 假设 transmission 是 joint
            if (trnid >= 0 && m->actuator_trntype[i] == mjTRN_JOINT) {
                int qpos_adr = m->jnt_qposadr[trnid];
                // 对于位置控制器，将其指令设为当前/初始角度
                d->ctrl[i] = d->qpos[qpos_adr]; 
            }
        }
    }
}

// ==========================================
// 控制回调函数：每次物理引擎 step 前执行
// ==========================================
void control_callback(const mjModel* m, mjData* d) {
    // 1. 频率控制：只在达到 MPC_DT 的时间间隔时才运行 NMPC
    if (d->time - last_mpc_time >= MPC_DT) {
        last_mpc_time = d->time;

        // --- A. 状态读取 (State Feedback) ---
        // 假设 base 是 root (通常有 freejoint，占据 qpos 的前 7 个元素: x,y,z, qw,qx,qy,qz)
        // 占据 qvel 的前 6 个元素: vx,vy,vz, wx,wy,wz
        double base_x = d->qpos[0];
        double base_y = d->qpos[1];
        double base_yaw = get_yaw_from_quaternion(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);

        double base_vx = d->qvel[0];
        double base_vy = d->qvel[1];
        double base_omega = d->qvel[5];

        // 读取 4 个舵轮的当前偏航角
        double steer_angles[4];
        for(int i=0; i<4; ++i) {
            int qpos_adr = m->jnt_qposadr[steer_joint_ids[i]];
            steer_angles[i] = d->qpos[qpos_adr];
        }

        // --- B. 调用 NMPC 求解 ---
        // TODO: 将上方读取的 base_x, base_y, base_yaw, base_vx, ... steer_angles 打包进你的 state 向量
        // std::vector<double> current_state = {base_x, base_y, base_yaw, base_vx, base_vy, base_omega, steer_angles[0], steer_angles[1], steer_angles[2], steer_angles[3]};
        
        // TODO: 生成期望轨迹 (Reference)
        // std::vector<double> reference = {...}; 

        // TODO: NMPC 求解
        // nmpc_solver->solve(current_state, reference, current_control);
        
        /* * 假设 current_control 的顺序是:
         * [drive_v1, steer_p1, drive_v2, steer_p2, drive_v3, steer_p3, drive_v4, steer_p4]
         */
    }

    // --- C. 将控制指令写入 MuJoCo ---
    // 即使在没有计算 NMPC 的步长里，也要持续发送当前的控制指令(零阶保持)
    /*
    if (nmpc_solver != nullptr) {
        for (int i = 0; i < 4; ++i) {
            d->ctrl[drive_act_ids[i]] = current_control[i * 2];       // 车轮线速度
            d->ctrl[steer_act_ids[i]] = current_control[i * 2 + 1];   // 舵轮偏航角位置
        }
    }
    */
   
    // (调试用) 让机器人原地转圈测试: 轮子全打 0.78 rad (45度)，给一定线速度
    for (int i = 0; i < 4; ++i) {
        d->ctrl[steer_act_ids[i]] = 0.785; 
        d->ctrl[drive_act_ids[i]] = 2.0;   
    }
}

// ==========================================
// 主函数
// ==========================================
int main(int argc, char** argv) {
    std::cout << "Starting Galbot S1 MuJoCo Simulation..." << std::endl;

    // 1. 加载模型 (指向你的 XML 文件)
    const char* filename = "/home/galbot/galbot_ws/aca_test/galbot_S1_v1.0/galbot_S1_v1.0.xml";
    char error[1000] = "Could not load model";
    m = mj_loadXML(filename, 0, error, 1000);
    if (!m) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 2. 初始化执行器映射和上身姿态
    init_mujoco_ids();
    
    // 如果你希望初始状态机器人在半空中掉下来，可以修改 d->qpos[2]
    // d->qpos[2] = 0.5;

    // TODO: 3. 初始化你的 NMPC Solver
    // nmpc_solver = new MpcSolver(...);

    // 4. 注册控制回调函数
    mjcb_control = control_callback;

    // 5. 初始化 GLFW 窗口 (用于可视化)
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Galbot 4-Steer NMPC Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 6. 将上身各关节设定为当前状态 (防止仿真刚开始时散架)
    hold_upper_body();

    // 7. 主仿真和渲染循环
    while (!glfwWindowShouldClose(window)) {
        // 让 MuJoCo 步进直到跟上真实世界的时间
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) { // 以 60FPS 渲染
            mj_step(m, d);
        }

        // 渲染更新
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        // 允许通过鼠标控制摄像机视角
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 8. 释放资源
    mjcb_control = NULL;
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();

    // delete nmpc_solver;
    
    return 0;
}