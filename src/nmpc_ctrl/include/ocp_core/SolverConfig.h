#pragma once

#include <vector>
#include <string>

/**
 * @brief 通用求解器配置
 *
 * 只包含 OCP 求解器本身需要的参数。
 * 机器人特有的参数由应用层自行管理。
 */
struct SolverConfig {
    int    nx = 6;           // 状态维度
    int    nu = 3;           // 输入维度
    int    np = 1;           // 在线参数维度（最少 1，传 0 用哑参数）
    int    N  = 20;          // 预测步数
    double T  = 1.0;         // 预测时域 [s]
    double hz = 50.0;        // 控制频率 [Hz]

    // 代价权重（阶段代价）
    // W: 对角权重向量, 长度 = nx + nu
    // W_e: 终端代价对角权重向量, 长度 = nx
    std::vector<double> W;     // size = nx + nu
    std::vector<double> W_e;   // size = nx

    // 求解器选项
    double levenberg_marquardt = 1e-2;
    int    print_level = 0;

    std::string algorithm = "SQP_RTI";
    int sqp_max_iter = 50;

    /// 检查配置有效性
    bool isValid() const {
        if (nx <= 0 || nu <= 0 || N <= 0 || T <= 0) return false;
        if ((int)W.size() != nx + nu) return false;
        if ((int)W_e.size() != nx) return false;
        return true;
    }
};