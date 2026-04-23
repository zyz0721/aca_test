#pragma once
#include <memory>
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <thread>
#include <utility>
#include <nlohmann/json.hpp>
#include <condition_variable>

// 引入原有依赖
#include "galbot/singorix_common/utils/filter.hpp"
#include "galbot/singorix_wbc/interface.hpp"
#include "galbot/singorix_wbcs/controller/wbcs_controller.hpp"
#include "galbot/singorix_wbcs/interface/galbot_s1/galbot_s1_chassis_interface.hpp"

// 引入 NMPC 头文件
#include "galbot/singorix_wbcs/controller/galbot_s1/steer_nmpc.hpp"

using json = nlohmann::json;
namespace galbot {
namespace singorix {
namespace wbcs {


// 带时间戳的控制指令
struct StampedControl {    
    int idx;               // 对应索引
    double timestamp;      // 对应初始轨迹点时间 (秒)
    double vx, vy, omega;  // 控制量
};

enum MpcThreadState{
    IDLE,  // 空闲
    RUNNING, // 正在处理任务
    FINISHED // 任务处理完成
};

enum class ControlMode {
    IDLE,
    IN_PLACE_ROTATION,  // 包含：自动对齐舵角 -> 执行旋转 -> 旋转到位自动切 MPC
    MPC_TRACKING,       // MPC 轨迹跟踪
    DECELERATION,       // 减速
    ALIGN,              // 对准: 蟹行跟踪
    FINISHED
};

class GalbotS1ChassisPoseNMPCController : public WBCSController {
public:
    GalbotS1ChassisPoseNMPCController(std::string ctrl_name, std::string group_name, int ctrl_id, 
                               wbc::WBCInterfacePtr wbc_interface_ptr, RobotInterfacePtr robot_interface_ptr);
    ~GalbotS1ChassisPoseNMPCController();

    const std::string& className() const override { return class_name_; }
    std::any getAsDerived() const override { return getAsDerivedImpl(); }
    mutable std::function<std::any()> getAsDerivedImpl;

    bool init() override;
    bool start(bool blocking = true) override;
    bool update(Time t, Duration dt) override;
    bool stop(bool blocking = true) override;

private:
    // --- 核心逻辑函数 ---
    ControlMode control_mode_ = ControlMode::IDLE;
    ControlMode next_control_mode_ = ControlMode::IDLE;

    // 原地旋转相关参数
    int rotation_end_idx_ = 0;         // 跳点索引 (旋转结束点/MPC开始点)
    double rotation_target_yaw_ = 0.0; // 旋转的目标角度

    // 阈值参数
    double static_pos_threshold_ = 0.05;   // 位置静止阈值 (m)
    double align_angle_threshold_ = 0.03;  // 舵轮对齐允许误差 (rad, 约1.7度)
    double rotate_yaw_threshold_ = 0.03;   // 旋转完成允许误差 (rad, 约1.7度)
    double fast_vel_switch_threshold_;     // 快速切换速度阈值
    scalar_t mpc_stop_vel_threshold_;      // MPC停止阈值
    scalar_t mpc_stop_wait_timeout_;       // 末端索引到达后等待降速的最大时间
    scalar_t mpc_stop_wait_elapsed_ = 0.0; // 末端索引到达后已等待的时间
    scalar_t min_mpc_distance_threshold_;  // 使用mpc的最小距离阈值
    bool is_pose_initialized_ = false;
    bool align_finished_;                  // 检测到对齐完成

    scalar_t rotate_kp_;                                // 旋转控制参数kp
    scalar_t rotate_ki_;                                // 旋转控制参数ki
    scalar_t rotate_kd_;                                // 旋转控制参数kd
    scalar_t align_speed_p_gain_;                       // 对齐速度比例增益

    double last_yaw_err_ = 0.0;                        // 上一次的yaw误差
    double yaw_err_integral_ = 0.0;                    // yaw误差积分
    double yaw_err_threshold_ = 0.35;                  // 防止大角度旋转积分过大
    stdvec_scalar_t align_and_rotate_integral_limit_;  // 积分限幅
    stdvec_scalar_t min_align_and_rotate_vel_;         // 最小速度防止死区

    // 对齐阶段控制参数

    matrix_3t align_and_rotate_kp_;                     // 对齐控制参数kp
    matrix_3t align_and_rotate_ki_;                     // 对齐控制参数ki   
    matrix_3t align_and_rotate_kd_;                     // 对齐控制参数kd


    vector_3t last_align_err_ = {0.0, 0.0, 0.0};        // 上一次的对齐误差
    vector_3t align_err_integral_ = {0.0, 0.0, 0.0};    // 对齐误差积分
    vector_3t align_err_threshold_ = {0.35, 0.35, 0.35};// 防止大角度旋转积分过大
    vector_3t last_mpc_vel_cmd_;                        // 记录上一次mpc的速度指令
    scalar_t mpc_stop_dist_margin_;

    // (xy基本不变，yaw变化的最大索引)
    int findRotationCutoffIndex();

    // 检测舵轮对齐
    bool checkWheelsAligned();

    // 旋转模式逆运动学
    void inverseKinematicsForSpinMode(vector_3t &target_vel, std::vector<double> &output_steer_angles, std::vector<double> &output_wheel_vels);

    // 运行蟹行模式
    void inverseKinematicsForCrabMode(vector_3t &target_vel, std::vector<double> &output_steer_angles, std::vector<double> &output_wheel_vels);

    // 运行原地旋转模式 (包含对齐和旋转)
    void runInPlaceRotationMode();

    // 封装后的 MPC 跟踪函数
    void runMpcTrackingMode();

    // 运行减速模式
    void runDecelerationMode();

    // 最后的对准函数
    void runAlignMode();
    
    // NMPC 线程主循环
    void nmpcTaskLoop();
    
    // 生成参考轨迹 (内部填充 nmpc_data_)
    void generateReferenceHorizon(std::vector<Frame> &target_chassis_traj, size_t start_idx, size_t N, std::vector<double>& cur_pose, 
                                 std::vector<std::vector<double>>& out_horizon, std::vector<int>& out_horizon_idx, std::vector<double>& out_horizon_time, size_t virtual_end_idx);
    
    // 计算下次 MPC 起始索引 (考虑延迟)
    size_t getNextMpcStartIndex(size_t target_traj_size, size_t cur_start_idx, int mpc_delay_cycle_num) const;
    
    // 根据当前位姿投影查找最近索引
    int selectIdxByProjRatio(size_t &last_start_idx, const vector_3t& cur_pos, size_t min_idx);
    
    // 逆运动学解算
    void computeIK(const vector_3t& body_vel, std::vector<double>& angles, std::vector<double>& speeds);
    
    // 从指令队列获取控制量 (插值)
    bool getCmdFromControlDeque(size_t &cur_start_idx, vector_3t& cmd_out);

    // 限制加速度
    void limitAcceleration(vector_3t& velocity_command, const vector_3t& last_velocity_command,
                           stdvec_scalar_t max_acc, double dt);

    // 解析任务信息
    void parseTaskInfo(const std::string& task_name);

    // 新轨迹处理
    void newTrajProcess();

    // 辅助计算函数
    inline double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    double projectionRatio(const vector_3t& p, const vector_3t& left, const vector_3t& right);
    inline double lerp(double v0, double v1, double t) const { return v0 + t * (v1 - v0); }

    // 安全停止逻辑
    bool slowStop();
    bool slowDown(scalar_t& vel, scalar_t acc, double dt);

    // 对齐阶段逻辑
    void updateErrors(vector_3t& err);
    bool isWindowValid();
    double findNearestAngle(double angle, double ref);

    static inline std::string class_name_ = "GalbotS1ChassisPoseNMPCController";
    GalbotS1ChassisInterfacePtr galbot_s1_chassis_interface_ptr_;
    
    int wheel_num_ = 0;
    std::vector<vector_2t> wheel_pos_;
    double wheel_radius_ = 1.0; 
    scalar_t max_steer_angle_ = 1.57;
    stdvec_scalar_t max_vel_;
    stdvec_scalar_t max_acc_;

    int command_publish_cnt_;
    int log_skip_;
    std::vector<Frame> target_chassis_traj_, chassis_traj_interface_;
    std::string sub_task_name_;
    std::string body_frame_id_;
    std::string reference_frame_id_;
    int target_traj_size_ = 0;
    
    vector_3t current_pose_;              
    vector_3t current_vel_;                 // 当前body速度       
    std::vector<double> current_state_vec_; 

    size_t window_size_;                    // 窗口大小
    stdvec_scalar_t pos_threshold_;          // 到点阈值
    stdvec_scalar_t align_acc_;              // 对齐加速度
    std::deque<vector_3t> sliding_window_;  // 使用队列维护窗口数据

    size_t first_rotation_end_idx_ = 0;     // 第一段旋转结束索引
    size_t mid_mpc_end_idx_ = 0;            // 第二段MPC结束索引
    size_t last_rotation_end_idx_ = 0;      // 最后一段旋转结束索引
    std::string task_id_;                   // 任务ID    


    vector_3t cmd_vel_body_;
    vector_3t last_align_vel_cmd_;          // 最后对齐阶段速度，用于加速度限制
    vector_3t last_rot_vel_cmd_;
    scalar_t delta_time_;
    size_t cur_start_idx_ = 0; 
    size_t last_start_idx_ = 0; 
    size_t cur_excute_idx_ = 0;
    
    bool ctrl_sampling_finished_ = false;

    FilterPtr lpf_cur_x_;
    FilterPtr lpf_cur_y_;
    FilterPtr lpf_cur_yaw_;

    std::vector<double> current_steering_angles_; 
    std::vector<double> steer_angle_cmd_;
    std::vector<double> wheel_speed_cmd_;
    std::vector<double> stop_acc_;

    scalar_t localization_score_threshold_;
    scalar_t localization_delay_threshold_;
    bool slow_stop_when_shutdown_ = true;
    scalar_t traj_step_dt_ = 0.02;   
    scalar_t mpc_cycle_time_ = 0.02; 
    scalar_t control_step_dt_ = 0.008; 
    size_t mpc_delay_cycle_num_ = 0; 

    std::shared_ptr<SteeringNMPC> nmpc_core_;
    VehicleConfig nmpc_cfg_;
    int nmpc_horizon_ = 20;

    std::thread nmpc_thread_;
    std::atomic<bool> nmpc_thread_running_{false};
    
    std::mutex state_mutex_;
    std::mutex nmpc_data_mutex_;
    std::mutex traj_buffer_mutex_;

    struct NMPCData {
        std::vector<std::vector<double>> ref_horizon; 
        std::vector<int> ref_horizon_idx; 
        std::vector<double> ref_horizon_time;
        std::vector<double> init_state_vec; 
        MpcResult result;
        bool has_new_task = false; 
        int mpc_state;
    } nmpc_data_;

    std::deque<StampedControl> control_traj_buffer_;
    
    // int excuted_max_idx_ = 0; // 当前执行过的最大索引【一定是最近的，如果由于定位调到之前的索引去了，保持现有索引不动】
    // size_t last_excute_idx_ = 0; // 上一次执行的索引 
    // int keep_excute_idx_cycles_ = 0;// 改变索引后记录的循环周期数
    // int MAX_KEEP_CYCLES = 5; // 最大保持周期数量

    int look_ahead_cycles_ = 1; // 前瞻周期数
    vector_3t last_odom_pose_;  // 上一次的里程计位姿
    vector_3t last_map_pose_;   // 上一次的地图位姿
    bool has_last_odom_pose_ = false;
    bool has_last_map_pose_ = false;

};

} // namespace wbcs
} // namespace singorix
} // namespace galbot