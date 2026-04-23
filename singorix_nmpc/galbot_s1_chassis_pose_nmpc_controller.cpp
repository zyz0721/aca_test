#include "galbot/singorix_wbcs/controller/galbot_s1/galbot_s1_chassis_pose_nmpc_controller.hpp"

using namespace galbot::singorix;
using namespace galbot::singorix::wbcs;

// [注册] 注册新的 Controller 名称
static WBCSControllerAutoRegister<GalbotS1ChassisPoseNMPCController, std::string, int, wbc::WBCInterfacePtr, RobotInterfacePtr> 
    regGalbotS1ChassisPoseNMPCController("GalbotS1ChassisPoseNMPCController");

GalbotS1ChassisPoseNMPCController::GalbotS1ChassisPoseNMPCController(std::string ctrl_name, std::string group_name, int ctrl_id, 
                                                       wbc::WBCInterfacePtr wbc_interface_ptr, RobotInterfacePtr robot_interface_ptr)
    : WBCSController(ctrl_name, group_name, ctrl_id, wbc_interface_ptr)
{
    galbot_s1_chassis_interface_ptr_ = RobotInterfaceFactory::castToDerived<GalbotS1ChassisInterface>(robot_interface_ptr);
    init();
}

GalbotS1ChassisPoseNMPCController::~GalbotS1ChassisPoseNMPCController() {
    stop();
}

bool GalbotS1ChassisPoseNMPCController::init() {
    reload();
    controller_state_ = wbc::ControllerState::ControllerStateInitialized;

    return true;
}

bool GalbotS1ChassisPoseNMPCController::start(bool blocking) {

    // 获取机器人几何信息
    wheel_num_ = galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->wheel_num_;
    wheel_pos_ = galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->wheel_pos_;
    max_steer_angle_ = galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->max_steer_angle_;
    
    steer_angle_cmd_.assign(wheel_num_, 0.0);
    wheel_speed_cmd_.assign(wheel_num_, 0.0);
    current_steering_angles_.assign(wheel_num_, 0.0);

    // 读取 YAML 配置参数 (建议添加到配置文件中)
    auto& params_reader = wbc_interface_ptr_->wbc_model_ptr_;
    window_size_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "window_size", 10);
    pos_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "pos_threshold", stdvec_scalar_t{0.03, 0.03, 0.03});
    auto filter_cur = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "filter_cur", stdvec_scalar_t{1.0, 1.0, 1.0});
    stop_acc_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "stop_acc", stdvec_scalar_t{1.5, 1.5, 1.5});
    align_acc_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_acc", stdvec_scalar_t{0.4, 0.4, 0.4});
    mpc_stop_vel_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "mpc_stop_vel_threshold", 0.1);
    mpc_stop_wait_timeout_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "mpc_stop_wait_timeout", 0.3);
    min_mpc_distance_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "min_mpc_distance_threshold", 0.3);
    mpc_stop_dist_margin_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "mpc_stop_dist_margin", 0.15);
    fast_vel_switch_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "fast_vel_switch_threshold", 0.05);
    log_skip_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "log_skip", int(wbc_interface_ptr_->wbc_model_ptr_->robot_config_.wbc_rate));
    localization_score_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "localization_score_threshold", 0.75);
    localization_delay_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "localization_delay_threshold", 0.5);
    slow_stop_when_shutdown_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "slow_stop_when_shutdown", true);
    
    auto align_and_rotate_kp = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_and_rotate_kp", stdvec_scalar_t{1.0, 1.0, 1.5});
    auto align_and_rotate_ki = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_and_rotate_ki", stdvec_scalar_t{0.0, 0.0, 0.0});
    auto align_and_rotate_kd = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_and_rotate_kd", stdvec_scalar_t{0.03, 0.03, 0.03});
    align_speed_p_gain_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_speed_p_gain", 2.0);
    align_and_rotate_integral_limit_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_and_rotate_integral_limit", stdvec_scalar_t{0.3, 0.3, 0.3});
    min_align_and_rotate_vel_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "min_align_and_rotate_vel", stdvec_scalar_t{0.03, 0.03, 0.03});
    
    align_and_rotate_kp_= matrix_3t::Zero(3, 3);
    align_and_rotate_ki_= matrix_3t::Zero(3, 3);
    align_and_rotate_kd_= matrix_3t::Zero(3, 3);
    align_and_rotate_kp_.diagonal() << align_and_rotate_kp[0], align_and_rotate_kp[1], align_and_rotate_kp[2];
    align_and_rotate_ki_.diagonal() << align_and_rotate_ki[0], align_and_rotate_ki[1], align_and_rotate_ki[2];
    align_and_rotate_kd_.diagonal() << align_and_rotate_kd[0], align_and_rotate_kd[1], align_and_rotate_kd[2];

    
    rotate_kp_ = align_and_rotate_kp[2];
    rotate_ki_ = align_and_rotate_ki[2];
    rotate_kd_ = align_and_rotate_kd[2];
    
    nmpc_horizon_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "nmpc_horizon", 20);
    traj_step_dt_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "traj_step_dt", 0.02);
    control_step_dt_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "control_step_dt", 0.008);
    
    // 配置 NMPC 参数
    max_vel_  = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "max_vel", stdvec_scalar_t{0.8, 0.8, 0.8});
    max_acc_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "max_acc", stdvec_scalar_t{0.6, 0.6, 0.6});
    // 权重参数
    auto Q_pos = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "Q_pos", stdvec_scalar_t{10.0, 5.0});      // 位置权重
    auto Q_vel = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "Q_vel", stdvec_scalar_t{0.1, 0.1});       // 速度权重
    auto R_u = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "R_u", stdvec_scalar_t{0.1, 0.1});
    auto Q_term_pos = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "Q_term_pos", stdvec_scalar_t{10.0, 10.0});
    auto Q_term_vel = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "Q_term_vel", stdvec_scalar_t{50.0, 50.0});

    static_pos_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "static_pos_threshold", 0.02);   //位置静止阈值[针对跳点] 
    align_angle_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "align_angle_threshold", 0.03); //舵轮对齐允许误差 (rad, 约4.5度)
    rotate_yaw_threshold_ = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "rotate_yaw_threshold", 0.03);   //旋转完成允许误差 (rad, 约1.7度)


    nmpc_cfg_.max_vel = max_vel_[0];        // 最大速度
    nmpc_cfg_.max_yaw_rate = max_vel_[2];   // 最大角速度
    nmpc_cfg_.max_acc = max_acc_[0];        // 最大加速度
    nmpc_cfg_.max_yaw_acc  = max_acc_[2];   // 最大角加速度
    
    nmpc_cfg_.Q_xy   = Q_pos[0];           // 位置权重
    nmpc_cfg_.Q_yaw  = Q_pos[1];           // 航向权重
    nmpc_cfg_.Q_vxy  = Q_vel[0];           // 横向速度权重
    nmpc_cfg_.Q_dyaw  = Q_vel[1];          // 航向角速度权重
    
    nmpc_cfg_.R_axy  = R_u[0];             // 控制量-加速度权重
    nmpc_cfg_.R_ddyaw= R_u[1];             // 控制量-角加速度权重
    
    nmpc_cfg_.Q_term_xy = Q_term_pos[0];   // 终点位置权重
    nmpc_cfg_.Q_term_yaw = Q_term_pos[1];  // 终点航向权重
    nmpc_cfg_.Q_term_vxy = Q_term_vel[0];  // 终点横向速度权重
    nmpc_cfg_.Q_term_ddyaw = Q_term_vel[1];// 终点航向角速度权重
    nmpc_cfg_.W_steer_rate = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "weight_steer_rate", 200); // 舵角转向速率软约束权重

    // 舵轮约束参数
    nmpc_cfg_.max_steer_rate = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_, "max_steer_rate", 2.0); // rad/s
    
    nmpc_cfg_.steer_lim_min = -max_steer_angle_; 
    nmpc_cfg_.steer_lim_max = max_steer_angle_;

    // 填入轮子位置 (用于 NMPC 内部约束计算)
    nmpc_cfg_.wheel_positions.clear();
    for(const auto& p : wheel_pos_) {
        nmpc_cfg_.wheel_positions.push_back({p.x(), p.y()});
    }
    
    double delay_time = params_reader->tryGetGroupControllerCustomParams(group_name_, ctrl_name_,"nmpc_delay", 0.05); // NMPC 延迟时间
    mpc_delay_cycle_num_ = std::ceil(delay_time / traj_step_dt_); // mpc_delay_cycle_num_：0.0
    delta_time_ = 1.0 / (wbc_interface_ptr_->wbc_model_ptr_->robot_config_.wbc_rate );
    last_start_idx_ = 0;
    cur_excute_idx_ = 0;
    // 清空误差
    last_yaw_err_ = 0.0;
    yaw_err_integral_ = 0.0;
    last_align_err_.setZero();
    align_err_integral_.setZero();

    align_finished_ = false;
    ctrl_sampling_finished_ = false;
    mpc_stop_wait_elapsed_ = 0.0;
    is_pose_initialized_ = false;
    cmd_vel_body_.setZero();
    last_mpc_vel_cmd_.setZero();
    current_vel_.setZero();
    last_align_vel_cmd_.setZero();
    last_rot_vel_cmd_.setZero();
    last_odom_pose_.setZero();
    last_map_pose_.setZero();

    target_chassis_traj_.clear();
    target_traj_size_ = 0;
    control_traj_buffer_.clear();
    sliding_window_.clear();
    command_publish_cnt_ = 0;

    // 初始化状态
    control_mode_ = ControlMode::IDLE;
    nmpc_data_.mpc_state = MpcThreadState::IDLE;

    // 防止重复启动NMPC线程
    if (!nmpc_thread_running_) {
        nmpc_thread_running_ = true;
        nmpc_thread_ = std::thread(&GalbotS1ChassisPoseNMPCController::nmpcTaskLoop, this);
    }

    // 初始化一些基础滤波器，用于平滑定位数据
    lpf_cur_x_ = std::make_shared<IIROneOrderLowPassFilter>(filter_cur[0]);
    lpf_cur_y_ = std::make_shared<IIROneOrderLowPassFilter>(filter_cur[1]);
    lpf_cur_yaw_ = std::make_shared<IIROneOrderLowPassFilter>(filter_cur[2]);

    // 实例化 NMPC Core
    LOG(INFO) << "Initializing Steering NMPC with N=" << nmpc_horizon_ << ", dt=" << traj_step_dt_;
    nmpc_core_ = std::make_shared<SteeringNMPC>(nmpc_horizon_, traj_step_dt_, nmpc_cfg_);

    controller_state_ = wbc::ControllerState::ControllerStateRunning;
    
    return true;
}
// 
bool GalbotS1ChassisPoseNMPCController::update(Time t, Duration dt) {
    if (controller_state_ != wbc::ControllerState::ControllerStateRunning) return false;
    // --- 1. 获取任务与轨迹 ---
    auto data_ptr = wbc_interface_ptr_->wbc_data_ptr_;
    auto target_task_traj_it = data_ptr->target_task_traj_map_.find("swerve_chassis");
    
    // 查询当前位置
    if (target_task_traj_it == data_ptr->target_task_traj_map_.end()) {
        return false;
    } else {
        if (target_task_traj_it->second.task_now.size() != 1) {
            LOG(ERROR) << "Task command size must be 1";
            return false;
        }
        if (!(target_task_traj_it->second.target_config.target_data & wbc::TargetDataFramePose)) {
            LOG(ERROR) << "target_data must contain TargetDataFramePose";
            return false;
        }
    } 
 
    auto& task_data = target_task_traj_it->second;
    bool is_new_task = (sub_task_name_ != task_data.subtask_names.front());
    // LOG(INFO)<< sub_task_name_<< " " << task_data.subtask_names.front();
    if (is_new_task) {
        sub_task_name_ = task_data.subtask_names.front();
        LOG(INFO) << "New task: " << sub_task_name_;
        parseTaskInfo(sub_task_name_);
        {
            std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
            // 清空轨迹
            chassis_traj_interface_.clear();
            for(const auto& task_pt : task_data.tasks) {
                chassis_traj_interface_.push_back(task_pt.front());
            }
            // 接收任务标志位
            // nmpc_data_.has_new_task = true;
        }

        if (!chassis_traj_interface_.empty()) {
            body_frame_id_ = chassis_traj_interface_.front().body_frame_id;
            reference_frame_id_ = chassis_traj_interface_.front().reference_frame_id;
            // 处理新轨迹
            newTrajProcess();
        }
        // 第一次打印目标轨迹
        for (auto i = 0; i < chassis_traj_interface_.size(); ++i) {
            double tmp_pos_x = chassis_traj_interface_[i].pose.position.x();
            double tmp_pos_y = chassis_traj_interface_[i].pose.position.y();
            double tmp_yaw = quaternionToEulerZYX(chassis_traj_interface_[i].pose.orientation)[0];
            LOG(INFO) << "Target trajectory index " << i << ": pos: " << tmp_pos_x << " " << tmp_pos_y << " " << tmp_yaw;
        }
    }
    // 2. 状态读取
    current_steering_angles_ = galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->getSteerWheelPositions();
    auto odom = galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->getOdomData();
    // 更新机器人速度
    current_vel_[0] = odom.linear_vx;
    current_vel_[1] = odom.linear_vy;
    current_vel_[2] = odom.angular_vz;
    scalar_t raw_odom_x = odom.position_x;
    scalar_t raw_odom_y = odom.position_y;
    scalar_t raw_odom_yaw = odom.orientation;

    // LOG(INFO) << "current_vel_: " << current_vel_[0] << ", " << current_vel_[1] << ", " << current_vel_[2];
    vector_3t vel_body(odom.linear_vx, odom.linear_vy, odom.angular_vz);

    // unreliable position
    double localization_score = data_ptr->tryGetOtherInfo("localization_score", 1.0);
    double localization_delay = data_ptr->tryGetOtherInfo("localization_delay", 100.0);
    if (reference_frame_id_ == "map") {
        if (localization_score < localization_score_threshold_) {
            std::stringstream error_description;
            error_description << "low localization score: " << localization_score;
            LOG(ERROR) << error_description.str();
            // TODO: using ctrl_name_
            data_ptr->updateControllerError(group_name_, "localization_score_in_ctrl", {wbc::ErrorLevel::ErrorSafetyMedium, 0x30011, error_description.str()});
            return stop(false);
        }
        if (localization_delay > localization_delay_threshold_) {
            std::stringstream error_description;
            error_description << "high localization delay: " << localization_delay;
            LOG(ERROR) << error_description.str();
            // TODO: using ctrl_name_
            data_ptr->updateControllerError(group_name_, "localization_delay_in_ctrl", {wbc::ErrorLevel::ErrorSafetyMedium, 0x30012, error_description.str()});
            return stop(false);
        }
    }

    auto ref_edges = data_ptr->fusion_graph_.queryEdge(body_frame_id_, reference_frame_id_);
    if(ref_edges.empty()) return false;

    double raw_x = ref_edges.front().pose.position.x();
    double raw_y = ref_edges.front().pose.position.y();
    double raw_yaw = normalizeAngle(quaternionToEulerZYX(ref_edges.front().pose.orientation)[0]);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_pose_[0] = lpf_cur_x_->process(raw_x);
        current_pose_[1] = lpf_cur_y_->process(raw_y);
        current_pose_[2] = lpf_cur_yaw_->process(raw_yaw);
        // 转到世界坐标系下
        double yaw = current_pose_[2];
        double vx_g = std::cos(yaw) * cmd_vel_body_[0] - std::sin(yaw) * cmd_vel_body_[1];
        double vy_g = std::sin(yaw) * cmd_vel_body_[0] + std::cos(yaw) * cmd_vel_body_[1];

        current_state_vec_ = {current_pose_[0], current_pose_[1], current_pose_[2], vx_g, vy_g, cmd_vel_body_[2]};
        is_pose_initialized_ = true;
    }

    // 定义用于周期性日志打印的变量，计算在机器人坐标系的位置跳变
    double jump_x = 0.0, jump_y = 0.0, jump_yaw = 0.0;
    double dx_body_map = 0.0, dy_body_map = 0.0;
    double dx_body_odom = 0.0, dy_body_odom = 0.0;

    if (has_last_map_pose_ && has_last_odom_pose_) {
        double dx_map = raw_x - last_map_pose_[0];
        double dy_map = raw_y - last_map_pose_[1];
        double dyaw_map = normalizeAngle(raw_yaw - last_map_pose_[2]);

        // Map 增量(Body Frame)
        dx_body_map =  std::cos(last_map_pose_[2]) * dx_map + std::sin(last_map_pose_[2]) * dy_map;
        dy_body_map = -std::sin(last_map_pose_[2]) * dx_map + std::cos(last_map_pose_[2]) * dy_map;

        double dx_odom = raw_odom_x - last_odom_pose_[0];
        double dy_odom = raw_odom_y - last_odom_pose_[1];
        double dyaw_odom = normalizeAngle(raw_odom_yaw - last_odom_pose_[2]);

        // Odom 增量(Body Frame)
        dx_body_odom =  std::cos(last_odom_pose_[2]) * dx_odom + std::sin(last_odom_pose_[2]) * dy_odom;
        dy_body_odom = -std::sin(last_odom_pose_[2]) * dx_odom + std::cos(last_odom_pose_[2]) * dy_odom;

        // 在 Body 坐标系下计算误差
        jump_x = std::abs(dx_body_map - dx_body_odom);
        jump_y = std::abs(dy_body_map - dy_body_odom);
        jump_yaw = std::abs(normalizeAngle(dyaw_map - dyaw_odom));
    }
    last_map_pose_ = vector_3t(raw_x, raw_y, raw_yaw);
    has_last_map_pose_ = true;

    last_odom_pose_ = vector_3t(raw_odom_x, raw_odom_y, raw_odom_yaw);
    has_last_odom_pose_ = true;

    switch (control_mode_) {
        case ControlMode::IN_PLACE_ROTATION:
            runInPlaceRotationMode();
            break;

        case ControlMode::MPC_TRACKING:
            runMpcTrackingMode();
            break;
        
        case ControlMode::DECELERATION:
            runDecelerationMode();
            break;

        case ControlMode::ALIGN:
            runAlignMode();
            break;
        
        case ControlMode::FINISHED:
            // task_data.target_finished = true;
            break;

        case ControlMode::IDLE:
        default:
            break;
    }
    if (++command_publish_cnt_ >= log_skip_) {
        command_publish_cnt_ = 0;
        // 获取当前控制状态的可读字符串
        std::string mode_str;
        switch (control_mode_) {
            case ControlMode::IDLE:              mode_str = "IDLE"; break;
            case ControlMode::IN_PLACE_ROTATION: mode_str = "IN_PLACE_ROTATION"; break;
            case ControlMode::MPC_TRACKING:      mode_str = "MPC_TRACKING"; break;
            case ControlMode::DECELERATION:      mode_str = "DECELERATION"; break;
            case ControlMode::ALIGN:             mode_str = "ALIGN"; break;
            case ControlMode::FINISHED:          mode_str = "FINISHED"; break;
            default:                             mode_str = "UNKNOWN"; break;
        }

        LOG(INFO) << group_name_ << " pub " << command_publish_cnt_ << " commands, time: " << getTimeSeconds() 
                  << "[State] Control Mode : " << mode_str;
        LOG(INFO) << "[Pos]: x=" << current_pose_[0] << ", y=" << current_pose_[1] << ", yaw=" << current_pose_[2];
        if (!chassis_traj_interface_.empty()) {
            size_t log_target_idx = cur_excute_idx_; 
            if (control_mode_ == ControlMode::ALIGN || control_mode_ == ControlMode::FINISHED) {
                log_target_idx = chassis_traj_interface_.size() - 1;
            } else if (control_mode_ == ControlMode::IN_PLACE_ROTATION) {
                log_target_idx = rotation_end_idx_;
            }
            if (log_target_idx >= chassis_traj_interface_.size()) {
                log_target_idx = chassis_traj_interface_.size() - 1;
            }

            LOG(INFO) << "[Pos] Desired (Ref)  : x=" << chassis_traj_interface_[log_target_idx].pose.position.x() 
                      << ", y=" << chassis_traj_interface_[log_target_idx].pose.position.y() 
                      << ", yaw=" << quaternionToEulerZYX(chassis_traj_interface_[log_target_idx].pose.orientation)[0];
        }
        LOG(INFO) << "[Odom] Raw Position  : x=" << raw_odom_x << ", y=" << raw_odom_y << ", yaw=" << raw_odom_yaw
                  << "[Odom] Raw Velocity: vx=" << odom.linear_vx << ", vy=" << odom.linear_vy << ", omega=" << odom.angular_vz;

        if (has_last_map_pose_ && has_last_odom_pose_) {
            LOG(INFO) << "[Delta] Body_Map     : dx=" << dx_body_map << ", dy=" << dy_body_map 
                      << "[Delta] Body_Odom    : dx=" << dx_body_odom << ", dy=" << dy_body_odom
                      << "[Delta] Jump Error   : x=" << jump_x << "m, y=" << jump_y << "m, yaw=" << jump_yaw << "rad";
        }   
        LOG(INFO) << "[Cmd] Body Velocity: vx=" << cmd_vel_body_[0] << ", vy=" << cmd_vel_body_[1] << ", omega=" << cmd_vel_body_[2];
        LOG(INFO) << "[Cmd] Steer Angles: " << steer_angle_cmd_ << ", Wheel Speeds: " << wheel_speed_cmd_;
        if (reference_frame_id_ == "map") {
            LOG(INFO) << "[Loc] Map Score: " << localization_score << ", Delay: " << localization_delay << "s";
        }
    }
    
    return true;
}

// MPC模式
void GalbotS1ChassisPoseNMPCController::runMpcTrackingMode() {

    // 根据当前位置选择索引
    vector_3t pose_snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pose_snapshot = current_pose_;
    }
    
    cur_start_idx_ = last_start_idx_;
    // 这里的cur_excute_idx_是当前段的下一段终点索引
    cur_excute_idx_ = selectIdxByProjRatio(cur_start_idx_, pose_snapshot, first_rotation_end_idx_);
    last_start_idx_ = cur_start_idx_;

    auto pt_mpc_end = target_chassis_traj_[mid_mpc_end_idx_].pose.position;
    scalar_t current_speed = std::hypot(cmd_vel_body_[0], cmd_vel_body_[1]);

    scalar_t handover_acc = std::min(stop_acc_[0], stop_acc_[1]);                         
    handover_acc = std::max(handover_acc, 0.05);

    scalar_t required_dist = current_speed * current_speed / (2 * handover_acc) +  mpc_stop_dist_margin_;    // 根据刹车距离提前停止mpc
    scalar_t dist_to_goal = std::hypot(pose_snapshot[0] - pt_mpc_end.x(), pose_snapshot[1] - pt_mpc_end.y());
    
    LOG(INFO)<< "cur start idx:  "<< cur_start_idx_ << "    current_pose_:   "<<current_pose_[0]<<" "<<current_pose_[1]<<"  "<<current_pose_[2];
    // 获取控制指令    
    bool valid_cmd = getCmdFromControlDeque(cur_start_idx_, cmd_vel_body_);

    // 判断mpc终止条件：必须沿路径索引到达 mid_mpc_end_idx_ 才结束，
    // 避免弯道处直线距离提前触发而让 ALIGN 切弯
    // 到达末端索引后，继续执行 MPC 末端衰减段，等待实际速度降下来或等待超时才切 ALIGN
    bool index_reached = (cur_excute_idx_ >= mid_mpc_end_idx_);
    scalar_t cur_vel_norm = std::hypot(cmd_vel_body_[0], cmd_vel_body_[1]);
    bool vel_low_enough = (cur_vel_norm < mpc_stop_vel_threshold_);

    if (index_reached) {
        mpc_stop_wait_elapsed_ += delta_time_;
    } else {
        mpc_stop_wait_elapsed_ = 0.0;
    }
    bool wait_timeout = (mpc_stop_wait_elapsed_ >= mpc_stop_wait_timeout_);

    bool mpc_segment_finished = index_reached && (vel_low_enough || wait_timeout);

    if (mpc_segment_finished) {
        ctrl_sampling_finished_ = true; // 强制标记完成
    }

    if (valid_cmd && !ctrl_sampling_finished_) {
        // LOG(INFO) << "NMPC Body Velocity: " << cmd_vel_body_;
        // LOG(INFO) << "NMPC WheelSpeed: " << wheel_speed_cmd_;
        // LOG(INFO) << "NMPC WheelSteer Angle: " << steer_angle_cmd_;
        limitAcceleration(cmd_vel_body_, last_mpc_vel_cmd_, max_acc_, delta_time_);
        computeIK(cmd_vel_body_, steer_angle_cmd_, wheel_speed_cmd_);
        last_mpc_vel_cmd_ = cmd_vel_body_;
        galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
    } 

    // 切换至减速模式
    if (ctrl_sampling_finished_) {
        // MPC停止
        {
            std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
             nmpc_data_.has_new_task = false; 
             nmpc_data_.mpc_state = MpcThreadState::FINISHED;
        }
        
        LOG(INFO) << "[Mode Switch] MPC Done (dist_to_goal: " << dist_to_goal
                  << "m, required_dist: " << required_dist
                  << "m, vel_norm: " << cur_vel_norm
                  << "m/s, wait: " << mpc_stop_wait_elapsed_
                  << "s, timeout: " << (wait_timeout ? "yes" : "no")
                  << "). Next Step: ALIGN.";
        next_control_mode_ = ControlMode::ALIGN;

        control_mode_ = next_control_mode_;

        // 将当前车体的局部速度，转换到全局坐标系，传递给 ALIGN
        double cur_yaw = current_pose_[2];
        last_align_vel_cmd_ = cmd_vel_body_;

        last_rot_vel_cmd_.setZero();
        mpc_stop_wait_elapsed_ = 0.0;
    }
 }

 void GalbotS1ChassisPoseNMPCController::runDecelerationMode() {
    const double vel_zero_threshold = 0.05; 
    // LOG(INFO) << "Current Vel: " << current_vel_.transpose();

    // 检查是否停稳
    bool is_linear_stopped = std::abs(current_vel_[0]) < vel_zero_threshold && std::abs(current_vel_[1]) < vel_zero_threshold;
    bool is_angular_stopped = std::abs(current_vel_[2]) < vel_zero_threshold;

    if (is_linear_stopped && is_angular_stopped) {
        LOG(INFO) << "[Deceleration] Robot Stopped. Switching to Mode ID: " << (int)next_control_mode_;
        
        std::vector<double> zero_speeds(wheel_num_, 0.0);
        std::vector<double> zero_steering_angles(wheel_num_, 0.0);

        // 如果下一控制模式是MPC，那么等待舵角回正
        if (next_control_mode_ == ControlMode::MPC_TRACKING || next_control_mode_ == ControlMode::ALIGN) {
            galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(zero_steering_angles, zero_speeds);
            // 判断当前角度有没有回正
            bool is_steer_angle_aligned = true;
            for (int i = 0; i < wheel_num_; i++) {
                double angle_diff = std::abs(normalizeAngle(current_steering_angles_[i] - zero_steering_angles[i]));
                if (angle_diff > align_angle_threshold_) {
                    is_steer_angle_aligned = false;
                    break;
                }
            }
            // 角度没有回正
            if (!is_steer_angle_aligned) return; 
        } else {
            // 旋转模式，或者对齐模式，保持当前角度
            std::vector<double> micro_speeds(wheel_num_, 0.001); // 1mm/s 微速
            galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(zero_steering_angles, micro_speeds);
        }

        control_mode_ = next_control_mode_;
        
        // 如果减速阶段没有唤醒MPC线程，这里再次唤醒
        if (control_mode_ == ControlMode::MPC_TRACKING) {
            std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
            if (nmpc_data_.mpc_state != MpcThreadState::RUNNING) {
                nmpc_data_.has_new_task = true;
            }
        }
        cmd_vel_body_.setZero();   
        last_align_vel_cmd_.setZero();
        last_rot_vel_cmd_.setZero();
        sliding_window_.clear();
        return;
    }
    // 同比例减速
    scalar_t current_speed = std::hypot(cmd_vel_body_[0], cmd_vel_body_[1]);
    scalar_t current_wz = std::abs(cmd_vel_body_[2]);

    scalar_t final_ratio = 1.0;

    if (current_speed > 1e-3) {
        scalar_t speed_drop = std::min(stop_acc_[0], stop_acc_[1]) * delta_time_;
        scalar_t ratio_xy = std::max(0.0, (current_speed - speed_drop) / current_speed);
        final_ratio = std::min(final_ratio, ratio_xy); 
    }

    if (current_wz > 1e-3) {
        scalar_t w_drop = stop_acc_[2] * delta_time_;
        scalar_t ratio_w = std::max(0.0, (current_wz - w_drop) / current_wz);
        final_ratio = std::min(final_ratio, ratio_w);
    }

    cmd_vel_body_[0] *= final_ratio;
    cmd_vel_body_[1] *= final_ratio;
    cmd_vel_body_[2] *= final_ratio;

    computeIK(cmd_vel_body_, steer_angle_cmd_, wheel_speed_cmd_);
    galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
}
 void GalbotS1ChassisPoseNMPCController::runAlignMode() {
    if (chassis_traj_interface_.empty()) return;

    auto last_target_pose = chassis_traj_interface_.back().pose;
    double last_target_yaw = quaternionToEulerZYX(last_target_pose.orientation)[0];

    vector_3t target_pt(last_target_pose.position.x(), last_target_pose.position.y(), last_target_yaw);
    vector_3t current_err(
            target_pt[0] - current_pose_[0],
            target_pt[1] - current_pose_[1],
            normalizeAngle(target_pt[2] - current_pose_[2])
        );

    updateErrors(current_err);

    if (isWindowValid()) {
        
        bool is_physically_stopped = std::abs(current_vel_[0]) < fast_vel_switch_threshold_ && 
                                     std::abs(current_vel_[1]) < fast_vel_switch_threshold_;
        if (is_physically_stopped){
            LOG(INFO) << "[Align] Target XY Reached. Error: " << current_err.transpose();
            scalar_t final_yaw_err = std::abs(normalizeAngle(last_target_yaw - current_pose_[2]));

            if (mid_mpc_end_idx_ < target_chassis_traj_.size() - 1 ||  final_yaw_err > rotate_yaw_threshold_) {
                LOG(INFO) << "-> Next Step: Final Rotation.";
                rotation_end_idx_ = last_rotation_end_idx_;
                rotation_target_yaw_ = quaternionToEulerZYX(target_chassis_traj_[rotation_end_idx_].pose.orientation)[0];
                
                control_mode_ = ControlMode::IN_PLACE_ROTATION; 

                std::vector<double> stop_vel(wheel_num_, 0.0);
                cmd_vel_body_.setZero();
                last_align_vel_cmd_.setZero();
                last_rot_vel_cmd_.setZero();
                sliding_window_.clear();

                steer_angle_cmd_ = current_steering_angles_;
                wheel_speed_cmd_ = stop_vel;
                galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);

            } else {
                // 不存在最后一段旋转，直接完成任务
                LOG(INFO) << "-> Task Completely Finished. Error: " << current_err.transpose(); 
                stop();
                wbc_interface_ptr_->wbc_data_ptr_->target_task_traj_map_["swerve_chassis"].target_finished = true;
                control_mode_ = ControlMode::FINISHED;
            }
            return;
        }
    }

    vector_3t raw_cmd_vel_global = vector_3t::Zero();
    scalar_t xy_err_norm = std::hypot(current_err[0], current_err[1]);

    if (xy_err_norm > 1e-4) {
        scalar_t max_xy_vel = std::min(max_vel_[0], max_vel_[1]);
        scalar_t handover_acc = std::min(align_acc_[0], align_acc_[1]) * 0.8;
        // scalar_t target_speed_sqrt = std::sqrt(2.0 * handover_acc * xy_err_norm);
        scalar_t target_speed = align_speed_p_gain_ * xy_err_norm;

        // scalar_t target_speed = std::min(target_speed_sqrt, target_speed_p);
        target_speed = std::clamp(target_speed, 0.0, max_xy_vel);

        raw_cmd_vel_global[0] = (current_err[0] / xy_err_norm) * target_speed;
        raw_cmd_vel_global[1] = (current_err[1] / xy_err_norm) * target_speed;
    }

    double cur_yaw = current_pose_[2];
    vector_3t raw_cmd_vel_body = vector_3t::Zero();
    raw_cmd_vel_body[0] = raw_cmd_vel_global[0] * std::cos(cur_yaw) + raw_cmd_vel_global[1] * std::sin(cur_yaw);
    raw_cmd_vel_body[1] = -raw_cmd_vel_global[0] * std::sin(cur_yaw) + raw_cmd_vel_global[1] * std::cos(cur_yaw);
    raw_cmd_vel_body[2] = 0.0;

    stdvec_scalar_t dynamic_limit_acc = align_acc_;

    if ((cmd_vel_body_[0] > 0 && raw_cmd_vel_body[0] < cmd_vel_body_[0]) || 
        (cmd_vel_body_[0] < 0 && raw_cmd_vel_body[0] > cmd_vel_body_[0])) {
        // X方向需要刹车，使用刹车加速度
        dynamic_limit_acc[0] = std::max(align_acc_[0], stop_acc_[0]);
    }
    if ((cmd_vel_body_[1] > 0 && raw_cmd_vel_body[1] < cmd_vel_body_[1]) || 
        (cmd_vel_body_[1] < 0 && raw_cmd_vel_body[1] > cmd_vel_body_[1])) {
        dynamic_limit_acc[1] = std::max(align_acc_[1], stop_acc_[1]);
    }
    if ((cmd_vel_body_[2] > 0 && raw_cmd_vel_body[2] < cmd_vel_body_[2]) ||
        (cmd_vel_body_[2] < 0 && raw_cmd_vel_body[2] > cmd_vel_body_[2])) {
        dynamic_limit_acc[2] = std::max(align_acc_[2], stop_acc_[2]);
    }

    limitAcceleration(raw_cmd_vel_body, last_align_vel_cmd_, dynamic_limit_acc, delta_time_);
    last_align_vel_cmd_ = raw_cmd_vel_body;
    cmd_vel_body_ = raw_cmd_vel_body;
    
    computeIK(cmd_vel_body_, steer_angle_cmd_, wheel_speed_cmd_);

    galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);

 }
// 处理新轨迹
void GalbotS1ChassisPoseNMPCController::newTrajProcess()
{
    // 更新共享轨迹数据
    {
        std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
        has_last_odom_pose_ = false;
        has_last_map_pose_ = false;

        // 先暂停 MPC 任务标志，防止线程在数据更新中途读取
        nmpc_data_.has_new_task = false; 
        nmpc_data_.mpc_state = MpcThreadState::IDLE; 

        // 安全地更新轨迹数据
        target_chassis_traj_.clear();
        target_chassis_traj_ = chassis_traj_interface_;
        target_traj_size_ = target_chassis_traj_.size();
    }

    // 重置变量
    cur_excute_idx_ = 0;
    last_start_idx_ = 0;
    cur_start_idx_ = 0;
    last_yaw_err_ = 0.0;
    yaw_err_integral_ = 0.0;
    last_align_err_.setZero();
    last_align_vel_cmd_.setZero();
    last_rot_vel_cmd_.setZero();
    align_err_integral_.setZero();
    ctrl_sampling_finished_ = false;
    mpc_stop_wait_elapsed_ = 0.0;
    align_finished_ = false;                 // 启动对齐模式时，初始化对齐完成为false
    next_control_mode_ = ControlMode::IDLE;  // 新轨迹启动时，初始化下一个控制模式为IDLE

    // 寻找旋转跳点 & 计算角度差
    // rotation_end_idx_ = findRotationCutoffIndex();
    // 判断从轨迹得到的跳点是否有效，如果小于等于0，直接进入MPC跟踪
    if (last_rotation_end_idx_ >= target_chassis_traj_.size()) last_rotation_end_idx_ = target_chassis_traj_.size() - 1;
    if (last_rotation_end_idx_ < 0) last_rotation_end_idx_ = 0;

    if (first_rotation_end_idx_ >= target_chassis_traj_.size()) first_rotation_end_idx_ = target_chassis_traj_.size() - 1;
    if (mid_mpc_end_idx_ >= target_chassis_traj_.size()) mid_mpc_end_idx_ = target_chassis_traj_.size() - 1;

    if (target_chassis_traj_.empty()) return;

    // 决策逻辑 (去掉了减速，直接切模式)
    if (first_rotation_end_idx_ > 0) {
        rotation_target_yaw_ = quaternionToEulerZYX(target_chassis_traj_[first_rotation_end_idx_].pose.orientation)[0];
        rotation_end_idx_ = first_rotation_end_idx_; // 复用这个变量作为当前旋转段的终点

        LOG(INFO) << "[Mode Switch] Rotation Detect. Cutoff Index: " << rotation_end_idx_ << ", Target Yaw: " << rotation_target_yaw_;
        // 进入旋转模式
        control_mode_ = ControlMode::IN_PLACE_ROTATION;

    } else {

        // 判断mpc的结束和旋转结束之间的距离，如果太短，不走mpc
        auto pt_mpc_begin = target_chassis_traj_[first_rotation_end_idx_].pose.position;
        auto pt_mpc_end   = target_chassis_traj_[mid_mpc_end_idx_].pose.position;
        double dis_from_rotation_end = std::hypot(pt_mpc_begin.x() - pt_mpc_end.x(), pt_mpc_begin.y() - pt_mpc_end.y());

        bool is_mpc_segment_valid = mid_mpc_end_idx_ > first_rotation_end_idx_;
        bool is_dist_long_enough = dis_from_rotation_end >= min_mpc_distance_threshold_;
        if (is_mpc_segment_valid && is_dist_long_enough){
            LOG(INFO) << "[Mode Switch] Direct MPC Tracking.";
            rotation_end_idx_ = 0;
            control_mode_ = ControlMode::MPC_TRACKING;
            {
                std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
                last_mpc_vel_cmd_ = current_vel_;
                nmpc_data_.has_new_task = true; // 立即唤醒 MPC
            }
        } else {
            LOG(INFO) << "-> Distance too short. Skip MPC, Jump to ALIGN directly.";
            // 进入最后一段的旋转模式
            control_mode_ = ControlMode::ALIGN;
            cmd_vel_body_.setZero();
        }

    }
}

void GalbotS1ChassisPoseNMPCController::inverseKinematicsForSpinMode(
    vector_3t &target_vel,
    std::vector<double> &output_steer_angles,
    std::vector<double> &output_wheel_vels)
{
    double wz = target_vel[2];

    output_steer_angles.resize(wheel_num_);
    output_wheel_vels.resize(wheel_num_);

    for (int i = 0; i < wheel_num_; ++i) {
        const vector_2t &ri = wheel_pos_[i];
        double vwx = - wz * ri.y();
        double vwy =   wz * ri.x();

        double v_magnitude = std::sqrt(vwx * vwx + vwy * vwy);
        double target_theta = std::atan2(vwy, vwx); 

        // 舵角超限时反向补偿
        double theta_a = normalizeAngle(target_theta);
        double theta_b = normalizeAngle(target_theta + M_PI);
        if (std::abs(theta_a) > M_PI / 2.0) {
            target_theta = theta_b;
            v_magnitude = -v_magnitude;
        }

        target_theta = std::clamp(target_theta, -max_steer_angle_, max_steer_angle_);
        output_steer_angles[i] = target_theta;
        output_wheel_vels[i] = v_magnitude / wheel_radius_;
    }
}

void GalbotS1ChassisPoseNMPCController::runInPlaceRotationMode() {

    // 检查并执行舵角对齐
    vector_3t rot_cmd_dummy(0.0, 0.0, 1.0); 
    std::vector<double> target_align_angles, target_align_speeds;

    inverseKinematicsForSpinMode(rot_cmd_dummy, target_align_angles, target_align_speeds);
    
    // 检查当前舵角是否到位
    bool all_wheels_aligned = true;
    for (int i = 0; i < wheel_num_; ++i) {
        double diff = std::abs(normalizeAngle(current_steering_angles_[i] - target_align_angles[i]));
        if (diff > M_PI_2) diff = std::abs(diff - M_PI);
        if (diff > align_angle_threshold_) {
            all_wheels_aligned = false;
            break;
        }
    }

    // 如果没对齐，发零速，只转舵
    if (!all_wheels_aligned) {
        std::vector<double> micro_vel(wheel_num_, 0.001);

        wheel_speed_cmd_ = micro_vel;
        steer_angle_cmd_ = target_align_angles;

        cmd_vel_body_.setZero(); 
        last_rot_vel_cmd_.setZero();
        galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
        return;
    }

    double cur_yaw = current_pose_[2];
    double yaw_err = normalizeAngle(rotation_target_yaw_ - cur_yaw);
    LOG(INFO) << "[Rotation] Yaw Error: " << yaw_err << ", Target Yaw: " << rotation_target_yaw_ << ", Current Yaw: " << cur_yaw;

    // 检查是否旋转到位 -> 切换 MPC or 结束任务
    if (std::abs(yaw_err) < rotate_yaw_threshold_) {
        LOG(INFO) << "[Mode Switch] Rotation Done.";

        // 判断mpc的结束和旋转结束之间的距离，如果太短，不走mpc
        auto pt_mpc_begin = target_chassis_traj_[first_rotation_end_idx_].pose.position;
        auto pt_mpc_end   = target_chassis_traj_[mid_mpc_end_idx_].pose.position;
        double dis_from_rotation_end = std::hypot(pt_mpc_begin.x() - pt_mpc_end.x(), pt_mpc_begin.y() - pt_mpc_end.y());

        bool is_mpc_segment_valid = mid_mpc_end_idx_ > first_rotation_end_idx_;
        bool is_dist_long_enough = dis_from_rotation_end >= min_mpc_distance_threshold_;

        if (rotation_end_idx_ == first_rotation_end_idx_){
            // 当前为第一段旋转，根据距离判断后面走MPC或者最后一段旋转
            if (is_mpc_segment_valid && is_dist_long_enough) {
                // 进入MPC
                LOG(INFO) << "-> Next Step: MPC TRACKING.";
                next_control_mode_ = ControlMode::MPC_TRACKING;
            
                // 同步更新所有索引，防止 MPC 掉头
                last_start_idx_ = rotation_end_idx_;
                cur_start_idx_ = rotation_end_idx_;
                cur_excute_idx_ = rotation_end_idx_; // NMPC线程读取这个作为起始点
                
                // 开启 MPC 线程
                {
                    std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
                    last_mpc_vel_cmd_ = current_vel_;
                    nmpc_data_.has_new_task = true; 
                }
            } else {
                // 第一段旋转完，发现距离太短，跳过MPC，直接进入下一阶段：平移对齐 (ALIGN)
                LOG(INFO) << "-> MPC Distance too short (" << dis_from_rotation_end 
                            << "m). Skipping MPC Phase. Next Step: ALIGN.";
                next_control_mode_ = ControlMode::ALIGN;
            }
        } else {
            // 全部任务彻底结束
            vector_3t current_error {
                current_pose_[0] - target_chassis_traj_.back().pose.position.x(),
                current_pose_[1] - target_chassis_traj_.back().pose.position.y(),
                normalizeAngle(current_pose_[2] - rotation_target_yaw_)
            };
            LOG(INFO) << "-> Final Rotation Done. Task Completely Finished. Error: " << current_error.transpose();
            stop();
            wbc_interface_ptr_->wbc_data_ptr_->target_task_traj_map_["swerve_chassis"].target_finished = true;
            control_mode_ = ControlMode::FINISHED;
            return; 
        }

        // 判断当前的速度是否很低，可以直接进入下个模式
        bool is_physically_stopped = std::abs(current_vel_[0]) < fast_vel_switch_threshold_ && 
                                     std::abs(current_vel_[1]) < fast_vel_switch_threshold_ && 
                                     std::abs(current_vel_[2]) < fast_vel_switch_threshold_;
        
        // 检查舵角是否已经在 0 位附近
        bool is_steer_zeroed = true;
        for (std::size_t i = 0; i < wheel_num_; i++) {
            if (std::abs(normalizeAngle(current_steering_angles_[i])) > align_angle_threshold_) {
                is_steer_zeroed = false;
                break;
            }
        }
        bool need_zeroing = (next_control_mode_ == ControlMode::MPC_TRACKING || next_control_mode_ == ControlMode::ALIGN);

        if (is_physically_stopped && (!need_zeroing || is_steer_zeroed)) {
            LOG(INFO) << "[Fast Track] Robot already stopped and wheels ready. Skip Deceleration -> " << (int)next_control_mode_;
            control_mode_ = next_control_mode_;
            cmd_vel_body_.setZero();
            last_align_vel_cmd_.setZero();
            last_rot_vel_cmd_.setZero();
            sliding_window_.clear();

            std::vector<double> stop_v(wheel_num_, 0.0);
            std::vector<double> zero_angle(wheel_num_, 0.0);
            galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(zero_angle, stop_v);
        } else {
            control_mode_ = ControlMode::DECELERATION;
        }
        return;
    }
    // 积分分离和限幅
    if(std::abs(yaw_err) < yaw_err_threshold_) {
        yaw_err_integral_ += yaw_err * delta_time_;
        yaw_err_integral_ = std::clamp(yaw_err_integral_, -align_and_rotate_integral_limit_[2], align_and_rotate_integral_limit_[2]);
    } else {
        yaw_err_integral_ = 0.0;
    }

    double yaw_err_rate = (yaw_err - last_yaw_err_) / delta_time_; // 角速度
    last_yaw_err_ = yaw_err;                                       // 更新上一次的误差

    double target_omega = rotate_kp_ * yaw_err + rotate_kd_ * yaw_err_rate + rotate_ki_ * yaw_err_integral_;
    //LOG(INFO) << "[Rotation] Yaw Error: " << yaw_err << ", Target Yaw: " << rotation_target_yaw_ << ", Current Yaw: " << cur_yaw << ", Target Omega: " << target_omega;

    target_omega = std::clamp(target_omega, -max_vel_[2], max_vel_[2]);

    vector_3t raw_cmd_vel_body = {0.0, 0.0, target_omega};
    limitAcceleration(raw_cmd_vel_body, last_rot_vel_cmd_, align_acc_, delta_time_);
    last_rot_vel_cmd_ = raw_cmd_vel_body;
    cmd_vel_body_ = raw_cmd_vel_body;
    // 这里再次调用 IK 计算速度 (舵角其实已经对齐了，微调即可)
    inverseKinematicsForSpinMode(cmd_vel_body_, steer_angle_cmd_, wheel_speed_cmd_);
    galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
}


// MPC线程
void GalbotS1ChassisPoseNMPCController::nmpcTaskLoop() {
    const auto period = std::chrono::milliseconds(static_cast<int>(mpc_cycle_time_ * 1000));
    int cycle_ms = static_cast<int>(mpc_cycle_time_ * 1000);
    if (cycle_ms <= 0) cycle_ms = 20; // 二次保护

    std::vector<std::vector<double>> ref_copy;
    std::vector<StampedControl> new_ctrls;
    std::vector<Frame> current_target_traj;
    new_ctrls.reserve(nmpc_horizon_ + 5);

    // 准备局部缓存变量，避免在锁内分配内存
    std::vector<std::vector<double>> local_ref_horizon;
    std::vector<int> local_ref_idx;
    std::vector<double> local_ref_time;
    // 预分配内存，减少动态分配耗时
    local_ref_horizon.reserve(nmpc_horizon_ + 5);
    local_ref_idx.reserve(nmpc_horizon_ + 5);
    local_ref_time.reserve(nmpc_horizon_ + 5);


    while (nmpc_thread_running_) {
        auto start_time = std::chrono::steady_clock::now();

        int local_excute_idx = 0;
        bool can_run = false;
        bool is_first_frame_of_new_task = false;
        std::vector<double> state_snapshot;
        bool should_sleep = false;
        {
            auto t_lock_start = std::chrono::steady_clock::now();
            // 如果没有新任务且处于空闲/完成状态，休眠
            std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
            if((nmpc_data_.mpc_state == MpcThreadState::IDLE ||
                nmpc_data_.mpc_state == MpcThreadState::FINISHED) && 
                !nmpc_data_.has_new_task)
            {
                should_sleep = true;
            }

            // 处理新任务标志
            if (nmpc_data_.has_new_task) {
                nmpc_data_.has_new_task = false;
                is_first_frame_of_new_task = true;  // 新的帧
                local_excute_idx = cur_excute_idx_; // 确保它使用最新的索引
                current_target_traj = target_chassis_traj_;
                nmpc_data_.mpc_state = MpcThreadState::RUNNING;
                LOG(INFO)<< "MPC Task Start. Index: " << local_excute_idx;
                should_sleep = false; // 有新任务，肯定不睡
            }

        }
        if (should_sleep) {
            std::this_thread::sleep_until(start_time + period);
            continue; // 进入下一次循环
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // 读取定位
            if (is_pose_initialized_) {
                state_snapshot = current_state_vec_;
                local_excute_idx = cur_excute_idx_;
                can_run = true;
            }
        }

        // 
        if (can_run && !current_target_traj.empty() && local_excute_idx < current_target_traj.size()) {
                // 【无锁计算】生成参考轨迹到局部变量
                // 注意：不要在这里加 nmpc_data_mutex_ 锁！
                // 这里只读 current_target_traj (线程局部副本) 和 state_snapshot (也是副本)，绝对线程安全
                LOG(INFO) << "local excute idx: " << local_excute_idx;
                generateReferenceHorizon(current_target_traj, local_excute_idx, nmpc_horizon_, state_snapshot,
                                        local_ref_horizon, local_ref_idx, local_ref_time, mid_mpc_end_idx_);

                // bool is_finished = (local_ref_idx.back() == current_target_traj.size() - 1);

                // 加锁,只做数据交换/赋值
                {
                    std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
                    
                    nmpc_data_.ref_horizon = local_ref_horizon;
                    nmpc_data_.ref_horizon_idx = local_ref_idx;
                    nmpc_data_.ref_horizon_time = local_ref_time;
                    nmpc_data_.init_state_vec = state_snapshot;
                    
                    ref_copy = local_ref_horizon; 

                    // if(is_finished) nmpc_data_.mpc_state = MpcThreadState::FINISHED;
                }

           if (!nmpc_core_) {
                LOG(ERROR) << "NMPC core is not initialized!";
                continue;
            }

            // MPC求解
            MpcResult res = nmpc_core_->control(ref_copy, state_snapshot);
            auto end_time = std::chrono::steady_clock::now();
            LOG(INFO)<< "mpc time:  "<< std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " ms";

            new_ctrls.clear();

            {
                std::lock_guard<std::mutex> lock(nmpc_data_mutex_);
                nmpc_data_.result = res;
                size_t num_ctrls = std::min(res.out_control.size(), nmpc_data_.ref_horizon_idx.size());
                // 从第一个开始赋值
                for(size_t i = 1; i < num_ctrls; ++i) {
                    new_ctrls.push_back({
                        nmpc_data_.ref_horizon_idx[i], // 索引
                        nmpc_data_.ref_horizon_time[i], // 时间
                        res.out_control[i][0], res.out_control[i][1], res.out_control[i][2] // 控制量
                    });
                }
            }

            {
                std::lock_guard<std::mutex> lock(traj_buffer_mutex_);
                // 打印参考轨迹 
                LOG(INFO)<<"ref traj:";
                for(int i = 0;i<nmpc_data_.ref_horizon.size();i++){
                    LOG(INFO)<< nmpc_data_.ref_horizon[i][0]<<" "<<nmpc_data_.ref_horizon[i][1]<<" "<<nmpc_data_.ref_horizon[i][2]<<" "
                             << nmpc_data_.ref_horizon[i][3]<<" "<<nmpc_data_.ref_horizon[i][4]<<" "<<nmpc_data_.ref_horizon[i][5];
                }
                // 打印预测轨迹
                LOG(INFO) << "Res Status: " << res.status;
                LOG(INFO)<<"pridect traj:";
                for(int i = 0; i < res.out_control.size(); i++){
                    LOG(INFO)<< res.result_traj[i][0]<<" "<<res.result_traj[i][1]<<" "<<res.result_traj[i][2]<<" "
                             << res.result_traj[i][3]<<" "<<res.result_traj[i][4]<<" "<<res.result_traj[i][5];
                }
                LOG(INFO)<<"control traj:";
                for(int i = 0; i < res.out_control.size(); i++){
                    LOG(INFO)<<  res.out_control[i][0]<<" "<<res.out_control[i][1]<<" "<<res.out_control[i][2];
                }

                // =====================================================

                // 第一帧数据，全部覆盖
                if (is_first_frame_of_new_task) {
                    control_traj_buffer_.clear();
                    control_traj_buffer_.insert(control_traj_buffer_.end(), new_ctrls.begin(), new_ctrls.end());
                    
                } else {
                    if (!new_ctrls.empty()) {
                        int new_ctl_start_idx = new_ctrls.front().idx;
                        if(new_ctl_start_idx < control_traj_buffer_.front().idx){
                            LOG(INFO)<< "new ctl start idx less than control buffer idx!";
                        }

                        control_traj_buffer_.clear();
                        control_traj_buffer_.insert(control_traj_buffer_.end(), new_ctrls.begin(), new_ctrls.end());

                        // auto it = std::lower_bound(control_traj_buffer_.begin(), control_traj_buffer_.end(), new_ctrls.front().idx,
                        //     [](const StampedControl& c, double idx) { return c.idx < idx; });
                        
                        // if (it != control_traj_buffer_.end()) {
                        //     control_traj_buffer_.erase(it, control_traj_buffer_.end());
                        // }
                        // for(auto & ctl:new_ctrls)
                        //     control_traj_buffer_.push_back(ctl);      
                            
                    } else {
                        // 如果 new_ctrls 为空 (MPC求解失败或未完成)，不清空 control_traj_buffer_，
                        // 这样主线程还能继续用旧 Buffer 里的剩余指令跑一会
                        LOG(INFO) << "MPC generated empty controls, keeping old buffer.";
                    }
                }
            }
            LOG(INFO)<< "control buffer size:  "<< control_traj_buffer_.size()<<"  start idx: "<< control_traj_buffer_.front().idx<<"  end idx: "<< control_traj_buffer_.back().idx;
            
            // for(int i = 0;i<control_traj_buffer_.size();i++){
            //     LOG(INFO)<<" i:  "<< control_traj_buffer_[i].idx<<"  "<<control_traj_buffer_[i].vx<<"  "<<control_traj_buffer_[i].vy<<"  "<<control_traj_buffer_[i].omega;
            // }
        }
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = end_time - start_time;
        std::this_thread::sleep_for(period - elapsed);
    }
}

// 获得MPC参考时域轨迹
void GalbotS1ChassisPoseNMPCController::generateReferenceHorizon(std::vector<Frame> &target_chassis_traj,size_t start_idx, size_t N, std::vector<double>& cur_pose,
                                                                std::vector<std::vector<double>>& out_horizon, std::vector<int>& out_horizon_idx,                   
                                                                std::vector<double>& out_horizon_time, size_t virtual_end_idx) {
    
    // 清空传入的局部变量
    out_horizon.clear();
    out_horizon_idx.clear();
    out_horizon_time.clear();

    if (target_chassis_traj.empty()) return;

    // 根据当前位置，计算起始索引
    std::size_t actual_start_idx = getNextMpcStartIndex(target_chassis_traj.size(),start_idx,mpc_delay_cycle_num_);
    std::vector<vector_3t> temp_path;

    // 填充滚动时域内的点
    // 添加当前点[总长度：rolling_horizen_size_+1,不加当前点]
    Frame start_frame = target_chassis_traj[actual_start_idx];
    double start_traj_yaw = normalizeAngle(quaternionToEulerZYX(start_frame.pose.orientation)[0]);
    double continuous_ref_yaw = start_traj_yaw + std::round((cur_pose[2] - start_traj_yaw) / (2.0 * M_PI)) * 2.0 * M_PI;
    
    // 保证idx不会超过轨迹长度
    size_t limit_idx = std::min(virtual_end_idx, target_chassis_traj.size() - 1);
    for(std::size_t k = 0; k <= N; k++)
    {
        std::size_t idx = actual_start_idx + k;

        // 进行idx截断，如果超过，则赋值为限制长度
        if (idx > limit_idx) idx = limit_idx;
        Frame target_frame;
        double pt_time = 0.0;

        if(idx >= target_chassis_traj.size()){
            target_frame = target_chassis_traj.back(); 
            pt_time = (actual_start_idx + k) * traj_step_dt_;
        }               
        else{
            target_frame = target_chassis_traj[idx];
            pt_time = idx * traj_step_dt_;
        }            

        double ref_target_yaw = normalizeAngle(quaternionToEulerZYX(target_frame.pose.orientation)[0]);
        if (!temp_path.empty()) {
            double diff = normalizeAngle(ref_target_yaw - continuous_ref_yaw);
            continuous_ref_yaw = continuous_ref_yaw + diff;
        }

        temp_path.push_back({target_frame.pose.position.x(), target_frame.pose.position.y(), continuous_ref_yaw});
        // nmpc_data_.ref_horizon_idx.push_back(std::min((int)idx, (int)(target_chassis_traj.size() - 1)));
        // nmpc_data_.ref_horizon_time.push_back(pt_time);
        out_horizon_idx.push_back(std::min((int)idx, (int)(target_chassis_traj.size() - 1)));
        out_horizon_time.push_back(pt_time);
    }
    //=========================转成MPC路径参数====================== 
    for (size_t i = 0; i < temp_path.size(); ++i) {
        double vx = 0, vy = 0, dyaw = 0;
        if (i < temp_path.size() - 1) {
            vx = (temp_path[i+1].x() - temp_path[i].x()) / traj_step_dt_;
            vy = (temp_path[i+1].y() - temp_path[i].y()) / traj_step_dt_;
            dyaw = normalizeAngle(temp_path[i+1].z() - temp_path[i].z()) / traj_step_dt_;
        } else {
            // 最后一个点
            if (i > 0) {
                 const auto& last_ref = out_horizon.back();
                 vx = last_ref[3]; vy = last_ref[4]; dyaw = last_ref[5];
            }
        }
        out_horizon.push_back({temp_path[i].x(), temp_path[i].y(), temp_path[i].z(), vx, vy, dyaw});
    }

    size_t clamp_start = out_horizon_idx.size();
    for (size_t i = 0; i < out_horizon_idx.size(); ++i) {
        if ((size_t)out_horizon_idx[i] >= limit_idx) { 
            clamp_start = i; 
            break; 
        }
    }

    if (clamp_start > 0 && clamp_start < out_horizon.size()) {
        // 获取截断前一刻的有效速度
        double v0x = out_horizon[clamp_start - 1][3];
        double v0y = out_horizon[clamp_start - 1][4];
        double v0w = out_horizon[clamp_start - 1][5];
        
        size_t tail_len = out_horizon.size() - clamp_start;
        for (size_t i = 0; i < tail_len; ++i) {
            double r = 1.0 - double(i + 1) / double(tail_len);  // 1.0 -> 0.0 线性衰减
            
            out_horizon[clamp_start + i][3] = v0x * r;
            out_horizon[clamp_start + i][4] = v0y * r;
            out_horizon[clamp_start + i][5] = v0w * r;

            double prev_x   = out_horizon[clamp_start + i - 1][0];
            double prev_y   = out_horizon[clamp_start + i - 1][1];
            double prev_yaw = out_horizon[clamp_start + i - 1][2];

            out_horizon[clamp_start + i][0] = prev_x + out_horizon[clamp_start + i][3] * traj_step_dt_;
            out_horizon[clamp_start + i][1] = prev_y + out_horizon[clamp_start + i][4] * traj_step_dt_;
            out_horizon[clamp_start + i][2] = prev_yaw + out_horizon[clamp_start + i][5] * traj_step_dt_;
        }
    }

}

/*
* 根据当前位置计算下一次MPC的起始索引
* 输入：last_start_idx 上一次起始索引
*      cur_state 当前状态向量 [x, y, yaw, vx, vy, dyaw] 用于做预测
*      mpc_delay_cycle_num MPC的求解延迟帧数
* 输出：下次MPC的起始索引
*/
std::size_t GalbotS1ChassisPoseNMPCController::getNextMpcStartIndex(size_t target_traj_size, size_t cur_start_idx,int delay_num)const
{
    size_t idx = cur_start_idx + delay_num;
    if (idx >= target_traj_size) return target_traj_size - 1;
    return idx;
}
void GalbotS1ChassisPoseNMPCController::computeIK(const vector_3t& body_vel, 
                                           std::vector<double>& angles, 
                                           std::vector<double>& speeds) {
    double vx = body_vel[0];
    double vy = body_vel[1];
    double wz = body_vel[2];

    angles.resize(wheel_num_);
    speeds.resize(wheel_num_);

    const double speed_lock_thresh = 0.03;

    double max_steer_step = nmpc_cfg_.max_steer_rate * M_PI * delta_time_;

    std::vector<double> temp_target_angles(wheel_num_, 0.0);
    std::vector<double> temp_target_speeds(wheel_num_, 0.0);
    
    double global_cos_scale = 1.0; 

    for (int i = 0; i < wheel_num_; ++i) {
        double wx = vx - wz * wheel_pos_[i].y();
        double wy = vy + wz * wheel_pos_[i].x();
        
        double v_mag = std::hypot(wx, wy);
        double current_theta = normalizeAngle(current_steering_angles_[i]);

        if (v_mag < speed_lock_thresh) {
            temp_target_angles[i] = current_theta;
            temp_target_speeds[i] = 0.0;
            continue;
        }

        double target_theta = std::atan2(wy, wx);
        double delta = normalizeAngle(target_theta - current_theta);

        if (std::fabs(delta) > M_PI / 2.0) {
            if (delta > 0) target_theta -= M_PI;
            else           target_theta += M_PI;
            v_mag = -v_mag; 
            delta = normalizeAngle(target_theta - current_theta); 
        }

        scalar_t expected_angle = current_theta + delta;
        if (expected_angle > max_steer_angle_ || expected_angle < -max_steer_angle_) {
            if (target_theta > 0) target_theta -= M_PI;
            else                  target_theta += M_PI;
            v_mag = -v_mag;
            delta = normalizeAngle(target_theta - current_theta);
            expected_angle = current_theta + delta;
        }

        double limited_delta = std::clamp(delta, -max_steer_step, max_steer_step);
        double next_theta = current_theta + limited_delta;

        next_theta = std::clamp(next_theta, -max_steer_angle_, max_steer_angle_);
        temp_target_angles[i] = next_theta;
        temp_target_speeds[i] = v_mag;

        double remaining_error = normalizeAngle(expected_angle - next_theta);
        double current_cos = std::max(0.0, std::cos(remaining_error));

        if (current_cos < global_cos_scale) {
            global_cos_scale = current_cos;
        }
    }

    for (int i = 0; i < wheel_num_; ++i) {
        angles[i] = temp_target_angles[i]; 
        speeds[i] = temp_target_speeds[i] * global_cos_scale/ wheel_radius_;
    }
}

// --- 停止逻辑 ---
bool GalbotS1ChassisPoseNMPCController::stop(bool blocking) {

    if (controller_state_ == wbc::ControllerState::ControllerStateRunning) {
        controller_state_ = wbc::ControllerState::ControllerStateStopping;
        try {
            auto future = thread_pool_task_manager_.submit(this, &GalbotS1ChassisPoseNMPCController::slowStop);
            if (blocking) {
                return future.get();
            }
        } catch (...) {
            return false;
        }
    }

    // 销毁nmpc计算专用线程
    if (nmpc_thread_running_) {
        nmpc_thread_running_ = false;
        if (nmpc_thread_.joinable()) {
            nmpc_thread_.join();
        }
    }

    // 释放NMPC核心资源
    nmpc_core_.reset();

    return true;
}

bool GalbotS1ChassisPoseNMPCController::slowDown(scalar_t& vel, scalar_t acc, double dt) {
    scalar_t adv = std::fabs(acc * dt);
    if (vel > adv) {
        vel -= adv;
        return false;
    } else if (vel < -adv) {
        vel += adv;
        return false;
    } else {
        vel = 0;
        return true;
    }
}

bool GalbotS1ChassisPoseNMPCController::slowStop()
{
    Rate rate(25);
    // TODO: follow the last traj ref odom
    bool stopped = false;
    while (!stopped && (!thread_pool_task_manager_.is_shutting_down() || slow_stop_when_shutdown_)) {
        stopped = true;
        stopped &= slowDown(cmd_vel_body_[0], stop_acc_[0], rate.dt());
        stopped &= slowDown(cmd_vel_body_[1], stop_acc_[1], rate.dt());
        stopped &= slowDown(cmd_vel_body_[2], stop_acc_[2], rate.dt());
        computeIK(cmd_vel_body_, steer_angle_cmd_, wheel_speed_cmd_);
        LOG(WARNING) << "chassis stop vel: " << cmd_vel_body_[0] << ", " << cmd_vel_body_[1] << ", " << cmd_vel_body_[2];
        galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
        rate.sleep();
    }
    for (int i = 0; i < wheel_num_; i++){
        wheel_speed_cmd_[i] = 0.0;
        steer_angle_cmd_[i] = 0.0;
    }
    galbot_s1_chassis_interface_ptr_->chassis_group_com_ptr_->setSwerveChassisCommand(steer_angle_cmd_, wheel_speed_cmd_);
    controller_state_ = wbc::ControllerState::ControllerStateStopped;
    return stopped;
}

// 通过比例投影选择路径点索引
int GalbotS1ChassisPoseNMPCController::selectIdxByProjRatio(std::size_t &last_start_idx,const vector_3t& cur_pos, size_t min_idx)
{
    // 边界检查
    if(target_chassis_traj_.size() < 2) return 0;
    if(last_start_idx == target_chassis_traj_.size()-1)
        return last_start_idx;

    std::size_t cur_idx = last_start_idx;
    
    std::size_t search_limit = 20; // 最大搜索步数，防止极端情况卡死
    std::size_t count = 0;

    // 逻辑：更新记忆索引为 cur_idx，但返回目标为 cur_idx + look_ahead_cycles_
    auto getSafeReturnIdx = [&](int idx) {
        last_start_idx = idx; // 记忆当前所在的线段起点
        // 返回 idx + look_ahead_cycles_，但在终点处钳位
        std::size_t target = idx + look_ahead_cycles_;
        if (target >= target_chassis_traj_.size()) target = target_chassis_traj_.size() - 1;
        return target;
    };
    // =================================================
    // =================================================
    while (count < search_limit) {
        // 获取当前线段端点
        vector_3t leftFramePt(target_chassis_traj_[cur_idx].pose.position.x(),
                              target_chassis_traj_[cur_idx].pose.position.y(),
                              quaternionToEulerZYX(target_chassis_traj_[cur_idx].pose.orientation)[0]);
        vector_3t rightFramePt(target_chassis_traj_[cur_idx+1].pose.position.x(),
                               target_chassis_traj_[cur_idx+1].pose.position.y(),
                               quaternionToEulerZYX(target_chassis_traj_[cur_idx+1].pose.orientation)[0]);
        // 计算投影比例
        double ratio = projectionRatio(cur_pos, leftFramePt, rightFramePt);
        
        // --- Case A: 跑过头了 (ratio=1) -> 后搜 ---
        if (std::fabs(ratio - 1.0) < 1e-6) {
            if (cur_idx >= target_chassis_traj_.size() - 2) return getSafeReturnIdx(cur_idx); // 到底了
            cur_idx++; 
            // LOG(INFO) << " cur_idx+: "<<cur_idx<<" project ratio:  "<< ratio;
        }
        // --- Case B: 还没到 (ratio=0) -> 前搜 ---
        else if (std::fabs(ratio) < 1e-6) {
            if (cur_idx <= min_idx) return getSafeReturnIdx(min_idx);  // 防止回退到之前的旋转段
            if (cur_idx <= 0) return getSafeReturnIdx(0);             // 到头了
            cur_idx--; 
            // LOG(INFO) << " cur_idx-: "<<cur_idx<<" project ratio:  "<< ratio;
        }
        // --- Case C: 命中 (0 < ratio < 1) ---
        else {
            // LOG(INFO) << " cur_idx: "<<cur_idx<<" project ratio:  "<< ratio;
            return getSafeReturnIdx(cur_idx);
        }
        count++;
    }
    return getSafeReturnIdx(cur_idx);
}
double GalbotS1ChassisPoseNMPCController::projectionRatio(const vector_3t& p, 
                                                        const vector_3t& left, 
                                                        const vector_3t& right)
{
    vector_2t dir = right.head<2>() - left.head<2>();
    double len = dir.norm(); 
    double yaw_diff = fabs(normalizeAngle(right.z() - left.z()));
    double s = 0.0;
    if (len <= 1e-6 && yaw_diff <= 1e-6) s = 1.0;
    else if(len < 1e-6 && yaw_diff > 1e-6) s = fabs(normalizeAngle(p.z() - left.z())) / yaw_diff;
    else s = (p.head<2>() - left.head<2>()).dot(dir) / (len * len);
    // 截断逻辑
    if (s < 0.0) s = 0.0;
    else if (s > 1.0) s = 1.0;
    return s;
}

/*
* 从控制队列中获取指令
* Inout: 当前索引 curExcuteIdx
* Output: 控制指令 cmd_out
          bool: 是否成功
*/ 
bool GalbotS1ChassisPoseNMPCController::getCmdFromControlDeque(size_t& cur_start_idx, vector_3t& cmd_out) {
    std::lock_guard<std::mutex> lock(traj_buffer_mutex_);
    
    // 1. 基础判空
    if (control_traj_buffer_.empty()){
        cmd_out.setZero();
        LOG(ERROR) << "Control deque is empty!";
        return false;
    } 

    // === 错误检查 A: 所有点都比 cur 小  ===
    // if (cur_start_idx < control_traj_buffer_.front().idx) {
    //     LOG(ERROR) << "Buffer Underrun! Request idx: " << cur_start_idx 
    //                << ", Min Buffer idx: " << control_traj_buffer_.front().idx;
    //     cmd_out.setZero();
    //     return false;
    // }

    // // === 错误检查 B: Buffer 里的点索引都很大 ===
    // if (cur_start_idx > control_traj_buffer_.back().idx) {
    //     LOG(ERROR) << "Buffer Overrun! Request idx: " << cur_start_idx 
    //                << ", Max Buffer idx: " << control_traj_buffer_.back().idx;
    //     cmd_out.setZero();
    //     return false;
    // }

    // 获取下一个点
    auto next_idx = cur_start_idx + 1;
    // 到达最后一段了
    // if(next_idx == target_traj_size_ - 1){
    if(next_idx == mid_mpc_end_idx_){
        // cmd_out.setZero();
        ctrl_sampling_finished_ = true;
        return true;
    }

    // 查找第一个 idx >= cur_start_idx 的位置
    auto it_curr = std::lower_bound(control_traj_buffer_.begin(), control_traj_buffer_.end(), cur_start_idx,
        [](const StampedControl& c, size_t val) {
            return (size_t)c.idx < val; 
        });
    bool found_curr = (it_curr != control_traj_buffer_.end() && (size_t)it_curr->idx == cur_start_idx);

    // not found
    if (!found_curr) {
        // 说明当前的索引不在控制轨迹中
        if (it_curr == control_traj_buffer_.begin()) {
            // 所有的index都大于当前索引
            cmd_out[0] = control_traj_buffer_.front().vx;
            cmd_out[1] = control_traj_buffer_.front().vy;
            cmd_out[2] = control_traj_buffer_.front().omega;  
        } else {
            cmd_out[0] = control_traj_buffer_.back().vx;
            cmd_out[1] = control_traj_buffer_.back().vy;
            cmd_out[2] = control_traj_buffer_.back().omega;  
        }
        return true;
    }

    auto it_next = it_curr + 1;
    bool found_next = (it_next != control_traj_buffer_.end() && (size_t)it_next->idx == next_idx);
    if (!found_next) { 
        cmd_out[0] = it_curr->vx;
        cmd_out[1] = it_curr->vy;
        cmd_out[2] = it_curr->omega;
        return true;
    }

    double s = 0;
    if (next_idx < target_chassis_traj_.size()) {

        last_start_idx_ = cur_start_idx;

        auto p_curr = target_chassis_traj_[cur_start_idx].pose;
        auto p_next = target_chassis_traj_[next_idx].pose;

        vector_3t leftFramePt (p_curr.position.x(), p_curr.position.y(), quaternionToEulerZYX(p_curr.orientation)[0]);
        vector_3t rightFramePt(p_next.position.x(), p_next.position.y(), quaternionToEulerZYX(p_next.orientation)[0]);

        s = projectionRatio(current_pose_, leftFramePt, rightFramePt);
        // 这里得给目标值
        s += control_step_dt_ / traj_step_dt_;
        s = std::clamp(s, 0.0, 1.0);

    }
    cmd_out[0] = lerp(it_curr->vx,    it_next->vx,    s);
    cmd_out[1] = lerp(it_curr->vy,    it_next->vy,    s);
    cmd_out[2] = lerp(it_curr->omega, it_next->omega, s);

    return true;
}

int GalbotS1ChassisPoseNMPCController::findRotationCutoffIndex() {
    if (target_chassis_traj_.size() < 2) return 0;

    double start_x = target_chassis_traj_[0].pose.position.x();
    double start_y = target_chassis_traj_[0].pose.position.y();
    
    // 从第1个点开始往后搜
    for (size_t i = 1; i < target_chassis_traj_.size(); ++i) {
        double dx = target_chassis_traj_[i].pose.position.x() - start_x;
        double dy = target_chassis_traj_[i].pose.position.y() - start_y;
        double dist = std::hypot(dx, dy);

        // 一旦位置移动超过阈值，说明开始走了，前一个点就是旋转结束点
        if (dist > static_pos_threshold_) {
            return (i > 0) ? (i - 1) : 0;
        }
    }
    // 如果整条轨迹都没移动位置（纯旋转任务），则返回最后一点
    return target_chassis_traj_.size() - 1;
}

bool GalbotS1ChassisPoseNMPCController::checkWheelsAligned() {
    std::vector<double> target_angles, target_speeds;
    vector_3t fake_rot_cmd(0.0, 0.0, 1.0);
    inverseKinematicsForSpinMode(fake_rot_cmd, target_angles, target_speeds); 

    bool all_aligned = true;
    for (int i = 0; i < wheel_num_; ++i) {
        double diff = std::abs(normalizeAngle(current_steering_angles_[i] - target_angles[i]));
        if (diff > M_PI_2) diff = std::abs(diff - M_PI);
        
        if (diff > align_angle_threshold_) {
            all_aligned = false;
            break;
        }
    }
    return all_aligned;
}
void GalbotS1ChassisPoseNMPCController::inverseKinematicsForCrabMode(vector_3t &target_vel, std::vector<double> &output_steer_angles, 
                                                                    std::vector<double> &output_wheel_vels)
{
    // === 1. 输入分解 ===
    double vx = target_vel[0];
    double vy = target_vel[1];
    double wz = target_vel[2];

    output_steer_angles.resize(wheel_num_);
    output_wheel_vels.resize(wheel_num_);

    // === 工程参数 ===
    const double speed_lock_thresh = 0.03;    // 低速锁角阈值

    for (int i = 0; i < wheel_num_; ++i) {
        const vector_2t &ri = wheel_pos_[i];

        // --- 计算该舵轮的目标速度向量 ---
        double vwx = vx - wz * ri.y();
        double vwy = vy + wz * ri.x();

        double v_mag = std::sqrt(vwx * vwx + vwy * vwy);

        // 当前舵角
        double current_theta = normalizeAngle(current_steering_angles_[i]);

        // === 2. 速度太小 → 锁角，不更新 ===
        if (v_mag < speed_lock_thresh) {
            output_steer_angles[i] = current_theta;
            output_wheel_vels[i] = 0.0;
            continue;
        }

        // === 3. atan2 求目标角度 ===
        double target_theta = std::atan2(vwy, vwx);
        target_theta = normalizeAngle(target_theta);

        // === 4. 计算 delta（范围 -pi ~ pi） ===
        double delta = normalizeAngle(target_theta - current_theta);

        // === 5. 若超过 90°，舵轮反转，避免跳变 180° ===
        if (std::fabs(delta) > M_PI / 2.0) {
            if (delta > 0)
                target_theta -= M_PI;
            else
                target_theta += M_PI;

            v_mag = -v_mag;   // 轮速反向
            target_theta = normalizeAngle(target_theta);
            delta = normalizeAngle(target_theta - current_theta);
        }

        // === 6. 限制舵角范围 ===
        target_theta = std::clamp(target_theta, -max_steer_angle_, max_steer_angle_);

        // === 7. 输出角度与轮速 ===
        output_steer_angles[i] = target_theta;
        output_wheel_vels[i] = v_mag / wheel_radius_;
    }
}

void GalbotS1ChassisPoseNMPCController::updateErrors(vector_3t& err) 
{
    sliding_window_.emplace_back(err);
    if (sliding_window_.size() > window_size_) {
        sliding_window_.pop_front();
    }
}

bool GalbotS1ChassisPoseNMPCController::isWindowValid()
{
    // 如果窗口还没填满，先不结束
    if (sliding_window_.size() < window_size_) return false;

    for (const auto& error: sliding_window_) {
        if (std::abs(error.x()) > pos_threshold_[0] ||
            std::abs(error.y()) > pos_threshold_[1]) 
            // ||std::abs(error.z()) > pos_threshold_[2])
        return false;
    }
    return true;
}

double GalbotS1ChassisPoseNMPCController::findNearestAngle(double angle, double ref) {
    double diff = angle - ref;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return ref + diff;
}

void GalbotS1ChassisPoseNMPCController::limitAcceleration(vector_3t& velocity_command, 
                                                         const vector_3t& last_velocity_command,
                                                         stdvec_scalar_t max_acc, double dt) {
    double max_ratio_xy = 1.0;
    for (int i = 0; i < 2; i++) { 
        double dv = velocity_command[i] - last_velocity_command[i];
        double max_dv = max_acc[i] * dt;
        if (max_dv > 1e-6) {
            max_ratio_xy = std::max(max_ratio_xy, std::abs(dv) / max_dv);
        }
    }
    for (int i = 0; i < 2; i++) {
        double dv = velocity_command[i] - last_velocity_command[i];
        velocity_command[i] = last_velocity_command[i] + (dv / max_ratio_xy);
    }

    double dv_z = velocity_command[2] - last_velocity_command[2];
    double max_dv_z = max_acc[2] * dt;
    if (std::abs(dv_z) > max_dv_z) {
        velocity_command[2] = last_velocity_command[2] + (dv_z > 0 ? max_dv_z : -max_dv_z);
    }
}
void GalbotS1ChassisPoseNMPCController::parseTaskInfo(const std::string& task_name)
{ 
    json data = json::parse(task_name);

    // 安全性检查
    if(data.contains("segments") && data["segments"].size() >= 3) {
        std::vector<int> segments = data["segments"].get<std::vector<int>>();
        first_rotation_end_idx_ = segments[0];
        mid_mpc_end_idx_ = segments[1];
        last_rotation_end_idx_ = segments[2];

        LOG(INFO) << "task_id: " << task_id_ << " first_rotation_end_idx: " << first_rotation_end_idx_ 
        << " mid_mpc_end_idx:: " << mid_mpc_end_idx_ << " last_rotation_start_idx: " << last_rotation_end_idx_;
    }
}