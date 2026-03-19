#include "mpc_visualizer.h"

MpcVisualizer::MpcVisualizer(ros::NodeHandle &nh, const MpcParams &p) : p_(p) {
    pub_ref_    = nh.advertise<nav_msgs::Path>("/mpc/ref_path", 1);
    pub_pred_   = nh.advertise<nav_msgs::Path>("/mpc/pred_path", 1);
    pub_act_    = nh.advertise<nav_msgs::Path>("/mpc/robot_path", 1);
    pub_mk_     = nh.advertise<visualization_msgs::Marker>("/mpc/robot_marker", 1);
    pub_wheels_ = nh.advertise<visualization_msgs::MarkerArray>("/mpc/wheel_markers", 1);
}

void MpcVisualizer::publish(const std::vector<double>& state, MPCSolver &solver,
                            const ReferenceGenerator &ref_gen,
							const WheelCmd wheels[NUM_WHEELS]) {
    ros::Time now = ros::Time::now();

    // 参考轨迹
    {
        auto pts = ref_gen.full_trajectory(200);
        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now;
        for (auto &p : pts)
            msg.poses.push_back(make_pose(p.val[0], p.val[1], p.val[2], now));
        pub_ref_.publish(msg);
    }

    // 预测轨迹
    {
        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now;
        for (int i = 0; i <= solver.N(); i++) {
            std::vector<double> xi(p_.solver_cfg.nx);
            solver.get_x(i, xi.data());
            msg.poses.push_back(make_pose(xi[0], xi[1], xi[2], now));
        }
        pub_pred_.publish(msg);
    }

    // 实际路径
    {
        actual_.header.frame_id = "map";
        actual_.header.stamp = now;
        actual_.poses.push_back(make_pose(state[0], state[1], state[2], now));
        if (actual_.poses.size() > 2000)
            actual_.poses.erase(actual_.poses.begin());
        pub_act_.publish(actual_);
    }

    // 机器人箭头
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = now;
        mk.ns = "robot"; mk.id = 0;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = state[0];
        mk.pose.position.y = state[1];
        mk.pose.position.z = 0.05;
        tf2::Quaternion q; q.setRPY(0, 0, state[2]);
        mk.pose.orientation.x = q.x();
        mk.pose.orientation.y = q.y();
        mk.pose.orientation.z = q.z();
        mk.pose.orientation.w = q.w();
        mk.scale.x = 0.5; mk.scale.y = 0.12; mk.scale.z = 0.12;
        mk.color.r = 1.0; mk.color.g = 0.2; mk.color.b = 0.2; mk.color.a = 1.0;
        mk.lifetime = ros::Duration(0.2);
        pub_mk_.publish(mk);
    }
    // 舵轮方向箭头
    {
        double yaw = state[2], c = cos(yaw), s = sin(yaw);
        visualization_msgs::MarkerArray arr;
        const char* colors[][3] = {{"1","0.5","0"},{"0","1","0"},{"0","0.5","1"},{"1","1","0"}};
        for (int i = 0; i < NUM_WHEELS; i++) {
            double lx = p_.wheel_lx[i], ly = p_.wheel_ly[i];
            // 轮子全局位置
            double wx = state[0] + c * lx - s * ly;
            double wy = state[1] + s * lx + c * ly;
            // 舵角 = body yaw + steer_angle
            double abs_angle = yaw + wheels[i].steer_angle;

            visualization_msgs::Marker mk;
            mk.header.frame_id = "map"; mk.header.stamp = now;
            mk.ns = "wheels"; mk.id = i;
            mk.type = visualization_msgs::Marker::ARROW;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position.x = wx; mk.pose.position.y = wy; mk.pose.position.z = 0.1;
            tf2::Quaternion q2; q2.setRPY(0, 0, abs_angle);
            mk.pose.orientation.x = q2.x(); mk.pose.orientation.y = q2.y();
            mk.pose.orientation.z = q2.z(); mk.pose.orientation.w = q2.w();
            double len = std::min(std::fabs(wheels[i].drive_vel) * 0.5, 0.3);
            mk.scale.x = std::max(len, 0.05); mk.scale.y = 0.04; mk.scale.z = 0.04;
            mk.color.r = atof(colors[i][0]);
            mk.color.g = atof(colors[i][1]);
            mk.color.b = atof(colors[i][2]);
            mk.color.a = 1.0;
            mk.lifetime = ros::Duration(0.2);
            arr.markers.push_back(mk);
        }
        pub_wheels_.publish(arr);
    }
}

geometry_msgs::PoseStamped MpcVisualizer::make_pose(double x, double y, double th, ros::Time stamp) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = stamp;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0;
    tf2::Quaternion q; q.setRPY(0, 0, th);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    return ps;
}