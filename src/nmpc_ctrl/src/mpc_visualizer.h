#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mpc_solver.h"
#include "reference_generator.h"
#include "steering_ik.h"

class MpcVisualizer {
public:
    MpcVisualizer(ros::NodeHandle &nh, const MpcParams &p);

    void publish(const std::vector<double>& state, MPCSolver &solver,
                 const ReferenceGenerator &ref_gen,
                const WheelCmd wheels[NUM_WHEELS]);

private:
    ros::Publisher pub_ref_, pub_pred_, pub_act_, pub_mk_, pub_wheels_;
    nav_msgs::Path actual_;
    MpcParams p_;

    geometry_msgs::PoseStamped make_pose(double x, double y, double th, ros::Time stamp);
};