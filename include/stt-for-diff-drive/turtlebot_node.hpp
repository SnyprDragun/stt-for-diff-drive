#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tube.hpp"

using namespace std;
using namespace tf2;
using namespace rclcpp;

static const vector<pair<double, double>> TUBE_TIMESTAMPS = {
    {0.0, 15.0},   // tube 0
    {15.0, 30.0},  // tube 1
    {30.0, 45.0},  // tube 2
    {45.0, 60.0}   // tube 3
};

class TurtleBotController : public Node
{
public:
    TurtleBotController();

private:
    double current_pose_x = 0.0, current_pose_y = 0.0, current_theta = 0.0;
    double current_vel_x = 0.0, current_vel_y = 0.0, current_omega = 0.0;

    int dim = 2;
    vector<Tube> tubes_;
    Time start_time_;

    Publisher<visualization_msgs::msg::Marker>::SharedPtr tube_marker_pub_;
    Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;

    nav_msgs::msg::Path actual_path_;
    nav_msgs::msg::Path desired_path_;

    Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    TimerBase::SharedPtr timer_;
    TimerBase::SharedPtr marker_timer_;
    ofstream traj_log_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    vector<Tube> loadTubeCoefficients(const string &filename);
    void initLogger();
    void logState(double t, double des_x, double des_y, double act_x, double act_y, double v, double omega);
    void publishTubeMarkers();
    void updatePaths(double des_x, double des_y);
    vector<double> trajectory(double t, const vector<double> &C, int degree);
    void controlLoop();
};
