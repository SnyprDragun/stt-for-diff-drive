#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <tf2/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tube.hpp"
#include "setpoint.hpp"

using namespace std;
using namespace tf2;
using namespace rclcpp;

static const vector<pair<double, double>> TUBE_TIMESTAMPS = {
    {0.0, 11.0},   // tube 0
    {11.0, 20.0},  // tube 1
};

class TurtleBotController : public Node
{
public:
    TurtleBotController();

private:
    double current_pose_x = 0.0, current_pose_y = 0.0, current_theta = 0.0;
    double current_vel_x = 0.0, current_vel_y = 0.0, current_omega = 0.0;

    double linear_vel_  = 0.0;
    double angular_vel_ = 0.0;

    int dim = 2;
    vector<Tube> tubes_;
    Time start_time_;

    Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    TimerBase::SharedPtr timer_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    vector<Tube> loadTubeCoefficients(const string &filename);
    vector<double> trajectory(double t, const vector<double> &C, int degree);
    void controlLoop();
};
