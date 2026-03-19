#include "stt-for-diff-drive/turtlebot_node.hpp"

TurtleBotController::TurtleBotController() : Node("turtlebot_controller")
{
    this->declare_parameter("linear_vel",  0.0);
    this->declare_parameter("angular_vel", 0.0);

    linear_vel_  = this->get_parameter("linear_vel").as_double();
    angular_vel_ = this->get_parameter("angular_vel").as_double();

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, bind(&TurtleBotController::odomCallback, this, placeholders::_1));

    timer_ = this->create_wall_timer(chrono::milliseconds(100), bind(&TurtleBotController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Controller initiated — v: %.3f m/s | ω: %.3f rad/s", linear_vel_, angular_vel_);

    tubes_ = loadTubeCoefficients("/home/focaslab/ros2_ws/src/stt-for-diff-drive/coefficients.csv");
    start_time_ = this->now();
}

void TurtleBotController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_x = msg->pose.pose.position.x;
    current_pose_y = msg->pose.pose.position.y;
    current_theta = getYaw(msg->pose.pose.orientation);

    current_vel_x = msg->twist.twist.linear.x;
    current_vel_y = msg->twist.twist.linear.y;
    current_omega = msg->twist.twist.angular.z;
}

vector<Tube> TurtleBotController::loadTubeCoefficients(const string &filename)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Could not open tube file: [" << filename << "]\n";
        return {};
    }

    vector<Tube> tubes;
    string line;

    while (getline(file, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (line.empty())
            continue;

        if (line.find("Tube Coefficients") != string::npos)
        {
            tubes.emplace_back();
            continue;
        }

        if (tubes.empty())
            continue;

        size_t delim_pos = line.find("\",");
        if (delim_pos == string::npos)
            continue;

        size_t value_start = delim_pos + 2;
        size_t next_comma  = line.find(',', value_start);

        string value_str = (next_comma != string::npos)
            ? line.substr(value_start, next_comma - value_start)
            : line.substr(value_start);

        if (value_str.empty())
            continue;

        try
        {
            tubes.back().coefficients.push_back(stod(value_str));
        }
        catch (const invalid_argument &)
        {
            // Non-numeric value — skip 
        }
    }

    for (size_t i = 0; i < tubes.size(); ++i)
    {
        if (i < TUBE_TIMESTAMPS.size())
        {
            tubes[i].start_time = TUBE_TIMESTAMPS[i].first;
            tubes[i].end_time   = TUBE_TIMESTAMPS[i].second;
        }
        else
        {
            tubes[i].start_time = 0.0;
            tubes[i].end_time   = 0.0;
            cerr << "Warning: No timestamp entry for tube index " << i << "\n";
        }
    }

    return tubes;
}

vector<double> TurtleBotController::realGammas(double t, const vector<double> &C, int degree)
{
    vector<double> result(dim, 0.0);

    for (int i = 0; i < dim; ++i)
    {
        double power = 1.0;
        for (int j = 0; j <= degree; ++j)
        {
            result[i] += C[j + i * (degree + 1)] * power;
            power *= t;
        }
    }
    return result;
}

vector<Waypoint> TurtleBotController::getTubeWaypoints(const vector<Tube> &tubes, double dt)
{
    vector<Waypoint> waypoints;

    for (size_t tube_idx = 0; tube_idx < tubes.size(); ++tube_idx)
    {
        const auto &tube   = tubes[tube_idx];
        const auto &coeffs = tube.coefficients;

        int num_coeffs = static_cast<int>(coeffs.size());
        if (num_coeffs % dim != 0)
        {
            cerr << "Error: Tube " << tube_idx
                      << " coefficients (" << num_coeffs
                      << ") not divisible by dim (" << dim << "). Skipping.\n";
            continue;
        }

        int degree = (num_coeffs / dim) - 1;
        cout << "Tube " << tube_idx
                  << ": degree=" << degree
                  << "  t=[" << tube.start_time << ", " << tube.end_time << "]\n";

        for (double t = tube.start_time; t <= tube.end_time + 1e-9; t += dt)
        {
            Waypoint wp;
            wp.t        = t;
            wp.position = realGammas(t, coeffs, degree);
            waypoints.push_back(wp);
        }
    }

    return waypoints;
}

void TurtleBotController::controlLoop()
{
    double t = (this->now() - start_time_).seconds();

    const Tube* active_tube = nullptr;
    for (const auto &tube : tubes_)
    {
        if (t >= tube.start_time && t <= tube.end_time)
        {
            active_tube = &tube;
            break;
        }
    }

    if (!active_tube)
    {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        return;
    }

    int degree = (static_cast<int>(active_tube->coefficients.size()) / dim) - 1;

    auto des = realGammas(t, active_tube->coefficients, degree);

    double ex = des[0] - current_pose_x;
    double ey = des[1] - current_pose_y;

    RCLCPP_INFO(this->get_logger(), "\n\nCurrent State — x: %.3f m | y: %.3f m | θ: %.3f rad \nDesired State — x: %.3f m | y: %.3f m | θ: %.3f rad", 
                                        current_pose_x, current_pose_y, current_theta, des[0], des[1], atan2(des[1], des[0]));

    linear_vel_  = sqrt(ex*ex + ey*ey);
    angular_vel_ = atan2(ey, ex) - current_theta;

    cmd.linear.x  = linear_vel_;
    cmd.angular.z = angular_vel_;
    vel_pub_->publish(cmd);
}
