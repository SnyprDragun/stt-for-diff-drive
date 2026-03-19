#include "stt-for-diff-drive/turtlebot_node.hpp"

TurtleBotController::TurtleBotController() : Node("turtlebot_controller")
{
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, bind(&TurtleBotController::odomCallback, this, placeholders::_1));

    timer_ = this->create_wall_timer(chrono::milliseconds(100), bind(&TurtleBotController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Controller Initiated!");

    string pkg_path = ament_index_cpp::get_package_share_directory("stt-for-diff-drive");
    filesystem::path file_path = pkg_path + "/config/coefficients.csv";
    cout << file_path << endl;
    tubes_ = loadTubeCoefficients(file_path);

    initLogger();

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

void TurtleBotController::initLogger()
{
    string log_path = string(PACKAGE_SOURCE_DIR) + "/config/trajectory.csv";
    filesystem::create_directories(string(PACKAGE_SOURCE_DIR) + "/config/");

    traj_log_.open(log_path, ios::out | ios::trunc);
    if (!traj_log_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open log file: %s", log_path.c_str());
        return;
    }
    traj_log_ << "t,des_x,des_y,act_x,act_y,v,omega\n";
    RCLCPP_INFO(this->get_logger(), "Logging to: %s", log_path.c_str());
}

void TurtleBotController::logState(double t, double des_x, double des_y, double act_x, double act_y, double v, double omega)
{
    if (traj_log_.is_open())
        traj_log_ << fixed << setprecision(6)
                  << t      << "," << des_x << "," << des_y << ","
                  << act_x  << "," << act_y << ","
                  << v      << "," << omega << "\n";
}

vector<double> TurtleBotController::trajectory(double t, const vector<double> &C, int degree)
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

void TurtleBotController::controlLoop()
{
    double linear_vel_  = 0.0;
    double angular_vel_ = 0.0;

    geometry_msgs::msg::Twist cmd;
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
        // ── All tubes finished — stop robot, flush log, shutdown ─────────────
        vel_pub_->publish(geometry_msgs::msg::Twist());   // zero velocity

        if (traj_log_.is_open())
        {
            traj_log_.flush();
            traj_log_.close();
            RCLCPP_INFO(this->get_logger(), "Trajectory log closed.");
        }

        timer_->cancel();   // stop the control loop timer
        RCLCPP_INFO(this->get_logger(), "All tubes complete. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    int degree = (static_cast<int>(active_tube->coefficients.size()) / dim) - 1;

    auto des = trajectory(t, active_tube->coefficients, degree);

    // double ex = des[0] - current_pose_x;
    // double ey = des[1] - current_pose_y;

    RCLCPP_INFO(this->get_logger(), "\n\nCurrent State — x: %.3f m | y: %.3f m | θ: %.3f rad \nDesired State — x: %.3f m | y: %.3f m | θ: %.3f rad", 
                                    current_pose_x, current_pose_y, current_theta, des[0], des[1], atan2(des[1], des[0]));

    // /*------------------------------- CONTROLLER - A -------------------------------*/
    // linear_vel_  = sqrt(ex*ex + ey*ey);
    // angular_vel_ = atan2(ey, ex) - current_theta;

    /*------------------------------- CONTROLLER - B -------------------------------*/
    const double MAX_V = 0.8;
    const double MAX_W = 1.5;
    const double k     = 0.5;

    // ── Error: actual - desired (matches paper convention e = γ_m⁻¹(2x - γ_s)) ──
    double ex = current_pose_x - des[0];
    double ey = current_pose_y - des[1];

    double normalized_error_x = clamp(ex / 1.0, -0.999, 0.999);
    double normalized_error_y = clamp(ey / 1.0, -0.999, 0.999);

    // ── Transformed error ε_i = ln((1 + e_i) / (1 - e_i)) ───────────────────────
    double eps_x = log((1.0 + normalized_error_x) / (1.0 - normalized_error_x));
    double eps_y = log((1.0 + normalized_error_y) / (1.0 - normalized_error_y));

    // ── ξ_ii = 4 / (1 - e_i²) ────────────────────────────────────────────────────
    double xi_x = 4.0 / (1.0 - normalized_error_x * normalized_error_x);
    double xi_y = 4.0 / (1.0 - normalized_error_y * normalized_error_y);

    // ── Cartesian control: u = -k * ξ * ε ────────────────────────────────────────
    double u_x = -k * xi_x * eps_x;
    double u_y = -k * xi_y * eps_y;

    // ── Map to differential drive ─────────────────────────────────────────────────
    double desired_heading = atan2(u_y, u_x);
    double heading_err     = atan2(sin(desired_heading - current_theta),
                                        cos(desired_heading - current_theta));

    double v     = sqrt(u_x*u_x + u_y*u_y) * cos(heading_err);
    double omega = heading_err;   // direct heading error as angular rate

    linear_vel_  = clamp(v,     -MAX_V, MAX_V);
    angular_vel_ = clamp(omega, -MAX_W, MAX_W);

    logState(t, des[0], des[1], current_pose_x, current_pose_y, linear_vel_, angular_vel_);

    cmd.linear.x  = linear_vel_;
    cmd.angular.z = angular_vel_;
    vel_pub_->publish(cmd);

}
