#include "ardurover_nav/ardurover_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <string>

namespace ardurover_nav {

ArduroverController::ArduroverController(rclcpp::Node &node, std::vector<Waypoint> path)
    : node_(node), path_(std::move(path)) {
    arming_ = node_.create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    setMode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    paramClient_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, "/mavros/setpoint_velocity");
    stateSub_ = node_.create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", rclcpp::QoS(10).best_effort(),
        [this](const mavros_msgs::msg::State &msg) { OnState(msg); }
    );
    velocityPublisher_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", rclcpp::QoS(10).best_effort()
    );

    std::string defaultLog = "paths/control.csv";
    if (const char *root = std::getenv("ARDUROVER_NAV_ROOT")) {
        defaultLog = std::string(root) + "/paths/control.csv";
    }
    const auto logPath = node_.declare_parameter("control_log_file", defaultLog);
    logFile_.open(logPath, std::ios::out | std::ios::trunc);
    if (!logFile_) {
        RCLCPP_WARN(node_.get_logger(), "Could not open control log %s", logPath.c_str());
    } else {
        logFile_ << "t,x,y,yaw,act_vx,act_wz,closest_i,cte,target_x,target_y,dist_target,dist_goal,"
                    "desired_yaw,heading_err,heading_err_deg,cmd_vx,cmd_wz,wz_sat,at_goal\n";
        RCLCPP_INFO(node_.get_logger(), "Control log: %s", logPath.c_str());
    }
}

void ArduroverController::OnState(const mavros_msgs::msg::State &msg) {
    state_ = msg;
}

void ArduroverController::RequestGuided() {
    if (modeFuture_.valid() && modeFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "GUIDED";
    modeFuture_ = setMode_->async_send_request(req).future.share();
}

void ArduroverController::RequestArm() {
    if (armFuture_.valid() && armFuture_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    armFuture_ = arming_->async_send_request(req).future.share();
}

bool ArduroverController::SetupArdurover() {
    if (setupState_ == SetupState::Ready && (!state_.armed || state_.mode != "GUIDED")) {
        RCLCPP_WARN(
            node_.get_logger(), "Left GUIDED/armed (mode=%s armed=%d), retrying", state_.mode.c_str(), state_.armed
        );
        setupState_ = SetupState::SetMode;
        modeFuture_ = {};
        armFuture_ = {};
    }

    switch (setupState_) {
        case SetupState::WaitServices:
            if (arming_->service_is_ready() && setMode_->service_is_ready() && state_.connected) {
                setupState_ = SetupState::SetFrame;
            }
            return false;
        case SetupState::SetFrame:
            if (!paramClient_->service_is_ready()) {
                return false;
            }
            paramClient_->set_parameters({rclcpp::Parameter("mav_frame", "BODY_NED")});
            setupState_ = SetupState::Prime;
            return false;
        case SetupState::Prime:
            if (++primeTicks_ >= 20) {
                setupState_ = SetupState::SetMode;
            }
            return false;
        case SetupState::SetMode:
            if (state_.mode == "GUIDED") {
                setupState_ = SetupState::Arm;
                return false;
            }
            RequestGuided();
            return false;
        case SetupState::Arm:
            if (state_.armed) {
                setupState_ = SetupState::Ready;
                RCLCPP_INFO(node_.get_logger(), "Armed and GUIDED — controller running");
                return true;
            }
            RequestArm();
            return false;
        case SetupState::Ready:
            return true;
    }
    return false;
}

constexpr double kLookaheadM = 2.0;
constexpr double kCruiseSpeed = 0.8;
constexpr double kHeadingP = 1.0;
constexpr double kMaxYawRate = 0.6;
constexpr double kGoalRadiusM = 1.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double distance(const Waypoint &a, const Waypoint &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

std::size_t closest_index(const Waypoint &pose, const std::vector<Waypoint> &path) {
    std::size_t best = 0;
    double best_distance = distance(pose, path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double candidate_distance = distance(pose, path[i]);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            best = i;
        }
    }
    return best;
}

// Walk forward from closest until ~lookahead_m along the polyline (or the last point).
Waypoint lookahead_waypoint(const std::vector<Waypoint> &path, std::size_t from, double lookahead_m) {
    double traveled = 0.0;
    std::size_t i = from;
    while (i + 1 < path.size() && traveled < lookahead_m) {
        traveled += distance(path[i], path[i + 1]);
        ++i;
    }
    return path[i];
}

// Positive = left of the path segment through closest_i.
double signed_cross_track(const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t closest_i) {
    if (path.size() < 2) {
        return 0.0;
    }
    const std::size_t i0 = (closest_i + 1 < path.size()) ? closest_i : closest_i - 1;
    const Waypoint &a = path[i0];
    const Waypoint &b = path[i0 + 1];
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len = std::hypot(abx, aby);
    if (len < 1e-12) {
        return 0.0;
    }
    return (abx * (pose.y - a.y) - aby * (pose.x - a.x)) / len;
}

void ArduroverController::Control(const nav_msgs::msg::Odometry &odometry) {
    Waypoint current_waypoint{
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        yaw_from_quat(odometry.pose.pose.orientation)
    };

    const std::size_t closest_i = closest_index(current_waypoint, path_);
    const Waypoint target = lookahead_waypoint(path_, closest_i, kLookaheadM);
    const double dist_target = distance(current_waypoint, target);
    const double dist_goal = distance(current_waypoint, path_.back());
    const double cte = signed_cross_track(current_waypoint, path_, closest_i);

    // Heading toward the look-ahead point, not path_[i].yaw.
    const double desired_heading = std::atan2(target.y - current_waypoint.y, target.x - current_waypoint.x);
    const double heading_error = wrap(desired_heading - current_waypoint.yaw);
    const double heading_error_deg = heading_error * kRadToDeg;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.now();
    cmd.header.frame_id = "base_link";

    const bool at_goal = dist_goal <= kGoalRadiusM;
    if (!at_goal) {
        cmd.twist.linear.x = kCruiseSpeed;
        cmd.twist.angular.z = std::clamp(kHeadingP * heading_error, -kMaxYawRate, kMaxYawRate);
    }
    const bool wz_sat = !at_goal && std::abs(kHeadingP * heading_error) > kMaxYawRate + 1e-9;

    velocityPublisher_->publish(cmd);

    if (logFile_.is_open()) {
        logFile_ << std::fixed << std::setprecision(4) << node_.now().seconds() << ',' << current_waypoint.x << ','
                 << current_waypoint.y << ',' << current_waypoint.yaw << ',' << odometry.twist.twist.linear.x << ','
                 << odometry.twist.twist.angular.z << ',' << closest_i << ',' << cte << ',' << target.x << ',' << target.y
                 << ',' << dist_target << ',' << dist_goal << ',' << desired_heading << ',' << heading_error << ','
                 << heading_error_deg << ',' << cmd.twist.linear.x << ',' << cmd.twist.angular.z << ',' << wz_sat << ','
                 << at_goal << '\n';
        logFile_.flush();
    }

    if (at_goal) {
        if (!loggedGoal_) {
            RCLCPP_INFO(node_.get_logger(), "At goal (%.2f m), stopping", dist_goal);
            loggedGoal_ = true;
        }
        return;
    }
    loggedGoal_ = false;

    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 500,
        "i=%zu cte=%.3f m  he=%.1f deg  cmd vx=%.2f wz=%.2f%s  act vx=%.2f wz=%.2f  d_la=%.2f d_goal=%.2f", closest_i,
        cte, heading_error_deg, cmd.twist.linear.x, cmd.twist.angular.z, wz_sat ? " SAT" : "",
        odometry.twist.twist.linear.x, odometry.twist.twist.angular.z, dist_target, dist_goal
    );
}

}  // namespace ardurover_nav
