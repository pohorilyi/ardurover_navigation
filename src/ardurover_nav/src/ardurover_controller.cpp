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
    // TwistStamped + best-effort: cmd_vel is TwistStamped; Twist never matches MAVROS.
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
    // GUIDED requested before GPS/EKF is ready gets dropped to MANUAL; retry from state.
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

constexpr double kLookaheadM = 1.4;     // default carrot; 2 m cut corners on path 1/2
constexpr double kLookaheadMinM = 0.6;  // floor in a sharp turn
constexpr double kTurnHeadingRad = 0.7; // |ψe| at which look-ahead/speed are fully tightened
constexpr double kMinTurnSpeed = 0.35;  // fraction of cruise when |ψe| is large
constexpr double kCruiseSpeed = 0.8;
constexpr double kHeadingP = 1.0;     // flip sign if BODY_NED steers the wrong way
constexpr double kMaxYawRate = 0.6;   // stop spin-outs when heading error is large
constexpr double kFinishRemainM = 0.1;  // leftover polyline; 1 m fires on path 0/1 reverse tails
constexpr double kReverseHeadingRad = 2.0;  // carrot behind (~115 deg) → reverse, don't yaw 180
constexpr double kMaxIndexAdvanceM = 3.0;  // don't skip to an overlapping later stretch
constexpr double kSlowdownM = 4.0;    // bleed speed into the finish
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double distance(const Waypoint &a, const Waypoint &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Nearest sample at or after `from`, but only within the next few metres of
// polyline. Searching the whole suffix snaps forward onto a reverse tail that
// occupies the same XY as the outbound (path 0 / 180-rollback).
std::size_t closest_index_from(const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t from) {
    std::size_t best = from;
    double best_distance = distance(pose, path[from]);
    double traveled = 0.0;
    for (std::size_t i = from + 1; i < path.size(); ++i) {
        traveled += distance(path[i - 1], path[i]);
        if (traveled > kMaxIndexAdvanceM) {
            break;
        }
        const double candidate_distance = distance(pose, path[i]);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            best = i;
        }
    }
    return best;
}

// Aim this far along the polyline, not at the finish and not at the closest
// sample (closest is beside you → heading error ~0 while you drift).
// Stop at a cusp (segment reverse) so a 1.4 m carrot does not sit on path 0's
// ~1 m rollback while still outbound — that starts REV early and looks like a stop.
Waypoint lookahead_waypoint(const std::vector<Waypoint> &path, std::size_t from, double lookahead_m) {
    double traveled = 0.0;
    std::size_t i = from;
    double prev_dx = 0.0;
    double prev_dy = 0.0;
    bool have_prev = false;
    while (i + 1 < path.size() && traveled < lookahead_m) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        if (have_prev && (prev_dx * dx + prev_dy * dy) < 0.0) {
            break;
        }
        traveled += std::hypot(dx, dy);
        prev_dx = dx;
        prev_dy = dy;
        have_prev = true;
        ++i;
    }
    return path[i];
}

// Leftover polyline after closest_i. Crow-flies to path.back() lies when the
// recording rolls back at the finish (path 0).
double remaining_path_length(const std::vector<Waypoint> &path, std::size_t from) {
    double length = 0.0;
    for (std::size_t i = from; i + 1 < path.size(); ++i) {
        length += distance(path[i], path[i + 1]);
    }
    return length;
}

// Signed distance to the path (m). Heading-only can stay parallel and offset;
// this is that sideways error (log / later CTE term). Positive = left of segment.
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

    progressIndex_ = closest_index_from(current_waypoint, path_, progressIndex_);
    const std::size_t closest_i = progressIndex_;

    // First carrot at the default look-ahead, then shorten if we are already
    // aimed across a bend (long carrot + speed = cut the vertex).
    auto heading_toward = [&](const Waypoint &to) {
        return wrap(std::atan2(to.y - current_waypoint.y, to.x - current_waypoint.x) - current_waypoint.yaw);
    };
    Waypoint target = lookahead_waypoint(path_, closest_i, kLookaheadM);
    double heading_error = heading_toward(target);
    // Path 0/1 recordings reverse along the same heading. |ψe|~π would otherwise spin.
    const bool reverse = std::abs(heading_error) > kReverseHeadingRad;
    double turn = 0.0;
    if (reverse) {
        heading_error = wrap(heading_error - std::copysign(3.14159265358979323846, heading_error));
    } else {
        turn = std::clamp(std::abs(heading_error) / kTurnHeadingRad, 0.0, 1.0);
        const double lookahead_m = kLookaheadM + (kLookaheadMinM - kLookaheadM) * turn;
        if (turn > 0.0) {
            target = lookahead_waypoint(path_, closest_i, lookahead_m);
            heading_error = heading_toward(target);
        }
    }
    const double desired_heading = std::atan2(target.y - current_waypoint.y, target.x - current_waypoint.x);
    const double heading_error_deg = heading_error * kRadToDeg;
    const double dist_target = distance(current_waypoint, target);
    const double dist_goal = distance(current_waypoint, path_.back());
    const double remaining = remaining_path_length(path_, closest_i);
    const double approach = remaining;
    const double cte = signed_cross_track(current_waypoint, path_, closest_i);

    // Body command only. Firmware mixes wheels. Zero Twist = request stop.
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.now();
    cmd.header.frame_id = "base_link";

    // Do not use dist_goal or a 1 m remaining bubble: path.back() lies on the
    // outbound, and the reverse tail is itself ~1 m, so both latch before the apex.
    if (!finished_ && (closest_i + 1 >= path_.size() || remaining <= kFinishRemainM)) {
        finished_ = true;
    }
    const bool at_goal = finished_;
    if (!at_goal) {
        const double end_scale =
            reverse ? 1.0 : std::clamp(approach / kSlowdownM, 0.0, 1.0);
        const double turn_scale = 1.0 - (1.0 - kMinTurnSpeed) * turn;
        const double vx_sign = reverse ? -1.0 : 1.0;
        cmd.twist.linear.x = vx_sign * kCruiseSpeed * end_scale * turn_scale;
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
            RCLCPP_INFO(
                node_.get_logger(), "At goal (i=%zu dist=%.2f m remaining=%.2f m), stopping", closest_i, dist_goal,
                remaining
            );
            loggedGoal_ = true;
        }
        return;
    }

    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 500,
        "i=%zu cte=%.3f m  he=%.1f deg  cmd vx=%.2f wz=%.2f%s%s  act vx=%.2f wz=%.2f  d_la=%.2f remain=%.2f", closest_i,
        cte, heading_error_deg, cmd.twist.linear.x, cmd.twist.angular.z, reverse ? " REV" : "", wz_sat ? " SAT" : "",
        odometry.twist.twist.linear.x, odometry.twist.twist.angular.z, dist_target, remaining
    );
}

}  // namespace ardurover_nav
