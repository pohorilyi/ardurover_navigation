#include "ardurover_nav/ardurover_controller.hpp"
#include "ardurover_nav/path_follow.hpp"

#include <chrono>
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
        logFile_ << "t,x,y,yaw,roll,pitch,act_vx,act_wz,closest_i,cte,target_x,target_y,dist_target,dist_goal,"
                    "desired_yaw,heading_err,heading_err_deg,cmd_vx,cmd_wz,wz_sat,at_goal,reverse,unstuck\n";
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

void ArduroverController::LeaveUnstick() {
    const char *resume = preUnstickMode_ == DriveMode::Turn ? "TURN" : "FWD";
    RCLCPP_INFO(node_.get_logger(), "Unstick done, resume %s", resume);
    driveMode_ = preUnstickMode_;
    unstickPhase_ = UnstickPhase::Reverse;
    unstickTicks_ = 0;
    stuckTicks_ = 0;
    unstickAttempt_ = 0;
    unstickCooldownTicks_ = kUnstickCooldownTicks;
}

// Reset the unstick timer and pose/tilt anchors when Reverse or Drive starts.
void ArduroverController::BeginUnstickPhase(UnstickPhase phase, const Waypoint &pose, double tilt) {
    unstickPhase_ = phase;
    unstickTicks_ = 0;
    unstickStartX_ = pose.x;
    unstickStartY_ = pose.y;
    unstickLastTilt_ = tilt;
    unstickWiggleSign_ = 1.0;
}

// Yaw command while driving off a lip: PD on heading, mixed toward the path if CTE is large.
double ArduroverController::SteerOffLip(double heading_error, double act_wz, double cte) const {
    double wz = heading_yaw_cmd(heading_error, act_wz);
    if (std::abs(cte) > 0.15) {
        const double toward_path = -std::copysign(kMaxYawRate, cte);
        wz = 0.5 * wz + 0.5 * toward_path;
        wz = std::clamp(wz, -kMaxYawRate, kMaxYawRate);
    }
    return wz;
}

void ArduroverController::RecoverStuck(
    const Waypoint &pose,
    double act_vx,
    double act_wz,
    double heading_error,
    double cte,
    double roll,
    double pitch,
    bool at_goal,
    geometry_msgs::msg::TwistStamped &cmd
) {
    ++controlTicks_;
    if (std::hypot(act_vx, act_wz) > kSeenMotionVel) {
        seenMotion_ = true;
    }
    if (at_goal) {
        stuckTicks_ = 0;
        if (driveMode_ == DriveMode::Unstick) {
            LeaveUnstick();
        }
        return;
    }

    const double tilt = std::hypot(roll, pitch);

    if (driveMode_ == DriveMode::Unstick) {
        ++unstickTicks_;
        const double moved = std::hypot(pose.x - unstickStartX_, pose.y - unstickStartY_);
        const bool tilt_falling = tilt > kTiltRad && tilt + 0.015 < unstickLastTilt_;
        unstickLastTilt_ = tilt;
        const bool heading_clear = std::abs(heading_error) < kUnstickClearHeadingRad;

        if (unstickPhase_ == UnstickPhase::Reverse) {
            // One-sided wz (always toward the carrot) leaves a wheel on the lip.
            // Alternate full yaw so left then right wheels take the reverse load.
            if (unstickTicks_ > 1 && (unstickTicks_ - 1) % kUnstickWiggleTicks == 0) {
                unstickWiggleSign_ = -unstickWiggleSign_;
            }
            cmd.twist.linear.x = -kUnstickReverseSpeed;
            cmd.twist.angular.z = unstickWiggleSign_ * kMaxYawRate;
            const bool wiggling = unstickTicks_ < kUnstickReverseTicks;
            const bool unhooked = !wiggling && moved >= kUnstickReverseM;
            if (unhooked || !wiggling) {
                RCLCPP_INFO(
                    node_.get_logger(),
                    "Unstick: forward push (reversed %.2f m tilt=%.1f deg)",
                    moved,
                    tilt * kRadToDeg
                );
                BeginUnstickPhase(UnstickPhase::Drive, pose, tilt);
                cmd.twist.linear.x = kUnstickDriveSpeed;
                cmd.twist.angular.z = SteerOffLip(heading_error, act_wz, cte);
            }
            return;
        }

        cmd.twist.linear.x = kUnstickDriveSpeed;
        cmd.twist.angular.z = SteerOffLip(heading_error, act_wz, cte);
        // XY motion alone is not "free" — last run left at he=-28° and rammed
        // the same lip. Resume only once the long carrot is roughly ahead.
        if (unstickTicks_ >= kUnstickMinTicks && moved >= kUnstickDriveM && heading_clear) {
            LeaveUnstick();
            return;
        }
        if (unstickTicks_ >= kUnstickDriveTicks && !tilt_falling) {
            if (heading_clear && moved >= 0.12) {
                LeaveUnstick();
                return;
            }
            // Already rolling (hairpin orbit, not a hang). Reverse-wiggle here
            // is what threw path 2 i=103 into a 2 m circle.
            if (std::hypot(act_vx, act_wz) > kSeenMotionVel) {
                LeaveUnstick();
                return;
            }
            if (unstickAttempt_ + 1 >= kUnstickMaxAttempts) {
                RCLCPP_WARN(
                    node_.get_logger(), "Unstick gave up after %d cycles (moved %.2f m)", unstickAttempt_ + 1, moved
                );
                LeaveUnstick();
                return;
            }
            ++unstickAttempt_;
            RCLCPP_INFO(node_.get_logger(), "Unstick: reverse wiggle again (fwd moved %.2f m he=%.1f deg)", moved,
                        heading_error * kRadToDeg);
            BeginUnstickPhase(UnstickPhase::Reverse, pose, tilt);
            cmd.twist.linear.x = -kUnstickReverseSpeed;
            cmd.twist.angular.z = unstickWiggleSign_ * kMaxYawRate;
        }
        return;
    }

    if (unstickCooldownTicks_ > 0) {
        --unstickCooldownTicks_;
        stuckTicks_ = 0;
        return;
    }
    if (!seenMotion_) {
        stuckTicks_ = 0;
        return;
    }
    // Intentional in-place turn (carrot was behind) is not a boardwalk hang.
    if (std::abs(heading_error) > kTurnInPlaceRad) {
        stuckTicks_ = 0;
        return;
    }

    const bool commanding =
        std::abs(cmd.twist.linear.x) > kStuckCmdVx || std::abs(cmd.twist.angular.z) > kStuckCmdWz;
    const bool not_moving = std::abs(act_vx) < kStuckActVx && std::abs(act_wz) < kStuckActWz;
    if (!commanding || !not_moving) {
        stuckTicks_ = 0;
        return;
    }
    if (stuckTicks_ == 0) {
        stuckAnchorX_ = pose.x;
        stuckAnchorY_ = pose.y;
    }
    if (std::hypot(pose.x - stuckAnchorX_, pose.y - stuckAnchorY_) > kStuckPoseM) {
        stuckTicks_ = 0;
        stuckAnchorX_ = pose.x;
        stuckAnchorY_ = pose.y;
        return;
    }
    ++stuckTicks_;
    if (stuckTicks_ < kStuckDetectTicks) {
        return;
    }

    const double tracking_vx = cmd.twist.linear.x;
    const double tracking_wz = cmd.twist.angular.z;
    preUnstickMode_ = driveMode_;
    driveMode_ = DriveMode::Unstick;
    unstickAttempt_ = 0;
    stuckTicks_ = 0;
    BeginUnstickPhase(UnstickPhase::Reverse, pose, tilt);
    cmd.twist.linear.x = -kUnstickReverseSpeed;
    cmd.twist.angular.z = unstickWiggleSign_ * kMaxYawRate;
    RCLCPP_INFO(
        node_.get_logger(),
        "Stuck (cmd vx=%.2f wz=%.2f act vx=%.2f wz=%.2f cte=%.2f tilt=%.1f deg) — reverse wiggle then forward",
        tracking_vx, tracking_wz, act_vx, act_wz, cte, tilt * kRadToDeg
    );
}

// Enter Turn at a yaw-flip cusp, or latch finished_ on a recorded rollback tail.
void ArduroverController::UpdateCuspMode(const Waypoint &pose) {
    const std::size_t closest_i = progressIndex_;

    // Drive to the cusp with forward vx (carrot already stopped there).
    // Recorded rollback → stop. Yaw-flip → spin, then continue forward.
    if (driveMode_ == DriveMode::Forward) {
        if (const auto vertex = next_cusp_vertex(path_, closest_i > 0 ? closest_i - 1 : 0)) {
            if (arrived_at_vertex(pose, path_, closest_i, *vertex)) {
                const CuspKind kind = classify_cusp(path_, *vertex);
                if (kind == CuspKind::End && is_rollback_tail(path_, *vertex)) {
                    if (!finished_) {
                        finished_ = true;
                        RCLCPP_INFO(node_.get_logger(), "Cusp i=%zu: recorded rollback, stopping", *vertex);
                    }
                } else if (kind == CuspKind::Turn) {
                    driveMode_ = DriveMode::Turn;
                    turnTargetYaw_ = path_yaw_after_vertex(path_, *vertex);
                    progressIndex_ = std::max(progressIndex_, *vertex);
                    RCLCPP_INFO(node_.get_logger(), "Cusp i=%zu: turn to yaw %.2f", *vertex, turnTargetYaw_);
                }
            }
        }
    } else if (driveMode_ == DriveMode::Turn) {
        if (std::abs(wrap(turnTargetYaw_ - pose.yaw)) < kTurnAlignRad) {
            driveMode_ = DriveMode::Forward;
        }
    }
}

// One tick: step the current segment, start a cusp maneuver if we just
// arrived at a fold, then steer with the carrot (or the Turn yaw target).
void ArduroverController::Control(const nav_msgs::msg::Odometry &odometry) {
    const auto rpy = rpy_from_quat(odometry.pose.pose.orientation);
    Waypoint current_waypoint{odometry.pose.pose.position.x, odometry.pose.pose.position.y, rpy.yaw};

    progressIndex_ = advance_progress(current_waypoint, path_, progressIndex_);
    UpdateCuspMode(current_waypoint);

    const std::size_t track_i = progressIndex_;
    const double track_t =
        (track_i + 1 < path_.size()) ? project_t(current_waypoint, path_[track_i], path_[track_i + 1]) : 1.0;

    const DriveMode heading_mode = (driveMode_ == DriveMode::Unstick) ? preUnstickMode_ : driveMode_;
    const bool turning = heading_mode == DriveMode::Turn;
    const bool unsticking = driveMode_ == DriveMode::Unstick;
    const Pursuit pursuit =
        pursuit_target(path_, current_waypoint, track_i, track_t, turning, unsticking, turnTargetYaw_);
    const Waypoint &target = pursuit.target;
    const double heading_error = pursuit.heading_error;
    const double turn = pursuit.turn;

    const double desired_heading = turning ? turnTargetYaw_
                                           : std::atan2(target.y - current_waypoint.y, target.x - current_waypoint.x);
    const double heading_error_deg = heading_error * kRadToDeg;
    const double dist_target = distance(current_waypoint, target);
    const double dist_goal = distance(current_waypoint, path_.back());
    const double remaining = remaining_path_length(path_, track_i, track_t);
    const double cte = signed_cross_track(current_waypoint, path_, track_i);
    const double yaw_rate = odometry.twist.twist.angular.z;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.now();
    cmd.header.frame_id = "base_link";

    // Finish is the last vertex of this unfolding, not crow-flies to path.back()
    // (path 0's reverse tail is itself ~1 m and is a stop, not leftover length).
    if (!finished_ && at_polyline_end(path_, progressIndex_, track_t)) {
        finished_ = true;
    }
    const bool at_goal = finished_;
    if (!at_goal) {
        cmd.twist.linear.x =
            tracking_speed(path_, current_waypoint, track_i, remaining, turning, turn, heading_error);
        cmd.twist.angular.z = heading_yaw_cmd(heading_error, yaw_rate);
    }

    RecoverStuck(
        current_waypoint,
        odometry.twist.twist.linear.x,
        yaw_rate,
        heading_error,
        cte,
        rpy.roll,
        rpy.pitch,
        at_goal,
        cmd
    );
    const bool unstuck = driveMode_ == DriveMode::Unstick;
    const int unstuck_code = !unstuck                                 ? 0
                             : unstickPhase_ == UnstickPhase::Reverse ? 1
                                                                      : 2;
    const bool wz_sat = !at_goal && std::abs(heading_yaw_pd(heading_error, yaw_rate)) > kMaxYawRate + 1e-9;

    velocityPublisher_->publish(cmd);

    if (logFile_.is_open()) {
        logFile_ << std::fixed << std::setprecision(4) << node_.now().seconds() << ',' << current_waypoint.x << ','
                 << current_waypoint.y << ',' << current_waypoint.yaw << ',' << rpy.roll << ',' << rpy.pitch << ','
                 << odometry.twist.twist.linear.x << ',' << odometry.twist.twist.angular.z << ',' << progressIndex_
                 << ',' << cte << ',' << target.x << ',' << target.y << ',' << dist_target << ',' << dist_goal << ','
                 << desired_heading << ',' << heading_error << ',' << heading_error_deg << ',' << cmd.twist.linear.x
                 << ',' << cmd.twist.angular.z << ',' << wz_sat << ',' << at_goal << ',' << 0 << ','
                 << unstuck_code << '\n';
        logFile_.flush();
    }

    if (at_goal) {
        if (!loggedGoal_) {
            RCLCPP_INFO(
                node_.get_logger(), "At goal (i=%zu dist=%.2f m remaining=%.2f m), stopping", progressIndex_, dist_goal,
                remaining
            );
            loggedGoal_ = true;
        }
        return;
    }

    const char *mode = unstuck ? (unstickPhase_ == UnstickPhase::Reverse ? " UNSTICK-REV" : " UNSTICK-FWD")
                       : turning ? " TURN"
                                 : "";
    RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 500,
        "i=%zu cte=%.3f m  he=%.1f deg  r=%.1f p=%.1f  cmd vx=%.2f wz=%.2f%s%s  act vx=%.2f wz=%.2f  d_la=%.2f remain=%.2f",
        progressIndex_, cte, heading_error_deg, rpy.roll * kRadToDeg, rpy.pitch * kRadToDeg, cmd.twist.linear.x,
        cmd.twist.angular.z, mode, wz_sat ? " SAT" : "", odometry.twist.twist.linear.x, odometry.twist.twist.angular.z,
        dist_target, remaining
    );
}

}  // namespace ardurover_nav
