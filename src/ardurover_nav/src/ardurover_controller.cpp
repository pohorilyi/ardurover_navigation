#include "ardurover_nav/ardurover_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <optional>
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
        // reverse=1 and cmd_vx<0 means a real rollback; a jump in closest_i
        // with reverse=0 is the old nearest-XY snap onto the overlapping tail.
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

// Pure-pursuit on a smooth stretch. Progress and 180s are not carrot problems:
// we step one segment at a time, and a cusp is a vertex plus a discrete maneuver.
constexpr double kLookaheadM = 1.4;     // default carrot; 2 m cut corners on path 1/2
constexpr double kLookaheadMinM = 0.6;  // floor in a sharp turn
constexpr double kTurnHeadingRad = 0.7; // |ψe| at which look-ahead/speed are fully tightened
constexpr double kMinTurnSpeed = 0.35;  // fraction of cruise when |ψe| is large
constexpr double kCruiseSpeed = 0.8;
constexpr double kHeadingP = 1.0;     // flip sign if BODY_NED steers the wrong way
constexpr double kMaxYawRate = 0.6;   // stop spin-outs when heading error is large
constexpr double kSlowdownM = 4.0;    // bleed speed into the true polyline end (not a stop rule)
constexpr double kCuspDot = -0.5;     // next tangent opposite current → fold / 180 vertex
constexpr double kSegPastEps = 1e-3;  // increment index only once projection is past the end
constexpr double kTinySegM = 1e-4;    // skip degenerate samples when taking a tangent
constexpr double kYawSameRad = 0.7;   // path yaw after the fold matches vehicle heading
constexpr double kYawFlipRad = 2.0;   // ~115 deg: recorded yaw flipped (real U-turn)
constexpr double kTurnAlignRad = 0.25;  // leave Turn once heading matches the outgoing yaw
constexpr double kTurnCreep = 0.15;     // slow vx while spinning at a heading-180
constexpr double kCuspApproachM = 1.5;  // bleed speed so we arrive at the vertex before reversing
constexpr double kReverseEndSlowM = 0.4;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
// Boardwalk hang: reverse to unhook, then a forward+yaw push while the bumper
// is free. Leaving on reverse-distance alone rams the same lip (path 2 i=91).
// GUIDED ramp-up also looks like act≈0 — wait until we have actually rolled.
constexpr int kStuckDetectTicks = 30;       // 1.5 s still commanding and frozen
constexpr int kUnstickMinTicks = 8;         // 0.4 s before a phase may end
constexpr int kUnstickReverseTicks = 20;    // 1.0 s reverse to unhook
constexpr int kUnstickDriveTicks = 36;      // 1.8 s forward push around the lip
constexpr int kUnstickMaxAttempts = 4;      // reverse→drive cycles before giving up
constexpr int kUnstickCooldownTicks = 30;   // 1.5 s before another unstick
constexpr double kStuckCmdVx = 0.20;
constexpr double kStuckCmdWz = 0.15;
constexpr double kStuckActVx = 0.04;        // slow slide (0.06) is not a hang yet
constexpr double kStuckActWz = 0.04;
constexpr double kStuckPoseM = 0.04;
constexpr double kUnstickReverseM = 0.22;   // unhook distance; then push forward
constexpr double kUnstickDriveM = 0.40;     // leave only after a real forward gain
constexpr double kUnstickReverseSpeed = 0.45;
constexpr double kUnstickDriveSpeed = 0.55;
constexpr double kSeenMotionVel = 0.20;     // have we ever really moved this run?
constexpr double kTurnInPlaceRad = 0.85;    // |ψe| above this → vx=0 (do not drive off the curb)
constexpr double kTiltRad = 0.10;           // ~6 deg: chassis on a lip (roll/pitch)

double distance(const Waypoint &a, const Waypoint &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double segment_length(const std::vector<Waypoint> &path, std::size_t i) {
    if (i + 1 >= path.size()) {
        return 0.0;
    }
    return distance(path[i], path[i + 1]);
}

// Unclamped projection of pose onto segment [a, b]. t in [0, 1] is on the
// segment; t > 1 means we have passed the end and may step the index.
// Degenerate (zero-length) samples count as already past so we skip them.
double project_t(const Waypoint &pose, const Waypoint &a, const Waypoint &b) {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    if (len2 < 1e-16) {
        return 1.0;
    }
    return ((pose.x - a.x) * abx + (pose.y - a.y) * aby) / len2;
}

struct Tangent {
    double dx{0.0};
    double dy{0.0};
    double len{0.0};
};

// First non-tiny chord into / out of a vertex. Recordings leave 1e-4 m jitter
// samples that would otherwise give a nonsense heading.
std::optional<Tangent> incoming_tangent(const std::vector<Waypoint> &path, std::size_t vertex) {
    if (vertex == 0) {
        return std::nullopt;
    }
    for (std::size_t i = vertex; i-- > 0;) {
        const double dx = path[vertex].x - path[i].x;
        const double dy = path[vertex].y - path[i].y;
        const double len = std::hypot(dx, dy);
        if (len > kTinySegM) {
            return Tangent{dx, dy, len};
        }
    }
    return std::nullopt;
}

std::optional<Tangent> outgoing_tangent(const std::vector<Waypoint> &path, std::size_t vertex) {
    for (std::size_t i = vertex + 1; i < path.size(); ++i) {
        const double dx = path[i].x - path[vertex].x;
        const double dy = path[i].y - path[vertex].y;
        const double len = std::hypot(dx, dy);
        if (len > kTinySegM) {
            return Tangent{dx, dy, len};
        }
    }
    return std::nullopt;
}

double tangent_dot(const Tangent &a, const Tangent &b) {
    return (a.dx * b.dx + a.dy * b.dy) / (a.len * b.len);
}

// Fold in the polyline: incoming and outgoing travel point opposite ways.
// This is a vertex problem, not a "carrot is behind us" problem.
bool is_cusp_at_vertex(const std::vector<Waypoint> &path, std::size_t vertex) {
    const auto in = incoming_tangent(path, vertex);
    const auto out = outgoing_tangent(path, vertex);
    return in && out && tangent_dot(*in, *out) < kCuspDot;
}

// First cusp at or after the end of from_seg. Used both to stop the carrot
// and to decide when we have physically arrived at the fold.
std::optional<std::size_t> next_cusp_vertex(const std::vector<Waypoint> &path, std::size_t from_seg) {
    const std::size_t start = from_seg + 1;
    for (std::size_t v = start; v + 1 < path.size(); ++v) {
        if (is_cusp_at_vertex(path, v)) {
            return v;
        }
    }
    return std::nullopt;
}

enum class CuspKind { Reverse, Turn, Pass };

// How to leave a cusp. Do not use |heading_error| to a carrot — after an
// index jump that carrot can already sit on the overlapping reverse tail.
//
// Reverse: path yaw is unchanged and XY after the fold goes the other way
//   (path 0/1 rollback; heading already matches body-back travel).
// Turn: recorded yaw flips ~180° — a real U-turn, spin then go forward.
// Pass: same yaw, travel still ahead — recording wiggle, just step through.
CuspKind classify_cusp(const std::vector<Waypoint> &path, std::size_t vertex, double vehicle_yaw) {
    const auto out = outgoing_tangent(path, vertex);
    if (!out) {
        return CuspKind::Pass;
    }
    const double travel = std::atan2(out->dy, out->dx);
    const std::size_t yaw_i = std::min(vertex + 1, path.size() - 1);
    const double path_yaw_after = path[yaw_i].yaw;
    const double yaw_err = std::abs(wrap(path_yaw_after - vehicle_yaw));
    const double travel_vs_yaw = std::abs(wrap(travel - vehicle_yaw));
    if (yaw_err < kYawSameRad && travel_vs_yaw > kYawFlipRad) {
        return CuspKind::Reverse;
    }
    if (yaw_err > kYawFlipRad) {
        return CuspKind::Turn;
    }
    return CuspKind::Pass;
}

// Recorded heading on the outgoing side of the fold — the yaw we spin toward.
double path_yaw_after_vertex(const std::vector<Waypoint> &path, std::size_t vertex) {
    return path[std::min(vertex + 1, path.size() - 1)].yaw;
}

// Arrived = we have stepped onto the vertex, or the projection on the
// inbound segment is past it and we are actually nearby (not a distant
// collinear projection onto an overlapping later stretch).
bool arrived_at_vertex(
    const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t progress, std::size_t vertex
) {
    if (progress >= vertex) {
        return true;
    }
    if (vertex == 0) {
        return true;
    }
    if (distance(pose, path[vertex]) > 2.0) {
        return false;
    }
    return project_t(pose, path[vertex - 1], path[vertex]) >= 1.0 - kSegPastEps;
}

Waypoint interpolate(const Waypoint &a, const Waypoint &b, double t) {
    t = std::clamp(t, 0.0, 1.0);
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), wrap(a.yaw + t * wrap(b.yaw - a.yaw))};
}

// Carrot at along-track s + L on this unfolding of the polyline (interpolated,
// not the nearest sample). Stop at a cusp so a 1.4 m look-ahead does not sit
// on path 0's ~1 m rollback while we are still outbound.
Waypoint lookahead_waypoint(const std::vector<Waypoint> &path, std::size_t from, double t_on_seg, double lookahead_m) {
    if (path.size() < 2) {
        return path.front();
    }
    if (from + 1 >= path.size()) {
        return path.back();
    }
    const double t0 = std::clamp(t_on_seg, 0.0, 1.0);
    const double seg_len = segment_length(path, from);
    const double remain_on_seg = (1.0 - t0) * seg_len;
    if (lookahead_m <= remain_on_seg) {
        const double u = (seg_len < 1e-12) ? 1.0 : t0 + lookahead_m / seg_len;
        return interpolate(path[from], path[from + 1], u);
    }
    lookahead_m -= remain_on_seg;
    for (std::size_t i = from + 1; i + 1 < path.size(); ++i) {
        if (is_cusp_at_vertex(path, i)) {
            return path[i];
        }
        const double len = segment_length(path, i);
        if (lookahead_m <= len) {
            return interpolate(path[i], path[i + 1], (len < 1e-12) ? 1.0 : lookahead_m / len);
        }
        lookahead_m -= len;
    }
    return path.back();
}

// Leftover polyline from the current projection. Used only to bleed speed;
// never as a stop condition (path 0's reverse tail is itself ~1 m).
double remaining_path_length(const std::vector<Waypoint> &path, std::size_t from, double t_on_seg) {
    if (from + 1 >= path.size()) {
        return 0.0;
    }
    double length = (1.0 - std::clamp(t_on_seg, 0.0, 1.0)) * segment_length(path, from);
    for (std::size_t i = from + 1; i + 1 < path.size(); ++i) {
        length += segment_length(path, i);
    }
    return length;
}

// Signed distance to the current segment. Positive = left of travel direction.
double signed_cross_track(const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t seg_i) {
    if (path.size() < 2) {
        return 0.0;
    }
    const std::size_t i0 = (seg_i + 1 < path.size()) ? seg_i : path.size() - 2;
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

// True end of this unfolding: last index, or last segment with t past 1.
// Crow-flies to path.back() is the same collinear trap as the old 136→156 jump.
bool at_polyline_end(const std::vector<Waypoint> &path, std::size_t progress, double t_on_seg) {
    if (path.size() < 2) {
        return true;
    }
    if (progress + 1 >= path.size()) {
        return true;
    }
    return progress + 1 == path.size() - 1 && t_on_seg >= 1.0 - kSegPastEps;
}

void ArduroverController::LeaveUnstick() {
    const char *resume = preUnstickMode_ == DriveMode::Reverse ? "REV"
                         : preUnstickMode_ == DriveMode::Turn  ? "TURN"
                                                               : "FWD";
    RCLCPP_INFO(node_.get_logger(), "Unstick done, resume %s", resume);
    driveMode_ = preUnstickMode_;
    unstickPhase_ = UnstickPhase::Reverse;
    unstickTicks_ = 0;
    stuckTicks_ = 0;
    unstickAttempt_ = 0;
    unstickCooldownTicks_ = kUnstickCooldownTicks;
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
    auto steer_off_lip = [&]() {
        double wz = std::clamp(kHeadingP * heading_error, -kMaxYawRate, kMaxYawRate);
        if (std::abs(cte) > 0.15) {
            const double toward_path = -std::copysign(kMaxYawRate, cte);
            wz = 0.5 * wz + 0.5 * toward_path;
            wz = std::clamp(wz, -kMaxYawRate, kMaxYawRate);
        }
        return wz;
    };
    auto begin_phase = [&](UnstickPhase phase) {
        unstickPhase_ = phase;
        unstickTicks_ = 0;
        unstickStartX_ = pose.x;
        unstickStartY_ = pose.y;
        unstickLastTilt_ = tilt;
    };

    if (driveMode_ == DriveMode::Unstick) {
        ++unstickTicks_;
        const double moved = std::hypot(pose.x - unstickStartX_, pose.y - unstickStartY_);
        const bool tilt_falling = tilt > kTiltRad && tilt + 0.015 < unstickLastTilt_;
        unstickLastTilt_ = tilt;
        cmd.twist.angular.z = steer_off_lip();

        if (unstickPhase_ == UnstickPhase::Reverse) {
            cmd.twist.linear.x = -kUnstickReverseSpeed;
            const bool unhooked = unstickTicks_ >= kUnstickMinTicks && moved >= kUnstickReverseM;
            const bool timed_out = unstickTicks_ >= kUnstickReverseTicks;
            if (unhooked || timed_out) {
                RCLCPP_INFO(
                    node_.get_logger(),
                    "Unstick: forward push (reversed %.2f m tilt=%.1f deg)",
                    moved,
                    tilt * kRadToDeg
                );
                begin_phase(UnstickPhase::Drive);
                cmd.twist.linear.x = kUnstickDriveSpeed;
            }
            return;
        }

        cmd.twist.linear.x = kUnstickDriveSpeed;
        if (unstickTicks_ >= kUnstickMinTicks && moved >= kUnstickDriveM) {
            LeaveUnstick();
            return;
        }
        // Tilt dropping means the chassis is coming down onto the path — keep
        // the forward push instead of rocking back into reverse.
        if (unstickTicks_ >= kUnstickDriveTicks && !tilt_falling) {
            if (moved >= 0.12) {
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
            RCLCPP_INFO(node_.get_logger(), "Unstick: reverse again (fwd moved %.2f m)", moved);
            begin_phase(UnstickPhase::Reverse);
            cmd.twist.linear.x = -kUnstickReverseSpeed;
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
    begin_phase(UnstickPhase::Reverse);
    cmd.twist.linear.x = -kUnstickReverseSpeed;
    cmd.twist.angular.z = steer_off_lip();
    RCLCPP_INFO(
        node_.get_logger(),
        "Stuck (cmd vx=%.2f wz=%.2f act vx=%.2f wz=%.2f cte=%.2f tilt=%.1f deg) — reverse then forward",
        tracking_vx, tracking_wz, act_vx, act_wz, cte, tilt * kRadToDeg
    );
}

// One tick: step the current segment, start a cusp maneuver if we just
// arrived at a fold, then steer with the carrot (or the Turn yaw target).
void ArduroverController::Control(const nav_msgs::msg::Odometry &odometry) {
    const auto rpy = rpy_from_quat(odometry.pose.pose.orientation);
    Waypoint current_waypoint{odometry.pose.pose.position.x, odometry.pose.pose.position.y, rpy.yaw};

    // Step along the current segment only. Never snap to a closer sample on an
    // overlapping reverse tail (that was the 136 → 156 jump on path 0).
    if (path_.size() >= 2) {
        const std::size_t last_pt = path_.size() - 1;
        const std::size_t last_seg = path_.size() - 2;
        while (progressIndex_ < last_seg) {
            const double t = project_t(current_waypoint, path_[progressIndex_], path_[progressIndex_ + 1]);
            if (t > 1.0 - kSegPastEps) {
                ++progressIndex_;
            } else {
                break;
            }
        }
        if (progressIndex_ == last_seg &&
            project_t(current_waypoint, path_[last_seg], path_[last_pt]) > 1.0 - kSegPastEps) {
            progressIndex_ = last_pt;
        }
    }

    const std::size_t closest_i = progressIndex_;

    // Drive to the cusp vertex with forward vx (carrot already stopped there).
    // Only then pick reverse vs spin from the path geometry, and pin the index
    // on the outgoing side so look-ahead walks the reverse tail instead of
    // sitting on the vertex.
    if (driveMode_ == DriveMode::Forward) {
        if (const auto vertex = next_cusp_vertex(path_, closest_i > 0 ? closest_i - 1 : 0)) {
            if (arrived_at_vertex(current_waypoint, path_, closest_i, *vertex)) {
                const CuspKind kind = classify_cusp(path_, *vertex, current_waypoint.yaw);
                if (kind == CuspKind::Reverse) {
                    driveMode_ = DriveMode::Reverse;
                    progressIndex_ = std::max(progressIndex_, *vertex);
                    RCLCPP_INFO(node_.get_logger(), "Cusp i=%zu: reverse (same yaw, travel behind)", *vertex);
                } else if (kind == CuspKind::Turn) {
                    driveMode_ = DriveMode::Turn;
                    turnTargetYaw_ = path_yaw_after_vertex(path_, *vertex);
                    progressIndex_ = std::max(progressIndex_, *vertex);
                    RCLCPP_INFO(node_.get_logger(), "Cusp i=%zu: turn to yaw %.2f", *vertex, turnTargetYaw_);
                }
            }
        }
    } else if (driveMode_ == DriveMode::Turn) {
        if (std::abs(wrap(turnTargetYaw_ - current_waypoint.yaw)) < kTurnAlignRad) {
            driveMode_ = DriveMode::Forward;
        }
    }

    const std::size_t track_i = progressIndex_;
    const double track_t =
        (track_i + 1 < path_.size()) ? project_t(current_waypoint, path_[track_i], path_[track_i + 1]) : 1.0;

    const DriveMode heading_mode = (driveMode_ == DriveMode::Unstick) ? preUnstickMode_ : driveMode_;
    const bool reversing = heading_mode == DriveMode::Reverse;
    const bool turning = heading_mode == DriveMode::Turn;

    // BODY_NED heading to the carrot. Reverse uses the same carrot on the
    // outgoing stretch, then subtracts π so we steer while backing instead of
    // yawing 180. Turn aims at the recorded outgoing yaw, not the carrot.
    auto heading_toward = [&](const Waypoint &to) {
        return wrap(std::atan2(to.y - current_waypoint.y, to.x - current_waypoint.x) - current_waypoint.yaw);
    };

    double lookahead_m = kLookaheadM;
    double turn = 0.0;
    Waypoint target = lookahead_waypoint(path_, track_i, track_t, lookahead_m);
    double heading_error = heading_toward(target);
    if (reversing) {
        heading_error = wrap(heading_error - std::copysign(kPi, heading_error));
    } else if (turning) {
        heading_error = wrap(turnTargetYaw_ - current_waypoint.yaw);
        target = lookahead_waypoint(path_, track_i, track_t, kLookaheadMinM);
    } else {
        // Shorten the carrot in a bend so we do not cut path 1/2 vertices.
        // Do not shrink it globally hoping a 180 falls out of the look-ahead.
        turn = std::clamp(std::abs(heading_error) / kTurnHeadingRad, 0.0, 1.0);
        lookahead_m = kLookaheadM + (kLookaheadMinM - kLookaheadM) * turn;
        if (turn > 0.0) {
            target = lookahead_waypoint(path_, track_i, track_t, lookahead_m);
            heading_error = heading_toward(target);
        }
    }

    const double desired_heading = turning ? turnTargetYaw_
                                           : std::atan2(target.y - current_waypoint.y, target.x - current_waypoint.x);
    const double heading_error_deg = heading_error * kRadToDeg;
    const double dist_target = distance(current_waypoint, target);
    const double dist_goal = distance(current_waypoint, path_.back());
    const double remaining = remaining_path_length(path_, track_i, track_t);
    const double cte = signed_cross_track(current_waypoint, path_, track_i);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.now();
    cmd.header.frame_id = "base_link";

    // Finish is the last vertex of this unfolding, not crow-flies to path.back()
    // and not leftover length (path 0's reverse tail is itself ~1 m).
    if (!finished_ && at_polyline_end(path_, progressIndex_, track_t)) {
        finished_ = true;
    }
    const bool at_goal = finished_;
    if (!at_goal) {
        // Reverse keeps cruise so a 1 m rollback is visible; only creep at a
        // U-turn. Forward still bleeds into the real end and into a cusp so
        // we do not punch through the vertex still commanding +vx.
        double end_scale = 1.0;
        if (reversing) {
            end_scale = remaining < kReverseEndSlowM ? std::clamp(remaining / kReverseEndSlowM, 0.35, 1.0) : 1.0;
        } else if (!turning) {
            end_scale = std::clamp(remaining / kSlowdownM, 0.0, 1.0);
            if (const auto vertex = next_cusp_vertex(path_, track_i > 0 ? track_i - 1 : 0)) {
                if (!arrived_at_vertex(current_waypoint, path_, track_i, *vertex)) {
                    const double to_cusp = distance(current_waypoint, path_[*vertex]);
                    if (to_cusp < kCuspApproachM) {
                        end_scale = std::min(end_scale, std::clamp(to_cusp / kCuspApproachM, 0.3, 1.0));
                    }
                }
            }
        }
        const double turn_scale = turning ? 1.0 : 1.0 - (1.0 - kMinTurnSpeed) * turn;
        double speed = turning ? kTurnCreep : kCruiseSpeed * end_scale * turn_scale;
        // After a hang the heading can be ~90°. Forward vx then drives off the
        // pathwalk; spin (or the unstick rock) until the carrot is ahead.
        if (!reversing && !turning && std::abs(heading_error) > kTurnInPlaceRad) {
            speed = 0.0;
        }
        cmd.twist.linear.x = (reversing ? -1.0 : 1.0) * speed;
        cmd.twist.angular.z = std::clamp(kHeadingP * heading_error, -kMaxYawRate, kMaxYawRate);
    }

    RecoverStuck(
        current_waypoint,
        odometry.twist.twist.linear.x,
        odometry.twist.twist.angular.z,
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
    const bool wz_sat = !at_goal && std::abs(kHeadingP * heading_error) > kMaxYawRate + 1e-9;

    velocityPublisher_->publish(cmd);

    if (logFile_.is_open()) {
        logFile_ << std::fixed << std::setprecision(4) << node_.now().seconds() << ',' << current_waypoint.x << ','
                 << current_waypoint.y << ',' << current_waypoint.yaw << ',' << rpy.roll << ',' << rpy.pitch << ','
                 << odometry.twist.twist.linear.x << ',' << odometry.twist.twist.angular.z << ',' << progressIndex_
                 << ',' << cte << ',' << target.x << ',' << target.y << ',' << dist_target << ',' << dist_goal << ','
                 << desired_heading << ',' << heading_error << ',' << heading_error_deg << ',' << cmd.twist.linear.x
                 << ',' << cmd.twist.angular.z << ',' << wz_sat << ',' << at_goal << ',' << (reversing ? 1 : 0) << ','
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

    const char *mode = unstuck                                      ? (unstickPhase_ == UnstickPhase::Reverse ? " UNSTICK-REV"
                                                                                                             : " UNSTICK-FWD")
                       : reversing                                  ? " REV"
                       : turning                                    ? " TURN"
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
