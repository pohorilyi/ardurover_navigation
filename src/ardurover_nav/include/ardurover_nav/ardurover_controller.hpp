#pragma once

#include <fstream>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

class ArduroverController {
  public:
    ArduroverController(rclcpp::Node& node, std::vector<Waypoint> path);

    bool SetupArdurover();
    // 20 Hz tick: progress → cusp mode → carrot → vx/wz → unstick may overwrite → publish.
    void Control(const nav_msgs::msg::Odometry& odom);

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, SetMode, Arm, Ready };
    // Forward = carrot / pure-pursuit. Turn is a yaw-flip cusp (spin, then go).
    // A same-yaw rollback tail is the end of the forward run, not a drive mode.
    // Unstick is physics recovery (boardwalk lip): reverse to unhook, then drive.
    enum class DriveMode { Forward, Turn, Unstick };
    enum class UnstickPhase { Reverse, Drive };

    void OnState(const mavros_msgs::msg::State& msg);
    void RequestGuided();
    void RequestArm();
    // Unstick map. Control() fills cmd, then calls RecoverStuck, which does one of:
    //   at goal            → cancel
    //   already Unstick    → TickUnstickReverse or TickUnstickDrive (overwrites cmd)
    //   else               → MaybeEnterUnstick (may switch to Reverse and overwrite cmd)
    // LeaveUnstick restores preUnstickMode_ (Forward or Turn). Path index is not moved here.
    void RecoverStuck(
        const Waypoint& pose,
        double act_vx,
        double act_wz,
        double heading_error,
        double cte,
        double roll,
        double pitch,
        bool at_goal,
        geometry_msgs::msg::TwistStamped& cmd
    );
    void LeaveUnstick();
    // Enter Turn at a yaw-flip cusp, or latch finished_ on a recorded rollback tail.
    void UpdateCuspMode(const Waypoint& pose);
    // Reset the unstick timer and pose/tilt anchors when Reverse or Drive starts.
    void BeginUnstickPhase(UnstickPhase phase, const Waypoint& pose, double tilt);
    // Yaw command while driving off a lip: PD on heading, mixed toward the path if CTE is large.
    double SteerOffLip(double heading_error, double act_wz, double cte) const;
    // Reverse wiggle until unhooked (or timeout), then switch to Drive.
    void TickUnstickReverse(
        const Waypoint& pose,
        double act_wz,
        double heading_error,
        double cte,
        double tilt,
        double moved,
        geometry_msgs::msg::TwistStamped& cmd
    );
    // Forward push off the lip; leave Unstick when clear, or reverse again if still hung.
    void TickUnstickDrive(
        const Waypoint& pose,
        double act_vx,
        double act_wz,
        double heading_error,
        double cte,
        double tilt,
        double moved,
        bool tilt_falling,
        bool heading_clear,
        geometry_msgs::msg::TwistStamped& cmd
    );
    // Start Unstick if we have been commanding and frozen on XY.
    void MaybeEnterUnstick(
        const Waypoint& pose,
        double act_vx,
        double act_wz,
        double heading_error,
        double cte,
        double tilt,
        geometry_msgs::msg::TwistStamped& cmd
    );

    rclcpp::Node& node_;
    std::vector<Waypoint> path_;
    SetupState setupState_{SetupState::WaitServices};
    int primeTicks_{0};
    mavros_msgs::msg::State state_;
    std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> modeFuture_;
    std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> armFuture_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr setMode_;
    rclcpp::AsyncParametersClient::SharedPtr paramClient_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr stateSub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocityPublisher_;
    std::ofstream logFile_;
    bool loggedGoal_{false};
    bool finished_{false};          // latch: once at the finish, stay stopped (don't re-accelerate)
    std::size_t progressIndex_{0};  // start of the current segment; only ++ when projection t > 1
    DriveMode driveMode_{DriveMode::Forward};
    double turnTargetYaw_{0.0};     // recorded yaw after a heading-180 cusp (Turn mode)
    DriveMode preUnstickMode_{DriveMode::Forward};
    UnstickPhase unstickPhase_{UnstickPhase::Reverse};
    int stuckTicks_{0};
    int unstickTicks_{0};
    int unstickCooldownTicks_{0};
    int unstickAttempt_{0};
    bool seenMotion_{false};
    double stuckAnchorX_{0.0};
    double stuckAnchorY_{0.0};
    double unstickStartX_{0.0};
    double unstickStartY_{0.0};
    double unstickLastTilt_{0.0};
    double unstickWiggleSign_{1.0};
};

}  // namespace ardurover_nav
