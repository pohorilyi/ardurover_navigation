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
    void Control(const nav_msgs::msg::Odometry& odom);

  private:
    enum class SetupState { WaitServices, SetFrame, Prime, SetMode, Arm, Ready };

    void OnState(const mavros_msgs::msg::State& msg);
    void RequestGuided();
    void RequestArm();

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
    bool finished_{false};       // latch: once at the finish, stay stopped (don't re-accelerate)
    std::size_t progressIndex_{0};  // monotonic closest index; never snap back along the path
};

}  // namespace ardurover_nav
