#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ur_simulation_gz/action/move_to_pose.hpp"
 
class AbsMoveMoveit : public rclcpp::Node {
public:
  using MoveToPose = ur_simulation_gz::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
 
  AbsMoveMoveit() : Node("abs_move_moveit", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    // Declare and set use_sim_time
    // this->declare_parameter<bool>("use_sim_time", true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "abs_move",
      std::bind(&AbsMoveMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AbsMoveMoveit::handle_cancel, this, std::placeholders::_1),
      std::bind(&AbsMoveMoveit::handle_accepted, this, std::placeholders::_1)
    );
  }

 
private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
 
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received absolute move goal: [%.3f, %.3f, %.3f]",
                goal->x, goal->y, goal->z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
 
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&AbsMoveMoveit::execute, this, goal_handle)}.detach();
  }
 
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<MoveToPose::Result>();
    auto goal = goal_handle->get_goal();
 
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group_interface(shared_from_this(), "ur_manipulator");
 
    // Target pose (absolute)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = goal->x;
    target_pose.position.y = goal->y;
    target_pose.position.z = goal->z;
    target_pose.orientation.x = goal->qx;
    target_pose.orientation.y = goal->qy;
    target_pose.orientation.z = goal->qz;
    target_pose.orientation.w = goal->qw;
 
    // Start pose
    geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
 
    // Waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);
 
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.5);
 
    // Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);
 
    if (fraction < 0.99) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed (%.2f%% achieved)", fraction * 100.0);
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      return;
    }
 
    // Execute
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_result = move_group_interface.execute(plan);
 
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Execution succeeded");
      result->success = true;
      result->message = "Execution succeeded";
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Execution failed");
      result->success = false;
      result->message = "Execution failed";
      goal_handle->abort(result);
    }
  }
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbsMoveMoveit>());
  rclcpp::shutdown();
  return 0;
}