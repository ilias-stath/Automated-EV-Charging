#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ur_simulation_gz/action/move_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
 
class MoveMoveit : public rclcpp::Node {
public:
  using MoveToPose = ur_simulation_gz::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  MoveMoveit() : Node("move_moveit", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(), "Constructor called. MoveGroupInterface will be initialized in the initialize() method.");

    RCLCPP_INFO(this->get_logger(), "Publishing to /movement/pose");
    pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("/movement/pose", 1);

    RCLCPP_INFO(this->get_logger(), "Publishing to /movement/status");
    move_status_pub = this->create_publisher<std_msgs::msg::String>("/movement/status", 10);

    RCLCPP_INFO(this->get_logger(), "Subscribing to /movement/status");
    move_status_sub = this->create_subscription<std_msgs::msg::String>("/movement/status", 10, std::bind(&MoveMoveit::status_callback, this, std::placeholders::_1));
  
  }
 
  // This method initializes components that depend on shared_from_this()
  void initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing action server and MoveGroupInterface...");
    
    // Create the MoveGroupInterface instance only once
    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_moveit",
      std::bind(&MoveMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveMoveit::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveMoveit::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Initialization complete.");
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  // This is the single, shared instance of MoveGroupInterface.
  std::shared_ptr<MoveGroupInterface> move_group_interface_;
  
  double x, y, z, qx, qy, qz, qw, dx, tempX, tempY, tempZ, thx, thy, thz;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_status_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_status_sub;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
 
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received move goal");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling goal and stopping robot.");
    (void)goal_handle;
    
    if (move_group_interface_) {
      move_group_interface_->stop();
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&MoveMoveit::execute, this, goal_handle)}.detach();
  }
 
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<MoveToPose::Result>();
    auto goal = goal_handle->get_goal();
 
    geometry_msgs::msg::Pose start_pose = move_group_interface_->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose1 = start_pose;
    geometry_msgs::msg::Pose target_pose2 = start_pose;
    geometry_msgs::msg::Pose goal_pose = start_pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal->x, goal->y, goal->z, goal->qx, goal->qy, goal->qz, goal->qw);
    RCLCPP_INFO(this->get_logger(), "Arm at: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",start_pose.position.x,start_pose.position.y,start_pose.position.z,start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);


    thx = goal->thz;
    thy = goal->thx;
    thz = goal->thy;
    tempX = 0.0;
    tempY = 0.0;
    tempZ = 0.0;
    
    dx = goal->x - start_pose.position.x;
    x = dx/2;

    RCLCPP_INFO(this->get_logger(), "thx = %f",thx);
    RCLCPP_INFO(this->get_logger(), "thy = %f",thy);
    RCLCPP_INFO(this->get_logger(), "thz = %f",thz);
    RCLCPP_INFO(this->get_logger(), "approach_index = %d",goal->approach_index);

    if(goal->approach_index == 0){


      // We need to be sure that we rotate in the order of Z-X-Y

      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      // Create a new quaternion from the user-provided RPY values.
      tf2::Quaternion relative_quat;
      // Create a quanternion for the final result
      tf2::Quaternion target_quat;


      // First orient by Z
      tf2::fromMsg(start_pose.orientation, current_quat);
      relative_quat.setRPY(0.0, 0.0, goal->qz);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      target_pose2.orientation = tf2::toMsg(target_quat);


      // Then orient by X
      tf2::fromMsg(target_pose2.orientation, current_quat);
      relative_quat.setRPY(goal->qx, 0.0, 0.0);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      target_pose2.orientation = tf2::toMsg(target_quat);


      // Then orient by Y
      tf2::fromMsg(target_pose2.orientation, current_quat);
      relative_quat.setRPY(0.0, goal->qy, 0.0);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      target_pose2.orientation = tf2::toMsg(target_quat);
      goal_pose.orientation = tf2::toMsg(target_quat);

      // First half x
      target_pose1.position.x = x;

      target_pose2.position.x = x;

      // Then allign at y and z
      goal_pose.position.x = x;


      // From Z-X-Y rotation matrix
      tempX = 0.0;
      tempY = goal->y;
      tempZ = goal->z;

      // 1. Convert tempX/Y/Z into a tf2::Vector3
      tf2::Vector3 relative_translation(tempY, tempZ, tempX);

      // 2. Rotate the translation vector by the current quaternion
      tf2::Vector3 global_translation = tf2::quatRotate(target_quat, relative_translation);

      // 3. Apply the global translation
      goal_pose.position.x += global_translation.x();
      goal_pose.position.y += global_translation.y();
      goal_pose.position.z += global_translation.z();

    
      RCLCPP_INFO(this->get_logger(), "Computed target_pose1: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Computed target_pose2: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose2.position.x,target_pose2.position.y,target_pose2.position.z,target_pose2.orientation.x,target_pose2.orientation.y,target_pose2.orientation.z,target_pose2.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);

      waypoints.push_back(start_pose);
      waypoints.push_back(target_pose1);
      waypoints.push_back(target_pose2);
      waypoints.push_back(goal_pose);


    }else if(goal->approach_index == 1){

      // Allign y and z
      tempX = 0.0;
      tempY = goal->y;
      tempZ = goal->z;

      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      tf2::fromMsg(start_pose.orientation, current_quat);

      // 1. Convert tempX/Y/Z into a tf2::Vector3
      tf2::Vector3 relative_translation(tempY, tempZ, tempX);

      // 2. Rotate the translation vector by the current quaternion
      tf2::Vector3 global_translation = tf2::quatRotate(current_quat, relative_translation);

      // 3. Apply the global translation
      goal_pose.position.x += global_translation.x();
      goal_pose.position.y += global_translation.y();
      goal_pose.position.z += global_translation.z();

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
 
      waypoints.push_back(start_pose);
      waypoints.push_back(goal_pose);
    }else if(goal->approach_index == 2){

      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      // Create a new quaternion from the user-provided RPY values.
      tf2::Quaternion relative_quat;
      // Create a quanternion for the final result
      tf2::Quaternion target_quat;


      // First orient by Z
      tf2::fromMsg(start_pose.orientation, current_quat);
      relative_quat.setRPY(0.0, 0.0, goal->qz);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      goal_pose.orientation = tf2::toMsg(target_quat);


      // Then orient by X
      tf2::fromMsg(goal_pose.orientation, current_quat);
      relative_quat.setRPY(goal->qx, 0.0, 0.0);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      goal_pose.orientation = tf2::toMsg(target_quat);


      // Then orient by Y
      tf2::fromMsg(goal_pose.orientation, current_quat);
      relative_quat.setRPY(0.0, goal->qy, 0.0);
      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      target_quat = current_quat * relative_quat;
      target_quat.normalize();
      goal_pose.orientation = tf2::toMsg(target_quat);

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
      
      waypoints.push_back(start_pose);
      waypoints.push_back(goal_pose);
    }else if(goal->approach_index == 3){
      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      tf2::fromMsg(start_pose.orientation, current_quat);

      // Create a new quaternion from the user-provided RPY values.
      tf2::Quaternion relative_quat;
      relative_quat.setRPY(0.0, 0.0, goal->qz);

      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      tf2::Quaternion target_quat = current_quat * relative_quat;
      target_quat.normalize();

      goal_pose.orientation = tf2::toMsg(target_quat);

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);

      waypoints.push_back(start_pose);
      waypoints.push_back(goal_pose);
    }else if (goal->approach_index == 4){

      tempX = 0.0;
      tempY = 0.0;
      tempZ = goal->z - 0.1;

      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      tf2::fromMsg(start_pose.orientation, current_quat);

      // 1. Convert tempX/Y/Z into a tf2::Vector3
      tf2::Vector3 relative_translation(tempY, tempZ, tempX);

      // 2. Rotate the translation vector by the current quaternion
      tf2::Vector3 global_translation = tf2::quatRotate(current_quat, relative_translation);

      // 3. Apply the global translation
      target_pose1.position.x += global_translation.x();
      target_pose1.position.y += global_translation.y();
      target_pose1.position.z += global_translation.z();

      // goal_pose.position.x += goal->x + 0.1;
      goal_pose.position.x = target_pose1.position.x;
      goal_pose.position.y = target_pose1.position.y;
      goal_pose.position.z = target_pose1.position.z;

      tempX = -goal->x - 0.1;
      tempY = 0.0;
      tempZ = 0.0;

      // 1. Convert tempX/Y/Z into a tf2::Vector3
      relative_translation = tf2::Vector3(tempY, tempZ, tempX); 

      // 2. Rotate the translation vector by the current quaternion
      global_translation = tf2::quatRotate(current_quat, relative_translation);

      // 3. Apply the global translation
      goal_pose.position.x += global_translation.x();
      goal_pose.position.y += global_translation.y();
      goal_pose.position.z += global_translation.z();


      RCLCPP_INFO(this->get_logger(), "Computed target_pose1: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
      

      waypoints.push_back(start_pose);
      waypoints.push_back(target_pose1);
      waypoints.push_back(goal_pose);
    }else{

      target_pose1.position.x += x;

      target_pose2.position = target_pose1.position;
      target_pose2.orientation.x = goal->qx;
      target_pose2.orientation.y = goal->qy;
      target_pose2.orientation.z = goal->qz;
      target_pose2.orientation.w = goal->qw;


      goal_pose.position.x = target_pose2.position.x;
      goal_pose.position.y = goal->y;
      goal_pose.position.z = goal->z;
      goal_pose.orientation = target_pose2.orientation;

      RCLCPP_INFO(this->get_logger(), "Computed target_pose1: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Computed target_pose2: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose2.position.x,target_pose2.position.y,target_pose2.position.z,target_pose2.orientation.x,target_pose2.orientation.y,target_pose2.orientation.z,target_pose2.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);

      waypoints.push_back(start_pose);
      waypoints.push_back(target_pose1);
      waypoints.push_back(target_pose2);
      waypoints.push_back(goal_pose);

      goal_pose.position.x = goal->x;

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
  
      waypoints.push_back(goal_pose);

    }
  
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory);
 
    if (fraction < 0.99) {
      result->success = false;
      result->message = "Planning failed";
      RCLCPP_INFO(this->get_logger(), "Planning failed!!!");
      goal_handle->abort(result);

      auto msg = std_msgs::msg::String();
      msg.data = "done";
      move_status_pub->publish(msg);

      return;
    }
 
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_result = move_group_interface_->execute(plan);

    // I will need to move this into exec_result success
    pose_pub->publish(goal_pose);

    RCLCPP_INFO(this->get_logger(), "Published end-effector pose to /movement/pose.");
    RCLCPP_INFO(this->get_logger(), "Pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);
 
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = true;
      result->message = "Execution succeeded";
      RCLCPP_INFO(this->get_logger(), "Execution succeeded!!!");
      goal_handle->succeed(result);

      auto msg = std_msgs::msg::String();
      msg.data = "done";
      move_status_pub->publish(msg);

    } else {
      result->success = false;
      result->message = "Execution failed";
      RCLCPP_INFO(this->get_logger(), "Execution failed!!!");
      goal_handle->abort(result);

      auto msg = std_msgs::msg::String();
      msg.data = "done";
      move_status_pub->publish(msg);
    }
  }

  void pose_send_thread(){
    RCLCPP_INFO(this->get_logger(), "Waiting for robot state to be received...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    // Use the member variable instance to get the pose
    geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
    pose_pub->publish(current_pose);

    RCLCPP_INFO(this->get_logger(), "Published end-effector pose to /movement/pose.");
    RCLCPP_INFO(this->get_logger(), "Pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",
        current_pose.position.x, current_pose.position.y, current_pose.position.z,
        current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  }

  void status_callback(const std_msgs::msg::String::SharedPtr msg){
    if(msg->data == "pose"){
      RCLCPP_INFO(this->get_logger(), "Received 'pose' request. Launching pose retrieval in a new thread.");
      // Launch a new thread to handle the request. This will not block the main callback queue.
      std::thread(&MoveMoveit::pose_send_thread, this).detach();
    }
  }
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create a shared pointer to your node
  auto node = std::make_shared<MoveMoveit>();
  
  // Call the initialize method after the shared pointer is created
  node->initialize();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
