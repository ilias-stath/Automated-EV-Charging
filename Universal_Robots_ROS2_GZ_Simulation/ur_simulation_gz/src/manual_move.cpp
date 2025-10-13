#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ur_simulation_gz/action/move_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
 
class manualMoveit : public rclcpp::Node {
public:
  using MoveToPose = ur_simulation_gz::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  bool init = false;
  double thx=0,thy=0,thz=0;

  // Constructor
  manualMoveit() : Node("manual_move", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(), "Constructor called. MoveGroupInterface will be initialized in the initialize() method.");

  }
 
  // This method initializes components that depend on shared_from_this()
  void initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing action server and MoveGroupInterface...");
    
    // Create the MoveGroupInterface instance only once
    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    
    action_server_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "manual_move",
      std::bind(&manualMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&manualMoveit::handle_cancel, this, std::placeholders::_1),
      std::bind(&manualMoveit::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Initialization complete.");

    init = true;
    
  }

  void turn(){
   while (init) {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot state to be received...");
        rclcpp::sleep_for(std::chrono::seconds(5));
        
        // Get the current pose of the end effector.
        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;

        // Create a vector of poses to serve as waypoints for the Cartesian path.
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Create the new target pose. The position remains the same as the current pose.
        geometry_msgs::msg::Pose target_pose = current_pose;
        geometry_msgs::msg::Pose goal_pose = current_pose;

        RCLCPP_INFO(this->get_logger(), "Arm at: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",
                    current_pose.position.x, current_pose.position.y, current_pose.position.z,
                    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                    current_pose.orientation.w);

        double tx,ty,tz,tqx,tqy,tqz;

        // Prompt for user input
        // Manual control
        std::cout << "Give translation for x mm:";
        std::cin >> tx;
        std::cout << std::endl << "Give translation for y mm:";
        std::cin >> ty;
        std::cout << std::endl << "Give translation for z mm:";
        std::cin >> tz;
        std::cout << std::endl << "Give rotation for x euler deg:";
        std::cin >> tqx;
        std::cout << std::endl << "Give rotation for y euler deg:";
        std::cin >> tqy;
        std::cout << std::endl << "Give rotation for z euler deg:";
        std::cin >> tqz;

        if(tx == -100){
          tx = 0.0;
          tf2::Quaternion q_x_axis_parallel;
          q_x_axis_parallel.setRPY(0, 0, M_PI); // Roll=0, Pitch=-90deg, Yaw=0 (in radians)
          target_pose.orientation = tf2::toMsg(q_x_axis_parallel);
          waypoints.push_back(target_pose);
        }else{
          // Convert degrees to radians
          double roll_rad = tqx * M_PI / 180.0;
          double pitch_rad = tqy * M_PI / 180.0;
          double yaw_rad = tqz * M_PI / 180.0;

          thx = yaw_rad;
          thy = roll_rad;
          thz = pitch_rad;

          RCLCPP_INFO(this->get_logger(), "R=%.2f, P=%.2f, Y=%.2f degrees",thy, thz, thx);

          // RCLCPP_INFO(this->get_logger(), "Attempting to rotate by: R=%.2f, P=%.2f, Y=%.2f degrees",
          //             roll_deg, pitch_deg, yaw_deg);

          // Create a quaternion from the current end effector orientation.
          tf2::Quaternion current_quat;
          // Create a new quaternion from the user-provided RPY values.
          tf2::Quaternion relative_quat;
          // Create a quanternion for the final result
          tf2::Quaternion target_quat;


          // First orient by Z
          tf2::fromMsg(current_pose.orientation, current_quat);
          relative_quat.setRPY(0.0, 0.0, thx);
          // Multiply the current quaternion by the relative one to get the new absolute orientation.
          target_quat = current_quat * relative_quat;
          target_quat.normalize();
          target_pose.orientation = tf2::toMsg(target_quat);


          // Then orient by X
          tf2::fromMsg(target_pose.orientation, current_quat);
          relative_quat.setRPY(thy, 0.0, 0.0);
          // Multiply the current quaternion by the relative one to get the new absolute orientation.
          target_quat = current_quat * relative_quat;
          target_quat.normalize();
          target_pose.orientation = tf2::toMsg(target_quat);


          // Then orient by Y
          tf2::fromMsg(target_pose.orientation, current_quat);
          relative_quat.setRPY(0.0, thz, 0.0);
          // Multiply the current quaternion by the relative one to get the new absolute orientation.
          target_quat = current_quat * relative_quat;
          target_quat.normalize();
          target_pose.orientation = tf2::toMsg(target_quat);
          goal_pose.orientation = tf2::toMsg(target_quat);

          // goal_pose.position.x += tx*(cos(thx)*cos(thz)-sin(thx)*sin(thy)*sin(thz)) - ty*cos(thy)*sin(thx) + tz*(cos(thx)*sin(thz)+cos(thz)*sin(thx)*sin(thy));
          // goal_pose.position.y += tx*(cos(thz)*sin(thx)+cos(thx)*sin(thy)*sin(thz)) + ty*cos(thx)*cos(thy) + tz*(sin(thx)*sin(thz)-cos(thx)*cos(thz)*sin(thy));
          // goal_pose.position.z += -tx*cos(thy)*sin(thz) + ty*sin(thy) + tz*cos(thy)*cos(thz);

          // goal_pose.position.x += tx*(cos(thx)*cos(thz)-sin(thx)*sin(thy)*sin(thz)) - ty*cos(thy)*sin(thx) + tz*(cos(thx)*sin(thz)+cos(thz)*sin(thx)*sin(thy));
          // goal_pose.position.y += tx*(cos(thz)*sin(thx)+cos(thx)*sin(thy)*sin(thz)) + ty*cos(thx)*cos(thy) + tz*(sin(thx)*sin(thz)-cos(thx)*cos(thz)*sin(thy));
          // goal_pose.position.z += -tx*cos(thy)*sin(thz) + ty*sin(thy) + tz*cos(thy)*cos(thz);

          // 1. Convert tempX/Y/Z into a tf2::Vector3
          tf2::Vector3 relative_translation(tx, ty, tz);

          // 2. Rotate the translation vector by the current quaternion
          tf2::Vector3 global_translation = tf2::quatRotate(target_quat, relative_translation);

          // 3. Apply the global translation
          goal_pose.position.x += global_translation.x();
          goal_pose.position.y += global_translation.y();
          goal_pose.position.z += global_translation.z();

          RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
          

          waypoints.push_back(current_pose);
          waypoints.push_back(target_pose); // The target pose is our only waypoint.
          waypoints.push_back(goal_pose);

        }

        // --- START CARTESIAN PLANNING ---

        // Set up the trajectory plan object.
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Compute the Cartesian path.
        // The first parameter is the vector of waypoints.
        // The second parameter is the eef_step (end-effector step) in meters.
        // The third parameter is the jump_threshold.
        double fraction = move_group_interface_->computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_);

        // Check if the entire path was planned successfully.
        if (fraction >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "Cartesian planning successful. Executing...");
            // Execute the planned motion.
            move_group_interface_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Execution finished.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian planning failed. %.2f of the path was planned.", fraction);
        }
        // --- END CARTESIAN PLANNING ---

        // Pause to give the user time to read the output.
        std::cout << "Press Enter to continue..." << std::endl;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }
  }

private:
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  // This is the single, shared instance of MoveGroupInterface.
  std::shared_ptr<MoveGroupInterface> move_group_interface_;
  
  double x, y, z, qx, qy, qz, qw, dx;
 
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MoveToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received move goal");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
 
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Canceling goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
 
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    // The action server already runs this in a new thread. No need for an additional thread here.
    std::thread{std::bind(&manualMoveit::execute, this, goal_handle)}.detach();
  }
 
  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto result = std::make_shared<MoveToPose::Result>();
    auto goal = goal_handle->get_goal();
 
    geometry_msgs::msg::Pose start_pose = move_group_interface_->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose1 = start_pose;
    geometry_msgs::msg::Pose goal_pose = start_pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal->x, goal->y, goal->z, goal->qx, goal->qy, goal->qz, goal->qw);
    RCLCPP_INFO(this->get_logger(), "Arm at: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",start_pose.position.x,start_pose.position.y,start_pose.position.z,start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w);



    // if(goal->qx != 0.0){
    //   std::cin >> qx;
    //   tf2::Quaternion q_x_axis_parallel;
    //   q_x_axis_parallel.setRPY(0, qx, 0); // Roll=0, Pitch=-90deg, Yaw=0 (in radians)
    //   goal_pose.orientation = tf2::toMsg(q_x_axis_parallel);
    //   waypoints.push_back(goal_pose);

    // // Create a quaternion from the current end effector orientation.
    // tf2::Quaternion current_quat;
    // tf2::fromMsg(start_pose.orientation, current_quat);

    // // Create a new quaternion from the user-provided RPY values.
    // tf2::Quaternion relative_quat;
    // relative_quat.setRPY(goal->qx, goal->qy, goal->qz);

    // // Multiply the current quaternion by the relative one to get the new absolute orientation.
    // tf2::Quaternion target_quat = current_quat * relative_quat;
    // target_quat.normalize();

    // target_pose1.orientation = tf2::toMsg(target_quat);
    // goal_pose.orientation = tf2::toMsg(target_quat);

    
    dx = goal->x - start_pose.position.x;
    // x = (2*dx)/5;
    x = dx/2;

    RCLCPP_INFO(this->get_logger(), "approach_index = %d",goal->approach_index);

    if(goal->approach_index == 0){


      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      tf2::fromMsg(start_pose.orientation, current_quat);

      // Create a new quaternion from the user-provided RPY values.
      tf2::Quaternion relative_quat;
      relative_quat.setRPY(0.0, 0.0, goal->qz);

      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      tf2::Quaternion target_quat = current_quat * relative_quat;
      target_quat.normalize();

      target_pose1.orientation = tf2::toMsg(target_quat);
      goal_pose.orientation = tf2::toMsg(target_quat);

      // First half x
      target_pose1.position.x = x;

      // Then allign at y and z
      goal_pose.position.x = x;
      goal_pose.position.y += goal->y;
      goal_pose.position.z += goal->z;
    
      RCLCPP_INFO(this->get_logger(), "Computed target_pose1: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",target_pose1.position.x,target_pose1.position.y,target_pose1.position.z,target_pose1.orientation.x,target_pose1.orientation.y,target_pose1.orientation.z,target_pose1.orientation.w);
      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);

      // waypoints.push_back(start_pose);
      waypoints.push_back(target_pose1);
      waypoints.push_back(goal_pose);

    }else if(goal->approach_index == 1){

      // Allign y and z
      goal_pose.position.y += goal->y;
      goal_pose.position.z += goal->z;

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
 
      // waypoints.push_back(start_pose);
      waypoints.push_back(goal_pose);
    }else if(goal->approach_index == 2){

      // Create a quaternion from the current end effector orientation.
      tf2::Quaternion current_quat;
      tf2::fromMsg(start_pose.orientation, current_quat);

      // Create a new quaternion from the user-provided RPY values.
      tf2::Quaternion relative_quat;
      relative_quat.setRPY(goal->qx, 0.0, 0.0);

      // Multiply the current quaternion by the relative one to get the new absolute orientation.
      tf2::Quaternion target_quat = current_quat * relative_quat;
      target_quat.normalize();
      goal_pose.orientation = tf2::toMsg(target_quat);

      // init = true;
      // goal_pose.position.x += goal->x + 0.2;

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
      
      // double tx,ty,tz,tqx,tqy,tqz;

      // // Manual control
      // std::cout << "Give translation for x mm:";
      // std::cin >> tx;
      // std::cout << std::endl << "Give translation for y mm:";
      // std::cin >> ty;
      // std::cout << std::endl << "Give translation for z mm:";
      // std::cin >> tz;
      // std::cout << std::endl << "Give rotation for x euler deg:";
      // std::cin >> tqx;
      // std::cout << std::endl << "Give rotation for y euler deg:";
      // std::cin >> tqy;
      // std::cout << std::endl << "Give rotation for z euler deg:";
      // std::cin >> tqz;

      // tqx = tqx * M_PI / 180.0;
      // tqy = tqy * M_PI / 180.0;
      // tqz = tqz * M_PI / 180.0;


      // // Create a quaternion from the current end effector orientation.
      // tf2::Quaternion current_quat;
      // tf2::fromMsg(start_pose.orientation, current_quat);

      // // Create a new quaternion from the user-provided RPY values.
      // tf2::Quaternion relative_quat;
      // relative_quat.setRPY(tqx, tqy, tqz);

      // // Multiply the current quaternion by the relative one to get the new absolute orientation.
      // tf2::Quaternion target_quat = current_quat * relative_quat;
      // target_quat.normalize();
      // goal_pose.orientation = tf2::toMsg(target_quat);

      // goal_pose.position.x += tx;
      // goal_pose.position.y += ty;
      // goal_pose.position.z += tz;

      // RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
      
      // waypoints.push_back(start_pose);
      waypoints.push_back(goal_pose);
    }else{
      
      goal_pose.position.x += goal->x + 0.2;

      RCLCPP_INFO(this->get_logger(), "Computed goal_pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x,goal_pose.position.y,goal_pose.position.z,goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z,goal_pose.orientation.w);
      


      // waypoints.push_back(start_pose);
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


      return;
    }
 
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_result = move_group_interface_->execute(plan);


    RCLCPP_INFO(this->get_logger(), "Published end-effector pose to /movement/pose.");
    RCLCPP_INFO(this->get_logger(), "Pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);
 
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      result->success = true;
      result->message = "Execution succeeded";
      RCLCPP_INFO(this->get_logger(), "Execution succeeded!!!");
      goal_handle->succeed(result);


    } else {
      result->success = false;
      result->message = "Execution failed";
      RCLCPP_INFO(this->get_logger(), "Execution failed!!!");
      goal_handle->abort(result);

    }
  }

};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create a shared pointer to your node
  auto node = std::make_shared<manualMoveit>();
  
  // Call the initialize method after the shared pointer is created
  node->initialize();
  std::thread turn_thread(&manualMoveit::turn, node);
  turn_thread.detach();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
