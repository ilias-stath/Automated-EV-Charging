#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ur_simulation_gz/action/move_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <optional>
#include <thread>
#include <chrono>

// NEW: Include the header for the action cancellation service request type.
// This header defines the necessary message structure for the cancellation request.
#include "action_msgs/srv/cancel_goal.hpp"

using namespace std;
 
class MainLogic : public rclcpp::Node {
public:
  using MoveToPose = ur_simulation_gz::action::MoveToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;
  // FIX: Directly use the canonical type for the cancellation service request:
  // action_msgs::srv::CancelGoal::Request. This resolves the 'does not name a type' error.
  using CancelGoalRequest = action_msgs::srv::CancelGoal::Request;
 
  MainLogic() : Node("main_logic") {
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to action server");
    client_ = rclcpp_action::create_client<MoveToPose>(this, "move_moveit");
 
    RCLCPP_INFO(this->get_logger(), "Subscribing to /movement/status");
    move_status_sub = this->create_subscription<std_msgs::msg::String>("/movement/status", 10,bind(&MainLogic::status_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Publishing to /movement/status");
    move_status_pub = this->create_publisher<std_msgs::msg::String>("/movement/status",10);
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to /movement/goal");
    read_goal = this->create_subscription<geometry_msgs::msg::Pose>("/movement/goal",10,std::bind(&MainLogic::goal_callback,this,std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to /movement/pose");
    read_pose = this->create_subscription<geometry_msgs::msg::Pose>("/movement/pose",2,std::bind(&MainLogic::pose_callback,this,std::placeholders::_1));

    // NEW: Subscriber for cancellation command
    RCLCPP_INFO(this->get_logger(), "Subscribing to cancellation command: /movement/cancel_command");
    cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/movement/cancel_command", 10,
        std::bind(&MainLogic::cancel_command_callback, this, std::placeholders::_1));

    // Main thread
    RCLCPP_INFO(this->get_logger(), "Starting main thread...");
    main_timer = this->create_wall_timer(
        chrono::seconds(1), 
        bind(&MainLogic::main_logic, this)
    );
 
    // Next timer.
    RCLCPP_INFO(this->get_logger(), "Starting next timer...");
    next_timer = this->create_wall_timer(
        chrono::seconds(5), 
        bind(&MainLogic::next_timeout, this)
    );
    // The timer starts when "next" is published.
    this->next_timer->cancel();

    // Original pose timer
    RCLCPP_INFO(this->get_logger(), "Starting original pose timer...");
    orig_pose_timer = this->create_wall_timer(
        chrono::seconds(4), 
        bind(&MainLogic::orig_pose_timeout, this)
    );
    // The timer starts when "next" is published.
    orig_pose_timer->cancel();


    // Using timer to avoid potential conflicts and non initialization of the subscriber
    RCLCPP_INFO(this->get_logger(), "Starting the process by sending next..");
    start_process = this->create_wall_timer(
      chrono::seconds(1),
      [this](){
        send_next = true;
        auto init_msg = std_msgs::msg::String();
        init_msg.data = "next";
        move_status_pub->publish(init_msg);
        RCLCPP_INFO(this->get_logger(), "Initial 'next' message published. Starting next timer.");
        this->next_timer->reset(); // Start the Next timer
        init_msg.data = "pose";
        move_status_pub->publish(init_msg);
        RCLCPP_INFO(this->get_logger(), "Initial 'pose' message published. Starting original pose timer.");
        this->orig_pose_timer->reset();
        start_process->cancel(); // run only once
      }
    );

    
  }
 
private:
  rclcpp_action::Client<MoveToPose>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr move_status_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_status_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr read_goal;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr read_pose;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub_; // Cancellation subscriber
  
  rclcpp::TimerBase::SharedPtr start_process;
  rclcpp::TimerBase::SharedPtr next_timer; // Timer for goal Next
  rclcpp::TimerBase::SharedPtr main_timer; // Main timer
  rclcpp::TimerBase::SharedPtr orig_pose_timer; // Timer for original pose
  
  // Member to store the active goal handle for cancellation
  std::shared_ptr<GoalHandle> active_goal_handle_; 

  vector<geometry_msgs::msg::Pose> poses;
  vector<geometry_msgs::msg::Pose> prev_poses;
  geometry_msgs::msg::Pose goal_pose;
  geometry_msgs::msg::Pose my_pose;
  geometry_msgs::msg::Pose start_pose;
  std::optional<geometry_msgs::msg::Pose> inlet_position;
 
  bool send_next = false, skip=false, goal_sent=false, orig_pose=true, first=true, end=false;
  uint8_t approach_index = 0,goal_index=0;
  double dist=10.0,temp_dist=0.0;

  void main_logic(){

    dist = 10.0;
    temp_dist = 0.0;

    if(goal_sent && !orig_pose){
      auto goal_msg = MoveToPose::Goal();

      goal_sent = false;
      
      if(approach_index < 5){

        end = false;

        if(!inlet_position.has_value()){
          for (uint8_t i = 0; i < poses.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "Inlet position relative to me: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w);
            temp_dist = calculate_distance_tf2(my_pose.position, poses[i].position);
            RCLCPP_INFO(this->get_logger(), "Inlet distance from end-effector: %f m",temp_dist);
            
            if(temp_dist < dist){
              goal_index = i;
              dist = temp_dist;
              RCLCPP_INFO(this->get_logger(), "Currently selected this inlet position.");
            }
          }
          inlet_position = poses[goal_index];
          inlet_position->position.x += my_pose.position.x;
          inlet_position->position.y += my_pose.position.y;
          inlet_position->position.z += my_pose.position.z;
          inlet_position->orientation.x = 0.0;
          inlet_position->orientation.y = 0.0;
          inlet_position->orientation.z = 0.0;
          inlet_position->orientation.w = 0.0;
          RCLCPP_INFO(this->get_logger(), "Inlet position relative to base origin, saved at: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",inlet_position->position.x, inlet_position->position.y, inlet_position->position.z, inlet_position->orientation.x, inlet_position->orientation.y, inlet_position->orientation.z, inlet_position->orientation.w);
        }else{
          for (uint8_t i = 0; i < poses.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "Inlet position relative to me: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w);
            goal_pose.position.x = poses[i].position.x + my_pose.position.x;
            goal_pose.position.y = poses[i].position.y + my_pose.position.y;
            goal_pose.position.z = poses[i].position.z + my_pose.position.z;
            RCLCPP_INFO(this->get_logger(), "Inlet distance from base origin: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);
            
            temp_dist = calculate_distance_tf2(inlet_position->position, goal_pose.position);
            RCLCPP_INFO(this->get_logger(), "Inlet distance (respect to base origin) from save inlet position: %f m",temp_dist);
            
            if(temp_dist < dist){
              goal_index = i;
              dist = temp_dist;
              RCLCPP_INFO(this->get_logger(), "Currently selected this inlet position.");
            }
            
          }

        }

        goal_pose = poses[goal_index];

        send_next = false;
        skip = false;
        RCLCPP_INFO(this->get_logger(), "Selected pose as goal: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);

        if (!this->client_->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available");
          skip = true;
        }
        
        if (fabs(goal_pose.orientation.z) > 0.02){
          goal_pose.orientation.z = (fabs(goal_pose.orientation.z) < 0.02) ? 0.0 : goal_pose.orientation.z;

          inlet_position->orientation.z += goal_pose.orientation.z;
          
          RCLCPP_INFO(this->get_logger(), "In case with index 4 new inlet orientation: qx=%.4f, qy=%.4f, qz=%.4f",inlet_position->orientation.x,inlet_position->orientation.y,inlet_position->orientation.z);
          RCLCPP_INFO(this->get_logger(), "In case with index 4: qx=%.4f, qy=%.4f, qz=%.4f",goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z);

          approach_index = 3;
          goal_msg.approach_index = approach_index;
          if(first){
            approach_index = 0;
            first = false;
          }
        }
        else if (approach_index == 0){
          goal_pose.orientation.x = (fabs(goal_pose.orientation.x) < 0.02) ? 0.0 : goal_pose.orientation.x;
          goal_pose.orientation.y = (fabs(goal_pose.orientation.y) < 0.02) ? 0.0 : goal_pose.orientation.y;
          goal_pose.orientation.z = (fabs(goal_pose.orientation.z) < 0.02) ? 0.0 : goal_pose.orientation.z;

          inlet_position->orientation.x += goal_pose.orientation.x;
          inlet_position->orientation.y += goal_pose.orientation.y;
          RCLCPP_INFO(this->get_logger(), "In case with index 0: qx=%.4f, qy=%.4f, qz=%.4f",goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z);
          RCLCPP_INFO(this->get_logger(), "In case with index 0 new inlet orientation: qx=%.4f, qy=%.4f, qz=%.4f",inlet_position->orientation.x,inlet_position->orientation.y,inlet_position->orientation.z);

          goal_msg.approach_index = approach_index;
          approach_index = 1;
          first = false;
        }else {
          // RCLCPP_INFO(this->get_logger(), "y->%.4f  ,  z->%.4f",goal_pose.position.y,goal_pose.position.z);
          if ((fabs(goal_pose.position.y) > 0.008 || fabs(goal_pose.position.z) > 0.008)){
            approach_index = 1;
            goal_msg.approach_index = approach_index;
          }else if (fabs(goal_pose.orientation.x) > 0.02 || fabs(goal_pose.orientation.y) > 0.02){
            // Set values to 0 if there are less than 0.02
            goal_pose.orientation.x = (fabs(goal_pose.orientation.x) < 0.02) ? 0.0 : goal_pose.orientation.x;
            goal_pose.orientation.y = (fabs(goal_pose.orientation.y) < 0.02) ? 0.0 : goal_pose.orientation.y;
            goal_pose.orientation.z = (fabs(goal_pose.orientation.z) < 0.02) ? 0.0 : goal_pose.orientation.z;

            inlet_position->orientation.x += goal_pose.orientation.x;
            inlet_position->orientation.y += goal_pose.orientation.y;
            // inlet_position->orientation.z += goal_pose.orientation.z;

            RCLCPP_INFO(this->get_logger(), "In case with index 2: qx=%.4f, qy=%.4f, qz=%.4f",goal_pose.orientation.x,goal_pose.orientation.y,goal_pose.orientation.z);
            RCLCPP_INFO(this->get_logger(), "In case with index 2 new inlet orientation: qx=%.4f, qy=%.4f, qz=%.4f",inlet_position->orientation.x,inlet_position->orientation.y,inlet_position->orientation.z);

            approach_index = 2;
            goal_msg.approach_index = approach_index;
          }else{
            approach_index = 4;
            goal_msg.approach_index = approach_index;
            approach_index = 5;
          }
        }

        goal_msg.x = goal_pose.position.x;
        goal_msg.y = goal_pose.position.y;
        goal_msg.z = goal_pose.position.z;
        goal_msg.qx = goal_pose.orientation.x;
        goal_msg.qy = goal_pose.orientation.y;
        goal_msg.qz = goal_pose.orientation.z;
        goal_msg.qw = goal_pose.orientation.w;
        goal_msg.thx = inlet_position->orientation.x;
        goal_msg.thy = inlet_position->orientation.y;
        goal_msg.thz = inlet_position->orientation.z;
      }else if(approach_index == 5){
        RCLCPP_INFO(this->get_logger(), "Waiting for car to charge (10s)...");
        rclcpp::sleep_for(std::chrono::seconds(10));

        approach_index = 5;
        goal_msg.approach_index = approach_index;
        approach_index = 0;

        start_pose = prev_poses.front();

        goal_msg.x = start_pose.position.x;
        goal_msg.y = start_pose.position.y;
        goal_msg.z = start_pose.position.z;
        goal_msg.qx = start_pose.orientation.x;
        goal_msg.qy = start_pose.orientation.y;
        goal_msg.qz = start_pose.orientation.z;
        goal_msg.qw = start_pose.orientation.w;
        goal_msg.thx = 0.0;
        goal_msg.thy = 0.0;
        goal_msg.thz = 0.0;

        inlet_position.reset();
        prev_poses.clear();
        first=true;
        end = true;
      }else{
        RCLCPP_INFO(this->get_logger(), "Move canceled going to home...");
        rclcpp::sleep_for(std::chrono::seconds(2));

        approach_index = 6;
        goal_msg.approach_index = approach_index;
        approach_index = 0;

        start_pose = prev_poses.front();

        goal_msg.x = start_pose.position.x;
        goal_msg.y = start_pose.position.y;
        goal_msg.z = start_pose.position.z;
        goal_msg.qx = start_pose.orientation.x;
        goal_msg.qy = start_pose.orientation.y;
        goal_msg.qz = start_pose.orientation.z;
        goal_msg.qw = start_pose.orientation.w;
        goal_msg.thx = 0.0;
        goal_msg.thy = 0.0;
        goal_msg.thz = 0.0;

        inlet_position.reset();
        prev_poses.clear();
        first=true;
        end = true;
      }



      
      // RCLCPP_INFO(this->get_logger(), "x = %f  ,  y = %f  ,  z = %f",x,y,z);
      RCLCPP_INFO(this->get_logger(), "approach_index = %d",approach_index);
      if(!skip){
        RCLCPP_INFO(this->get_logger(), "Sending goal to action server");
        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        
        // Goal response callback to store the active goal handle
        send_goal_options.goal_response_callback = 
            [this](GoalHandle::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
                    this->active_goal_handle_.reset(); // Clear handle if rejected
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server. Storing handle for cancellation.");
                    this->active_goal_handle_ = goal_handle; // Store the handle
                }
            };
            
        send_goal_options.result_callback =
          [this](const GoalHandle::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_INFO(this->get_logger(), "Move succeeded: %s. Clearing stored goal poses", result.result->message.c_str());
              poses.clear();
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
              RCLCPP_WARN(this->get_logger(), "Move was CANCELED at the client's request or server's decision.");
            } else {
              RCLCPP_ERROR(this->get_logger(), "Move failed: %s.", result.result->message.c_str());
            }
            // Always clear the handle when the goal terminates (success, fail, or cancel)
            this->active_goal_handle_.reset(); 
          };
          
        client_->async_send_goal(goal_msg, send_goal_options);
        
        auto state_msg = std_msgs::msg::String();
        state_msg.data = "moving";
        move_status_pub->publish(state_msg);
      }else{
        skip = false;
        RCLCPP_INFO(this->get_logger(), "Skipping");
        auto state_msg = std_msgs::msg::String();
        state_msg.data = "next";
        this->next_timer->reset();
        move_status_pub->publish(state_msg);
        send_next = true;
        // Also start the timer again if we are skipping
      }
    }
  }

  void cancel_current_move() {
    // 1. Check if we have a valid goal handle stored
    if (active_goal_handle_) {
        // 2. Check the status of the goal before attempting cancellation
        if (active_goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || active_goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
            RCLCPP_INFO(this->get_logger(), "Sending cancel request for the active move goal...");

            // FIX: Instead of manually creating the CancelGoalRequest message, pass the 
            // active goal handle directly to async_cancel_goal. The rclcpp_action library 
            // handles packaging the goal ID automatically, solving both type conversion errors.
            auto future_cancel = client_->async_cancel_goal(active_goal_handle_);

            // 5. Clear the handle immediately after sending the request. The result callback will handle the final status.
            active_goal_handle_.reset();
            approach_index = 6;

        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot cancel: Goal is not in EXECUTING or ACCEPTED status (Current Status: %d).", active_goal_handle_->get_status());
            active_goal_handle_.reset(); // Clear handle if it's already terminated
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Cannot cancel: No active goal handle stored.");
    }
  }
   
  void cancel_command_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "CANCEL") {
      RCLCPP_WARN(this->get_logger(), "Received CANCEL command on /movement/cancel_command. Initiating move termination.");
      cancel_current_move();
    }
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg){

    if(!orig_pose && !end){
      RCLCPP_INFO(this->get_logger(), "Stored previous pose....");
      prev_poses.push_back(my_pose);
    }
    my_pose = *msg;

    if(orig_pose){
      orig_pose = false;
      RCLCPP_INFO(this->get_logger(), "Received current pose canceling original pose timer....");
      orig_pose_timer->cancel();
    }else{
      RCLCPP_INFO(this->get_logger(), "Received current pose....");
    }

    RCLCPP_INFO(this->get_logger(), "Current pose: x=%.4f, y=%.4f, z=%.4f, qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f",my_pose.position.x, my_pose.position.y, my_pose.position.z, my_pose.orientation.x, my_pose.orientation.y, my_pose.orientation.z, my_pose.orientation.w);


  }


  void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    bool new_pose = true;

    // if()

    if(poses.empty()){
      poses.push_back(*msg);
      RCLCPP_INFO(this->get_logger(), "Saved new goal pose: x=%.4f(m) , y=%.4f(m) , z=%.4f(m) , qx=%.4f(m) , qy=%.4f(m) , qz=%.4f(m) , qw=%.4f(m)",msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    }else{
      for (const auto& pose : poses) {
        if (almost_equal(pose.position.x, msg->position.x) && almost_equal(pose.position.y, msg->position.y) && almost_equal(pose.position.z, msg->position.z)) {
          new_pose = false;
          RCLCPP_INFO(this->get_logger(), "Goal pose \" x=%.4f(m) , y=%.4f(m) , z=%.4f(m) , qx=%.4f(m) , qy=%.4f(m) , qz=%.4f(m) , qw=%.4f(m) \" already exists. Not saving.",my_pose.position.x, my_pose.position.y, my_pose.position.z, my_pose.orientation.x, my_pose.orientation.y, my_pose.orientation.z, my_pose.orientation.w);
          break; 
        }
      }

      if (new_pose) {
        poses.push_back(*msg);
        RCLCPP_INFO(this->get_logger(), "Saved new goal pose: x=%.4f(m) , y=%.4f(m) , z=%.4f(m) , qx=%.4f(m) , qy=%.4f(m) , qz=%.4f(m) , qw=%.4f(m)",my_pose.position.x, my_pose.position.y, my_pose.position.z, my_pose.orientation.x, my_pose.orientation.y, my_pose.orientation.z, my_pose.orientation.w);
      }
    }
  }
 
  void status_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "done") {
      RCLCPP_INFO(this->get_logger(), "Robot finished move, waiting 2s...");
      
      // Use a new thread to avoid blocking the main ROS event loop
      if(approach_index != 5){
        std::thread([this]() {
          std::this_thread::sleep_for(std::chrono::seconds(2));
          auto msg_out = std_msgs::msg::String();
          msg_out.data = "next";
          // Reset and start the next timer for the new request
          this->next_timer->cancel();
          this->next_timer->reset();
          move_status_pub->publish(msg_out);
          send_next = true;
          RCLCPP_INFO(this->get_logger(), "Changed state to next... Starting next timer.");
        }).detach();
      }else{
        goal_sent = true;
      }
    }

    if (msg->data == "received"){
      RCLCPP_INFO(this->get_logger(), "Received status canceling next timer....");
      this->next_timer->cancel();
    }

    if (msg->data == "sent"){
      goal_sent = true;
    }

    // if (msg->data == "posing"){
    //  orig_pose = false;
    //  RCLCPP_INFO(this->get_logger(), "Received status canceling original pose timer....");
    //  orig_pose_timeout->cancel();
    // }
  
    RCLCPP_INFO(this->get_logger(), "(From state_callback) State is: %s\n\n\n", msg->data.c_str());
  }

  void next_timeout() {
    // If we are still waiting for a goal, re-publish the "next" message.
    if (send_next) {
        RCLCPP_WARN(this->get_logger(), "Timeout! No status received in 5 seconds. Re-publishing 'next' status.");
        auto msg_out = std_msgs::msg::String();
        msg_out.data = "next";
        // Restart the timer for another 3-second check
        this->next_timer->reset();
        move_status_pub->publish(msg_out);
    }
  }


  void orig_pose_timeout(){
    // If we are still waiting for original pose, re-publish the "pose" message.
    if (orig_pose) {
        RCLCPP_WARN(this->get_logger(), "Timeout! No status received in 4 seconds. Re-publishing 'pose' status.");
        auto msg_out = std_msgs::msg::String();
        msg_out.data = "pose";
        move_status_pub->publish(msg_out);
        // Restart the timer for another 4-second check
        this->orig_pose_timer->reset();
    }
  }

  bool almost_equal(double a, double b, double e = 0.005) {
    return abs(a - b) <= e;
  }

  double calculate_distance_tf2(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    // Convert geometry_msgs::msg::Point to tf2::Vector3
    tf2::Vector3 v1;
    tf2::fromMsg(p1, v1);

    tf2::Vector3 v2;
    tf2::fromMsg(p2, v2);

    // Use the built-in distance() method
    return v1.distance(v2);
  }


};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainLogic>());
  rclcpp::shutdown();
  return 0;
}
