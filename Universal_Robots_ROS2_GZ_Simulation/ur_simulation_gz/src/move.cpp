#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class JointActionClient : public rclcpp::Node {
public:
    JointActionClient(const std::string &yaml_path, const std::string &goal_name)
        : Node("joint_action_client") {

        // Load YAML
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root[goal_name]) {
            RCLCPP_ERROR(this->get_logger(), "-----Goal '%s' not found in YAML file-----", goal_name.c_str());
            rclcpp::shutdown();
            return;
        }else{
            RCLCPP_INFO(this->get_logger(), "-----Goal found in YAML file-----");
        }

        auto goal = root[goal_name];

        // Create goal message
        FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = goal["joint_names"].as<std::vector<std::string>>();

        for (const auto &pt : goal["points"]) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = pt["positions"].as<std::vector<double>>();
            double secs = pt["time_from_start"].as<double>();
            point.time_from_start = rclcpp::Duration::from_seconds(secs);
            goal_msg.trajectory.points.push_back(point);
        }

        // Action client
        client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "-----Action server not available.-----");
            rclcpp::shutdown();
            return;
        }

        // Send goal and wait for result
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandle::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "-----Goal reached successfully!-----");
            } else {
                RCLCPP_ERROR(this->get_logger(), "-----Goal failed or was canceled.-----");
            }
            rclcpp::shutdown();  // exit when done
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        cerr << "Usage: move <goal_name>\n";
        return 1;
    }

    string goal_name = argv[1];
    string package_share_dir = ament_index_cpp::get_package_share_directory("ur_simulation_gz");
    string yaml_file = package_share_dir + "/config/positions.yaml";

    auto node = std::make_shared<JointActionClient>(yaml_file, goal_name);
    rclcpp::spin(node);

    return 0;
}
