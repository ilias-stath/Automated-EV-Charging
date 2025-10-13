cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gz)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# For building actions
find_package(rosidl_default_generators REQUIRED)

# Generate the action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

# Export include dirs
ament_export_dependencies(rosidl_default_runtime)


add_executable(move src/move.cpp)
ament_target_dependencies(move rclcpp trajectory_msgs yaml-cpp rclcpp_action control_msgs)

add_executable(main_logic src/main_logic.cpp)
ament_target_dependencies(main_logic rclcpp rclcpp_action control_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(abs_move_moveit src/abs_move_moveit.cpp)
ament_target_dependencies(abs_move_moveit rclcpp moveit_ros_planning_interface control_msgs)

add_executable(move_moveit src/move_moveit.cpp)
ament_target_dependencies(move_moveit rclcpp moveit_ros_planning_interface tf2 tf2_geometry_msgs control_msgs)

add_executable(manual_move src/manual_move.cpp)
ament_target_dependencies(manual_move rclcpp moveit_ros_planning_interface tf2 tf2_geometry_msgs control_msgs)

target_link_libraries(move yaml-cpp)

# Default to off as starting gzsim doesn't shut down correctly at the moment
option(
  UR_SIM_INTEGRATION_TESTS
  "Run ur_simulation_gz integration tests"
  OFF
)

install(TARGETS
  move main_logic abs_move_moveit move_moveit manual_move
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch worlds models action
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)

  if(${UR_SIM_INTEGRATION_TESTS})
    add_launch_test(test/test_gz.py
      TIMEOUT
        180
    )
  endif()
endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(abs_move_moveit ${cpp_typesupport_target})
target_link_libraries(move_moveit ${cpp_typesupport_target})
target_link_libraries(manual_move ${cpp_typesupport_target})
target_link_libraries(main_logic ${cpp_typesupport_target})

ament_package()
