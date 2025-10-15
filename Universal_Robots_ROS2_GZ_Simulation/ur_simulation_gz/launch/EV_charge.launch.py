import launch
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # 1. move_moveit node
    move_moveit_node = Node(
        package='ur_simulation_gz',
        executable='move_moveit',
        name='move_moveit',
        output='screen',
        parameters=[{"use_sim_time": True}],
    )

    # 2. yolo_model node
    yolo_node_node = Node(
        package='yolo_model',
        executable='yolo_node',
        name='yolo_node',
        output='screen'
    )

    # 3. main_logic node
    main_logic_node = Node(
        package='ur_simulation_gz',
        executable='main_logic',
        name='main_logic',
        output='screen'
    )

    # Launch order with delays
    delayed_move_moveit_node = TimerAction(period=10.0, actions=[move_moveit_node])
    delayed_main_logic = TimerAction(period=20.0, actions=[main_logic_node])  # 5s after move_moveit

    return LaunchDescription([
        yolo_node_node,
        delayed_move_moveit_node,
        delayed_main_logic
    ])
