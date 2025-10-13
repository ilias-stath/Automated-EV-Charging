import launch
from launch_ros.actions import Node

def generate_launch_description():
    # 1. ur_simulation_gz main_logic node
    main_logic_node = Node(
        package='ur_simulation_gz',
        executable='main_logic',
        name='main_logic_node',
        output='screen'
    )

    # 2. ur_simulation_gz move_moveit node
    move_moveit_node = Node(
        package='ur_simulation_gz',
        executable='move_moveit',
        name='move_moveit_node',
        output='screen'
    )

    # 3. yolo_model yolo_node
    yolo_node = Node(
        package='yolo_model',
        executable='yolo_node',
        name='yolo_detection_node',
        output='screen'
    )

    return launch.LaunchDescription([
        move_moveit_node,
        yolo_node,
        main_logic_node
    ])
