import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from xacro import process_file

def generate_launch_description():
    # Load the xacro file and convert it to URDF
    robot_description_path = os.path.join(get_package_share_directory('cell_envir'), 'my_robot.urdf')
    xacro_path = os.path.join(get_package_share_directory('cell_envir'), 'urdf', 'fact_robot.xacro')
    doc = process_file(xacro_path)
    robot_description_config = doc.toprettyxml(indent='  ')
    with open(robot_description_path, 'w') as file:
        file.write(robot_description_config)

    # Launch the robot_state_publisher node with the robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Launch RViz2 with the robot model
    rviz_config_path = os.path.join(get_package_share_directory('cell_envir'), 'rviz', 'my_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node
    ])

