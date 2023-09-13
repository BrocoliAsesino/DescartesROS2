import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch.event_handlers import OnProcessExit

#===================================================== load functions =====================================================

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  
        return None

# ===============================================================================================================================================================
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # ===================================================== Robot Model =====================================================
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("cell_envir"),
            "config",
            "my_robot_description.urdf.xacro",
        )
    )
    
    robot_description = {"robot_description": robot_description_config.toxml()}

    # with open("./src/cell_environment/config/setup_assist_fact_robot.urdf", "w") as f:
    #     f.write(robot_description["robot_description"])


    robot_description_semantic_config = load_file(
        "cell_envir", "config/my_robot_description.srdf"
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}


    kinematics_yaml = load_yaml(
        "cell_envir", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # ===================================================== Moveit2 Paths & Parameters =====================================================
    

    moveit_simple_controllers_yaml = load_yaml(
        "cell_envir", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "cell_envir", "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
   

    # ===================================================== Robot State Publisher =====================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}]
    )

     
    # ===================================================== RVIZ =====================================================
    
    rviz_config_file = (
        get_package_share_directory("cell_envir") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            ompl_planning_pipeline_config,
            robot_description_semantic,
            kinematics_yaml,
            
            {"use_sim_time": use_sim_time}
        ],
    )
    

    # ===================================================== ROS2 Controllers =====================================================

    ros2_controllers_path = os.path.join(
        get_package_share_directory("cell_envir"),
        "config",
        "ros2_controllers.yaml",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path,],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_arm_controller =  Node( 
        package="controller_manager", 
        executable="spawner", 
        arguments=["arm_group_controller"], 
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    ) 

    spawn_gripper_controller =  Node( 
        package="controller_manager", 
        executable="spawner", 
        arguments=["gripper_group_controller"], 
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    ) 

    spawn_joint_state_broadcaster =  Node( 
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster"], 
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    ) 

    
    # ===================================================== Moveit2 - MoveGroup Node =====================================================
   


    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[('joint_states', '/joint_states')]
    )

    # ===================================================== Other Executables =====================================================



    # ===================================================== Launch Description =====================================================

    return LaunchDescription(
        [
            robot_state_publisher,
            run_move_group_node,
            rviz_node,
            controller_manager_node,
            spawn_joint_state_broadcaster,
            spawn_gripper_controller,
            spawn_arm_controller,
        ]
        
    )