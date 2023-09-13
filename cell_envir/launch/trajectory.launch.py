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

    trajectory_node = Node(
        name = "trajectory_node",
        package="cell_envir",
        executable="simple_traject",
        output= "screen",
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        # remappings=[('joint_states', '/joint_states')]
    )

    

    return LaunchDescription(
        [
            trajectory_node
        ]
        
    )
