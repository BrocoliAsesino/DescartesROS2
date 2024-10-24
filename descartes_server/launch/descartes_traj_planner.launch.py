import os
import yaml
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

#========================================== TO MODIFY ===========================================

robot_model_config_pkg = "gp7_moveit_config"
config_dir_name = "config"
robot_name = "robot_description"

#===================================================== functions to generate paths =====================================================
def evaluate_substitution(context, substitution):
    return substitution.perform(context)
    
def load_file(absolute_file_path):
    # Check if the file exists
    if not os.path.exists(absolute_file_path):
        print(f"File does not exist: {absolute_file_path}")
        return NonePathJoinSubstituion

    print(f"Package path: {absolute_file_path}")

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError as e:  
        print(f"Error loading file: {str(e)}")
        return None



def load_yaml(absolute_file_path):
    print(f"Package path: {absolute_file_path}")

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:  
        print(f"Error loading YAML file: {str(e)}")  # Add this line to debug
        return None

# ===============================================================================================================================================================
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # ===================================================== Paths and Dictionaries =====================================================
    context = LaunchContext()

    robot_description_path = PathJoinSubstitution(
           [FindPackageShare(robot_model_config_pkg), config_dir_name, f"{robot_name}.urdf.xacro"])

    robot_description_config = xacro.process_file(evaluate_substitution(context, robot_description_path))

    robot_description = {"robot_description": robot_description_config.toxml()}

    # ================================
    robot_description_semantic_path = PathJoinSubstitution(
           [FindPackageShare(robot_model_config_pkg), config_dir_name, f"{robot_name}.srdf"])

    robot_description_semantic_config = load_file(evaluate_substitution(context, robot_description_semantic_path))
    
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # ================================
    kinematics_yaml_path = PathJoinSubstitution(
           [FindPackageShare(robot_model_config_pkg), config_dir_name, "kinematics.yaml"])

    kinematics_yaml = load_yaml(evaluate_substitution(context, kinematics_yaml_path))

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # ===================================================== Test Node =====================================================

    traj_planner_server_node = Node(
        package = "descartes_server",
        executable = "descartes_trajectory_planner_server",
        output = "screen",
        parameters=[robot_description, 
                    robot_description_semantic, 
                    robot_description_kinematics,
                    {"ikfast_base_frame": "base_link", 
                    ## ikfast_tool_frame & ikfast_base_frame are the initial and last link in your generated ikfast solver (very important)
                    "ikfast_tool_frame": "tool0",
                    "robot_description_name": "robot_description", 
                    "group_name": "torch_group",
                    "world_frame": "world", 
                    "tcp_frame": "tcp", 
                    "costFactorCSpaceDistance" : 1.0,
                    "costFactorDistanceLimits" : 0.0,
                    "costFactorManipulability" : 1.0,
                    "costFactorNearestCollision" : 0.0,
                    "costFactorCostChangePose" : 0.0,
                    "pure_rotation_rad_per_sec": 0.5,
                    "jointVelocityLimitFactor" : 1.0,} ## how close to joint velocity limits do we accept being, descartes will discard any joint solutions 
                    ## that involve a velocity higher than max_joint_velocity_robot*jointVelocityLimitFactor
                    ],
        arguments=['--ros-args', '--log-level', 'error']
    )



    return LaunchDescription(
        [
            # run_move_group_node,
            traj_planner_server_node
        ]
    )