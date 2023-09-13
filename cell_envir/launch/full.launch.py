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

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # ===================================================== Robot Model =====================================================
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("cell_envir"),
            "new_config",
            "fact_robot.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}


    # ========================= delete later =========================
    new_robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("cell_envir"),
            "urdf",
            "fact_robot.xacro",
        )
    )
    new_robot_description = {"new_robot_description": new_robot_description_config.toxml()}
    with open("new_setup_assist_fact_robot.urdf", "w") as f:
        f.write(new_robot_description["new_robot_description"])

    # maybe

    robot_description_semantic_config = load_file(
        "cell_envir", "new_config/fact_robot.srdf"
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}


    kinematics_yaml = load_yaml(
        "cell_envir", "new_config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # ===================================================== Moveit2 Paths & Parameters =====================================================
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        "cell_envir", "new_config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "cell_envir", "new_config/another_moveit_controllers.yaml"
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
            "cell_envir", "new_config/joint_limits.yaml"
        )
    }

  
    # ===================================================== Gazebo =====================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
    
                ## IMPORTANT : this launches as well the ros2 controller manager 
    spawn_entity = Node(
                    package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_robot_description'],
                    output='screen',
                    # parameters=[{'use_sim_time': use_sim_time}]
    )


    spawn_mesh = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'piece', '-file', 'src/cell_environment/world/piece.sdf'],
        output='screen'
    )

   

    # ===================================================== Robot State Publisher =====================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description,
        {"use_sim_time": True},
        # {"publish_frequency": 50},          # Set the tf publish frequency to 50 Hz
        # {"static_publish_frequency": 100}    # Set the tf_static publish frequency to 10 Hz
        ]
    )

     
    # ===================================================== RVIZ =====================================================
 
    
    rviz_config_file = (
        get_package_share_directory("cell_envir") + "/rviz/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            
            {"use_sim_time": True}
        ],
    )
    

    # ===================================================== ROS2 Controllers =====================================================

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("cell_envir"),
    #     "new_config",
    #     "ros2_controllers.yaml",
    # )

    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2_controllers_path,],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    # )

    # spawn_joint_trajectory_controller =  Node( 
    #     package="controller_manager", 
    #     executable="spawner", 
    #     arguments=["arm_controller", "-c", "/controller_manager"], 
    #     output="screen", 
    #     parameters=[{'use_sim_time': use_sim_time}]
    # ) 

    # spawn_gripper_controller =  Node( 
    #     package="controller_manager", 
    #     executable="spawner", 
    #     arguments=["hand_controller", "-c", "/controller_manager"], 
    #     output="screen", 
    #     parameters=[{'use_sim_time': use_sim_time}]
    # ) 

    # spawn_joint_state_broadcaster =  Node( 
    #     package="controller_manager", 
    #     executable="spawner", 
    #     arguments=["joint_state_broadcaster", "--controller-manager",
    #         "/controller_manager",], 
    #     output="screen", 
    #     parameters=[{'use_sim_time': use_sim_time}]
    # ) 

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hand_controller'],
        output='screen'
    )

    # ===================================================== Moveit2 - MoveGroup Node =====================================================

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    # ===================================================== Other Executables =====================================================

    static_tf_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.30", "0.0", "0.0", "-3.1416", "-1.5708", "1.5708", "link_6_t", "camera_link_optical"],
    )

    pointcloud_transform = Node(
        name = "transform_point_cloud_node",
        package= "cell_envir",
        executable= "transform_point_cloud",
        output="screen",

    )

    # ===================================================== Launch Description =====================================================

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            spawn_entity,
            spawn_mesh, 
            # controller_manager_node,
            load_joint_state_broadcaster,
            load_arm_controller,
            load_hand_controller,

            RegisterEventHandler(
            OnProcessExit(
                target_action = load_hand_controller,
                on_exit = [
                    TimerAction(
                        period=1.0,
                        actions=[
                            run_move_group_node,
                            rviz_node,
                            ]
                        ),
                    ]
                )
            )

        ]
        
    )