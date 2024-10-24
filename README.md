# Descartes_ROS2 (like Descartes but now in ROS2)

## Description
Descartes is a path planner for semi-constrained (or not) Cartesian trajectories.  It is based on the graphs, determisitic and complete.

This repo offers Descartes migrated to ROS2 as well as additional functionalities with respect to the original library such as:
1. Partial trajectories are returned when the planner is unable to solve the whole path 
2. The poderation options have already been defined and can be activated/deactivated from the launch file. The weight to each ponderation is also modified from the launch file and it is very easy to add new cost functions.
3. Add/Remove collision objects to the planning scene. 
4. Planify in piece portée.
5. Automatic time stamp nased on the linear velocity desired. 

This library has been used in the [COGNIMAN](https://gitlab.irt-jules-verne.fr/ert_roc/projets/cogniman) and [SMARTROBOT](https://gitlab.irt-jules-verne.fr/ert_roc/projets/smart-robot) projects as part of a high level trajectory planner, [global_planner](https://gitlab.irt-jules-verne.fr/ert_roc/briques_techno/zero-programming/planning/globalplanner). There you may find some inspiration and examples for how to use it. 
## Working Principle
Descartes motion planning operates on the principle of planning paths through a Cartesian space while maintaining the feasibility of motion in the robot's configuration space.
1. Tolerated Poses: Tolerances can be added to the desired path, leading to a more unconstraint solutions. These are the acceptable variations of the end effector's position and orientation that still achieve the task's goal.
2. Configuration Space Sampling: For each tolerated pose, Descartes performs a sampling process in the configuration space. This involves calculating the possible joint configurations (or nodes) that could achieve each pose through inverse kinematics. Each valid configuration represents a node in the planning graph.
3. Graph Construction: Each nominal pose will correspond to one column of the graph (also called rungs in Descartes). This rungs will include all the valid joint solutions found for the nominal and tolerated poses of the path forming the nodes of the graph.
4. Edge Evaluation: Descartes then creates edges between these nodes based on the feasibility of moving from one configuration to another. This involves checking for collisions, joint limits, and other constraints. These other constraints are added through the Custom Cost Function.  
5. Graph Search: Once the graph is constructed, Descartes searches it to find the path from the start configuration to the goal configuration with the lowest cost. 

## How To Use
To use Descartes, the `robot_description`, `robot_description_semantic` and `robot_description_kinematics` have to be passed as parameters when launching the IKFastDescartesTrajPlanner program. The recomendation when creating these configuration files is to use [moveit_setup_assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) and to create the `movegroup` from _base_link_ to _link_6_t_ or _tool0_. The idea for this is that in the library, before passing the pose to the IK solver, a method finds the transform between the _ikfast_tool_frame_ (more about it a continuacion) and the last link in the movegroup and applies it to all the poses in the path. This way our movegroup does not have to include the ee and thus everything remains more generic. 

We also need to pass other parameters like : 

- `ikfast_base_frame` & `ikfast_tool_frame`: These are the initial and last link in your generated ikfast solver. When generating the IKFast solution, these correspond to the movegroup's first and last link. Keep in mind that ussually in the `moveit_setup_assistant` we work with joints, the only thing to keep in mind is that if your first joint in the movegroup is _joint0_, that connects _base_link_ to _link1_, then _base_link_ is `ikfast_base_frame`.

- `robot_description_name`: The name of the robot set on the robot_description passed in the parameters. 
*"group_name": The name of the group you wish to planify for and that is set on the robot_description_semantic passed in the parameters.

- `world_frame`: The frame of reference with respect to what you want to work. 

- `tcp_frame`: The target frame you wish to follow the path.

- `costFactorCSpaceDistance` : Weight based on the angle change of every joint. 

- `costFactorDistanceLimits` : Weight based on how far the joint are from there joint limits.

- `costFactorManipulability` : Weigh based on the manipulability factor for a joint configuration. 

- `costFactorNearestCollision` : Weight based on the collision distance to objects in the planning scene. 

- `costFactorCostChangePose` : Weight based on the change of orientation of the tcp. 

By default if no weights are set the joint change distance is used to evaluate the most optimal trajectory. 

- `pure_rotation_rad_per_sec`: When it is pure rotation, the liner velocity method implemented cannot work and thus this parameter allows to  modify the default one (0.5 rads/sec). 

- `jointVelocityLimitFactor` : Factor to reduce the maximum joint velocity limit accepted when calculating the joint solutions. If set to 0.5 for example, the max joint velocity limit would be halved and any change in joint configuration that exeeds this reduced velocity would be discarded (lowering it allows for a faster solution, however there is a risk of not finding the most optimal solution or even not finding one). 

## The DescartesTrajPlanner srv

The are quite a lot of request srv fields that can be populated to parametrize the trajectory generation process in Descartes. The fields are the following:
### The toolpath - Mandatory
- geometry_msgs/Pose[] trajectory_poses
List of Pose(s) representing the desired toolpath to be followed by the tcp. The size of the vector must be > 2.

### The position and orientation offsets allowed - Optional, default 0
- float64 orientation_offset_x 
- float64 orientation_offset_y 
- float64 orientation_offset_z 
- float64 position_offset_x 
- float64 position_offset_y 
- float64 position_offset_z 

These offsets represent the allowable deviation in the tool's orientation and position from the Poses that are passed in the field trajectory_poses. These offsets are symetric so for example if the orientation_offset_z is 90, then the allowable tolerated poses 
would include anything from orientation_z - 90 to orientation_z + 90. In terms of the trajectory, it means that all the poses that are 90° in z clockwise and anticlockwise will also be explored as an allowable solution. 

### The discretization steps - Optional, default 0
- int32 position_increment
- int32 orientation_increment 

The increment (in degrees for the orientation) that Descartes will use to discretize the toleranced frames. This determines the granularity of the possible orientations and positions within the specified tolerance.

### The time from start - Optional, default 0.2 sec
- float64 time_from_start 

The way the time stamps are calculated given a linear velocity impedes the calculation for the first point in the toolpath (since to calculate
it we find the eucledian distance between the points in the path n-1 and n) and thus, it must be set manually. By default it is 0.2 sec.
If the first pose in the path is the current pose of the robot, then there wouldn't be an issue to set it to 0.0. This is just so that when
trying to execute the trajectory, if at time 0 the configuration of the joints is very different, we risk damaging the robot since it will 
have to experience really high accelerations (although at least in the yaskawa you cannot execute a traj that the first joint config is not
the current state of the robot). 

### The link to plan to (the tcp link) - Optional if un-set it will be the tcp_name in the parameters
- string tcp_name 
In principle the tcp is set as a launch parameter in the server node and modifying it at runtime does not update it in the current planning that has already been launched.  This variable allows to modify it between plans.

### The linear velocity to mantain along the toolpath - Mandatory 
- float64 velocity
The desired velocity of the robot in meters per second.
 
### Initial and final configurations - Optional  

- float64[] initial_configuration
- float64 time_to_init_config  
The robot's joint configuration at the start of the trajectory an the time stamped at which you want to be in that joint pose. 
Folllows the same reasoning as before for time_from_start. 

- float64[] final_configuration
- float64 time_to_final_config

The robot's desired joint configuration at the end of the trajectory.


## IkFast Plugin
### To generate the IkFast solution
[Link to the documentation](https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html)
The command in the documentation are for ROS1, for ROS2 would be: 
ros2 run moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D $MYROBOT_NAME.urdf <planning_group_name> <base_link> <eef_link>

* Notice that what we put are the links not the joints
* It is probable that when calculating the solutions of the IkFast solver, it will fail and the error will be related to a mismatch of python version. For that just go to the installed ROS libraries (in ubuntu and using apt the default is `/opt/ros/humble/`) go to the `/lib/create_ikfast_moveit_plugin.py` and change the shebang at the top : `#! /usr/bin/env python` to `#! /usr/bin/env python3`. Then re execute the previous command. 

### To keep in mind
The ikfast plugin solver must go from the first link to the last link in your movegroup. These are the names that you must specify as `ikfast_base_frame` and `ikfast_tool_frame` in the Descartes parameters. If the solver goes for example to _link_t_6_ but the last link in the movegroup is the _tcp_, the solver will automatically try to find the target pose for the _tcp_ not the _link_t_6_ (as one would expect).

The solver finds the transformation between the last link in the movegroup and the `ikfast_tool_frame`, applies it to the target poses and finally inserts the transformed poses into the solver (method `transformToChainFrame` line 1381 in 'robotName_groupName_moveit_plugin.cpp'). 

Although not a completly bad behavior, it limits the flexibility of descartes, where you specify as a parameter the `tcp_frame` and it does the same transformation from the `tcp_frame` to the `ikfast_tool_frame` as the solver does (in 'ikfast_moveit_state_adapter.cpp':  `Eigen::Isometry3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;`). The difference being that the tcp_frame is easily set and modified in the launch file, decoupled from the moveit config files. 

If for some reason when planning the trajectory there is a constant offset between the target poses and your `tcp_frame` or the transform is completly wrong, verify that there is not a double transform being applied, one by descartes and the same one by the solver. Also, the same thing happens for the `ikfast_base_frame`, make sure that it is set to the first link of the movegroup or a constant offset will be present in all the solutions.

That is why the recomendation is to have the movegroup from _base_link_ to _link_6_t_ or _tool0_, setting the `ikfast_base_frame` to _base_link_ and the `ikfast_tool_frame` to _link_t_6_ or _tool0_ (or the appropiate name) and then the `tcp_frame` to the planning link you want to be at the desired pose. 

## Troubleshooting
If the user tries to planify a trajectory and gets an error msg on the terminal saying that no joint solutions where found for any of the points in the toolpath, chances are that there is a link in the environment, usually a fix link,that is in "collision" and thus when the planner performs the collision checking it will always fail. To check quickly if that is the issue, in _descartes_server/IKFastDescartesTrajPlanner.cpp_ line 35 set model->setCheckCollisions(false); 

This behavior of checking all the collisions vs checking the collisions of only the robot arm can be changed in descartes_moveit/src/moveit_state_adapter.cpp line 316. It would be a matter of uncomenting the line where isStateColliding takes the movegroup name as an argument: 
```
bool MoveitStateAdapter::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  bool in_singularity = false;

  if (check_collisions_)
  {
    moveit::core::RobotState state (robot_model_ptr_);
    state.setToDefaultValues();
    state.setJointGroupPositions(joint_group_, joint_pose);
    state.update();
    // in_collision = planning_scene_->isStateColliding(state, group_name_);
    in_collision = planning_scene_->isStateColliding(state);
    // in_singularity = isInSingularity(state, joint_group_);
  }

  // return in_collision || in_singularity;
  return in_collision;

}
```
The reason why I modified it is that our movegroup might not have all the links that are actually mounted on the robot so, for example, if we have a gripper mounted but the movegroup goes to _tool0_ (as i recomended for the IKFast plugin) then the collisions of the gripper will not be calculated and bad bad things may happen. So this is a disclaimer, if you change it I am not responsible of future collisions.  
## To keep in mind
The library uses OMP for parallel computing of both the IK solutions and the edge evaluation. It is important to correctly have it installed since for every pose in the path it tries to find 10 IK solutions (this value can be modified TODO:say where). In a path with 1000 poses, if we unconstrain the rotation around Z with an increment of one, the number of IK that it will try to find is 1000x360x10 = 3,600,000. The big computing issue really comes with the edge evaluation, since the rung i is evaluated agaings all the nodes in rung i + 1, leading to: nodes_in_rung_i ^nodes_in_rung_i_+_1. And this is repeated a 1000 - 1 times. 

## Limitations and Future Work
There exist Descartes Light, a repository that uses BGL to build and explore the graph. It is only for graph building and exploration. The creation of the Tolerated Poses is not included. Will try to explore in the future how to integrate it since the BGL allows for an easy manipulation of the graph as well as being memory efficient.   

OMP have it activated
Also limitations in computing time
and space? see the thesis paper



