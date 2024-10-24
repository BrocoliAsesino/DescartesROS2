/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Jonathan Meyer
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "descartes_moveit/ikfast_moveit_state_adapter.h"

// #include <eigen_conversions/eigen_msg.h>
// #include <ros/node_handle.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <math.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer_client.h>



const static std::string default_base_frame = "base_link";
const static std::string default_tool_frame = "link_6_t";

// Compute the 'joint distance' between two poses
static double distance(const std::vector<double>& a, const std::vector<double>& b)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
static size_t closestJointPose(const std::vector<double>& target, const std::vector<std::vector<double>>& candidates)
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

// ======================================================================================================================================
//   We keep this method since it has the same signature than the one in the base class RobotModel. Cannot remove since the class inherits
//   from MoveitStateAdapter that in turns inherits from RobotModel and in there the initialize method is pure virtual, removing it would 
//   make this class abstract.

bool descartes_moveit::IkFastMoveitStateAdapter::initialize(const std::string& /*robot_description*/,
                                                            const std::string& /*group_name*/,
                                                            const std::string& /*world_frame*/,
                                                            const std::string& /*tcp_frame*/)
{
  // if (!MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame))
  // {
  //   return false;
  // }

  // return computeIKFastTransforms();
  return true;
}

// ======================================================================================================================================
//   New initialize method to include the pointer to a rclcpp::Node so that the RobotModelLoader works. In Moveit2 the RobotModelLoader               
//   takes an extra parameter (rclcpp::Node) to load the robot model than it did in Moveit. Implemented in MoveitStateAdapter. 
//   Pointer is stored in a protected variable to be used by other methods (setState, it calls computeIKFastTransforms that needs the node 
//   to fetch the params)

bool descartes_moveit::IkFastMoveitStateAdapter::initialize(const rclcpp::Node::SharedPtr& node,
                                                            const std::string& robot_description,
                                                            const std::string& group_name,
                                                            const std::string& world_frame,
                                                            const std::string& tcp_frame)
{
  node_ = node; 
  
  if (!MoveitStateAdapter::initialize(node, robot_description, group_name, world_frame, tcp_frame))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter: Failed to initialize MoveitStateAdapter");
    return false;
  }
  std::cout << "\n" << "\n"
            << "######################################" << std::endl;
  RCLCPP_INFO(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter initialized!");
  return computeIKFastTransforms(node);
}
// ======================================================================================================================================

bool descartes_moveit::IkFastMoveitStateAdapter::getAllIK(const Eigen::Isometry3d& pose,
                                                          std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();
  
  if (!joint_group_)
  {
      RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"),"Joint group is null.");
      return false;
  }

  const auto& solver = joint_group_->getSolverInstance();
  if (!solver)
  {
      std::cout << "No IK solver associated with joint group: %" << std::endl;
      return false;
  }

  // std::cout << "The pose is =  \n" << pose.matrix() << std::endl;
  // std::cout << "World to base  is =  \n" << world_to_base_.frame.matrix() << std::endl;
  // std::cout << "ikfast_tool_frame to tool_frame_ is =  \n" << tool0_to_tip_.frame.matrix() << std::endl;
  
  // Transform input pose
  Eigen::Isometry3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;
  // Eigen::Isometry3d tool_pose = world_to_base_.frame_inv * pose;

  // std::cout << "The tool_pose  is =  \n" << tool_pose.matrix() << std::endl;

  // convert to geometry_msgs ...
  geometry_msgs::msg::Pose geometry_pose;
  geometry_pose = tf2::toMsg(tool_pose);
  // tf2::eigenToMsg(tool_pose, geometry_pose);
  std::vector<geometry_msgs::msg::Pose> poses = { geometry_pose };

  std::vector<double> dummy_seed(getDOF(), 0.0);
  std::vector<std::vector<double>> joint_results;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;  // defaults are reasonable as of Indigo
  
  if (!solver->getPositionIK(poses, dummy_seed, joint_results, result, options))
  {

    return false;
  }
  
  const auto last_index = 5;
  const auto index4 = 3;

  for (auto& sol : joint_results)
  {

    const double joint6 = sol[last_index];
    const double joint4 = sol[index4];

    for (int i = -1; i <= 1; ++i)
    {
      sol[last_index] = joint6 + i * 2.0 * M_PI;
      for (int j = -1; j <= 1; ++j)
      {
        sol[index4] = joint4 + j * 2.0 * M_PI;

        if (isValid(sol))
          joint_poses.push_back(sol);
      }
    }
  }

  //flag an error wherever this is returned if the size < 2 and say that is a critical point with not too many sols
  return joint_poses.size() > 0;
}

bool descartes_moveit::IkFastMoveitStateAdapter::getIK(const Eigen::Isometry3d& pose,
                                                       const std::vector<double>& seed_state,
                                                       std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

bool descartes_moveit::IkFastMoveitStateAdapter::getFK(const std::vector<double>& joint_pose,
                                                       Eigen::Isometry3d& pose) const
{
  const auto& solver = joint_group_->getSolverInstance();

  std::vector<std::string> tip_frame = { solver->getTipFrame() };
  std::vector<geometry_msgs::msg::Pose> output;

  if (!isValid(joint_pose))
    return false;

  if (!solver->getPositionFK(tip_frame, joint_pose, output))
    return false;

  tf2::fromMsg(output[0], pose);  // pose in frame of IkFast base
  pose = world_to_base_.frame * pose * tool0_to_tip_.frame_inv;
  return true;
}

void descartes_moveit::IkFastMoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  descartes_moveit::MoveitStateAdapter::setState(state);
  computeIKFastTransforms(node_);
}

bool descartes_moveit::IkFastMoveitStateAdapter::computeIKFastTransforms(const rclcpp::Node::SharedPtr& node)
{

// ======================================================================================================================================
//   Updated to ROS2 the retrieval of parameters

  std::string ikfast_base_frame;
  std::string ikfast_tool_frame;
  node->get_parameter_or("ikfast_base_frame", ikfast_base_frame, std::string(default_base_frame));
  node->get_parameter_or("ikfast_tool_frame", ikfast_tool_frame, std::string(default_tool_frame));

  RCLCPP_INFO(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IKfast_base_frame is '%s', ikfast_tool_frame is:  '%s', and tool_frame is: '%s'",
             ikfast_base_frame.c_str(), ikfast_tool_frame.c_str(), tool_frame_.c_str());

// ======================================================================================================================================

  if (!robot_state_->knowsFrameTransform(ikfast_base_frame))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             ikfast_base_frame.c_str(), group_name_.c_str());
    return false;
  }

  if (!robot_state_->knowsFrameTransform(ikfast_tool_frame))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             ikfast_tool_frame.c_str(), group_name_.c_str());
    return false;
  }

  // calculate frames
  tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tool_frame_).inverse() *
                                        robot_state_->getFrameTransform(ikfast_tool_frame));

  world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(ikfast_base_frame));

  RCLCPP_INFO(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter initialized with IKFast tool frame '%s' and base frame '%s'.",
            ikfast_tool_frame.c_str(), ikfast_base_frame.c_str());

  
  return true;
}
// write 
void descartes_moveit::IkFastMoveitStateAdapter::updateTcpFrame(std::string& tool_frame_, std::string& ikfast_tool_frame){
  if (!robot_state_->knowsFrameTransform(tool_frame_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             tool_frame_.c_str(), group_name_.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "TCP not updated, keeping the one specified in launch file");
  }

  else {
    tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tool_frame_).inverse() *
                                        robot_state_->getFrameTransform(ikfast_tool_frame));
    RCLCPP_ERROR(rclcpp::get_logger("ikfast_moveit_state_adapter"), "IkFastMoveitStateAdapter updated with IKFast tool frame '%s' and tcp frame '%s'.",
            ikfast_tool_frame.c_str(), tool_frame_.c_str());
  }
}
