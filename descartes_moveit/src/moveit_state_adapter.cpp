/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Dan Solomon
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

#include <console_bridge/console.h>

#include "descartes_moveit/utils.h"
#include "descartes_moveit/moveit_state_adapter.h"
#include "descartes_moveit/seed_search.h"


// #include <eigen_conversions/eigen_msg.h>
#include <random_numbers/random_numbers.h>
// #include <ros/assert.h>
#include <sstream>

const static int SAMPLE_ITERATIONS = 10;

namespace
{
bool getJointVelocityLimits(const moveit::core::RobotState& state, const std::string& group_name,
                            std::vector<double>& output)
{
  std::vector<double> result;

  auto models = state.getJointModelGroup(group_name)->getActiveJointModels();
  for (const moveit::core::JointModel* model : models)
  {
    const auto& bounds = model->getVariableBounds();
    // Check to see if there is a single bounds constraint (more might indicate
    // not revolute joint)
    if (model->getType() != moveit::core::JointModel::REVOLUTE &&
        model->getType() != moveit::core::JointModel::PRISMATIC)
    {
      // ROS_ERROR_STREAM(__FUNCTION__ << " Unexpected joint type. Currently works only"
      // " with single axis prismatic or revolute joints.");

      RCLCPP_ERROR(descartes_moveit::logger,  " Unexpected joint type. Currently works only with single axis prismatic or revolute joints.");

      return false;
    }
    else
    {
      result.push_back(bounds[0].max_velocity_);
    }
  }

  output = result;
  return true;
}

}  // end anon namespace

namespace descartes_moveit
{

rclcpp::Logger logger = rclcpp::get_logger("moveit_state_adapter");

MoveitStateAdapter::MoveitStateAdapter() : world_to_root_(Eigen::Isometry3d::Identity())
{
}

// ======================================================================================================================================
//   We keep this method since it has the same signature than the one in the base class RobotModel. Cannot remove since the class inherits
//   from RobotModel and in there the initialize method is pure virtual, removing it would make this class abstract.

bool MoveitStateAdapter::initialize(const std::string& /*robot_description*/, const std::string& /*group_name*/,
                                    const std::string& /*world_frame*/, const std::string& /*tcp_frame*/)
{
  // // Initialize MoveIt state objects
  // robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(robot_description));
  // auto model = robot_model_loader_->getModel();
  // if (!model)
  // {
  //   RCLCPP_ERROR(logger,"Failed to load robot model from robot description parameter: %s", robot_description.c_str());
  //   return false;
  // }

  // return initialize(model, group_name, world_frame, tcp_frame);
  return true;
}

// ======================================================================================================================================
//   New initialize method to include the pointer to a rclcpp::Node so that the RobotModelLoader works. In Moveit2 the RobotModelLoader               
//   takes an extra parameter (rclcpp::Node) to load the robot model than it did in Moveit. 

bool MoveitStateAdapter::initialize(const rclcpp::Node::SharedPtr& node, const std::string& robot_description, const std::string& group_name,
                                    const std::string& world_frame, const std::string& tcp_frame)
{
  // Initialize MoveIt state objects
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(node, robot_description));
  auto model = robot_model_loader_->getModel();
  if (!model)
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_state_adapter"), "Failed to load robot model from robot description parameter: %s", robot_description.c_str());
    return false;
  }

  return initialize(model, group_name, world_frame, tcp_frame);
}
// ======================================================================================================================================

bool MoveitStateAdapter::initialize(moveit::core::RobotModelConstPtr robot_model, const std::string &group_name,
                                    const std::string &world_frame, const std::string &tcp_frame)
{
  robot_model_ptr_ = robot_model;
  

  robot_state_.reset(new moveit::core::RobotState(robot_model_ptr_));
  robot_state_->setToDefaultValues();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model));
  joint_group_ = robot_model_ptr_->getJointModelGroup(group_name);

// NEW: to store it in descartes' robot_model (the base class of this class) and have access to it in the CustomEdgesWithTime builder in the planning_graph 
  moveit_robot_model_ = robot_model;
  robot_model_joint_group_ = joint_group_;

  // Assign robot frames
  group_name_ = group_name;
  tool_frame_ = tcp_frame;
  world_frame_ = world_frame;

  // More debugging messages
  RCLCPP_INFO(logger,"Group name set to: %s", group_name_.c_str());
  RCLCPP_INFO(logger,"Tool frame set to: %s", tool_frame_.c_str());
  RCLCPP_INFO(logger,"World frame set to: %s", world_frame_.c_str());

  // Validate our model inputs w/ URDF
  if (!joint_group_)
  {
    RCLCPP_ERROR(logger,"%s: Joint group '%s' does not exist in robot model", __FUNCTION__, group_name_.c_str());
    std::stringstream msg;

    msg << "Possible group names: ";
    for (const auto& name : robot_state_->getRobotModel()->getJointModelGroupNames())
      msg << name << ", ";
    RCLCPP_ERROR(logger,msg.str().c_str());

    return false;
  }

  const auto& link_names = joint_group_->getLinkModelNames();
 
  // if (tool_frame_ != link_names.back())
  // {
  //   RCLCPP_ERROR(logger,"%s: Tool frame '%s' does not match group tool frame '%s', functionality"
  //            " will be implemented in the future",
  //            __FUNCTION__, tool_frame_.c_str(), link_names.back().c_str());
  //   return false;
  // }
  
  if (tool_frame_ != link_names.back())
  // {
  //   RCLCPP_ERROR(logger,"%s: Tool frame '%s' does not match group tool frame '%s', functionality"
  //            " will be implemented in the future NOT AN ISSUE THO - ALEX SAYS",
  //            __FUNCTION__, tool_frame_.c_str(), link_names.back().c_str());
  // }
  {
    RCLCPP_DEBUG(logger,"%s: Tool frame '%s' does not match group tool frame '%s', functionality"
             " will be implemented in the future NOT AN ISSUE THO - ALEX SAYS - This is part of the issue that arises because of the IKFast"
             "plugin calculating on its own the transform betwen these two frames, which is also done in the planner",
             __FUNCTION__, tool_frame_.c_str(), link_names.back().c_str());
  }

  if (!::getJointVelocityLimits(*robot_state_, group_name, velocity_limits_))
  {
    RCLCPP_WARN(logger,"%s: Could not determine velocity limits of RobotModel from MoveIt", __FUNCTION__);
  }

  if (seed_states_.empty())
  {
    seed_states_ = seed::findRandomSeeds(*robot_state_, group_name_, SAMPLE_ITERATIONS);
    RCLCPP_DEBUG(logger,"Generated %lu random seeds", static_cast<unsigned long>(seed_states_.size()));
  }

  auto model_frame = robot_state_->getRobotModel()->getModelFrame();
  if (world_frame_ != model_frame)
  {
    RCLCPP_DEBUG(logger,"%s: World frame '%s' does not match model root frame '%s', all poses will be"
              " transformed to world frame '%s'",
              __FUNCTION__, world_frame_.c_str(), model_frame.c_str(), world_frame_.c_str());

    Eigen::Isometry3d root_to_world = toIsometry(robot_state_->getFrameTransform(world_frame_));
    world_to_root_ = descartes_core::Frame(root_to_world.inverse());
  }

  return true;
}

bool MoveitStateAdapter::getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                               std::vector<double>& joint_pose) const
{
  robot_state_->setJointGroupPositions(group_name_, seed_state);
  return getIK(pose, joint_pose);
}

bool MoveitStateAdapter::getIK(const Eigen::Isometry3d& pose, std::vector<double>& joint_pose) const
{
  bool rtn = false;

  // transform to group base
  Eigen::Isometry3d tool_pose = world_to_root_.frame * pose;

  // std::cout << "WORLD TO ROOT  =  " << world_to_root_.frame.matrix() << std::endl << std::flush;

  if (robot_state_->setFromIK(joint_group_, tool_pose, tool_frame_))
  {
    robot_state_->copyJointGroupPositions(group_name_, joint_pose);
    if (!isValid(joint_pose))
    {
      RCLCPP_DEBUG(rclcpp::get_logger("descartes_moveit"), "Robot joint pose is invalid");
    }
    else
    {
      rtn = true;
    }
  }
  else
  {
    rtn = false;
  }

  return rtn;
}

bool MoveitStateAdapter::getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const
{
  // The minimum difference between solutions should be greater than the search discretization
  // used by the IK solver.  This value is multiplied by 4 to remove any chance that a solution
  // in the middle of a discretization step could be double counted.  In reality, we'd like solutions
  // to be further apart than this.
  double epsilon = 4 * joint_group_->getSolverInstance()->getSearchDiscretization();
  RCLCPP_DEBUG(logger,"Utilizing an min. difference of %f between IK solutions", epsilon);
  joint_poses.clear();
  
  for (size_t sample_iter = 0; sample_iter < seed_states_.size(); ++sample_iter)
  {
    robot_state_->setJointGroupPositions(group_name_, seed_states_[sample_iter]);  //sets the initial joint config (the seed state) that the Ik solver will use as starting point
    std::vector<double> joint_pose;
    if (getIK(pose, joint_pose))
    {
      if (joint_poses.empty())
      {
        std::stringstream msg;
        RCLCPP_DEBUG(logger,msg.str().c_str());
        joint_poses.push_back(joint_pose);
      }
      else
      {
        std::stringstream msg;
        RCLCPP_DEBUG(logger,msg.str().c_str());

        std::vector<std::vector<double> >::iterator joint_pose_it;
        bool match_found = false;
        for (joint_pose_it = joint_poses.begin(); joint_pose_it != joint_poses.end(); ++joint_pose_it)
        {
          if (descartes_core::utils::equal(joint_pose, (*joint_pose_it), epsilon))
          {
            RCLCPP_DEBUG(logger,"Found matching, potential solution is not new");
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          std::stringstream msg;
          RCLCPP_DEBUG(logger,msg.str().c_str());
          joint_poses.push_back(joint_pose);
        }
      }
    }
  }

  // RCLCPP_DEBUG(rclcpp::get_logger("moveit_state_adapter"), "Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
  //          static_cast<unsigned long>(seed_states_.size()));

  if (joint_poses.empty())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("moveit_state_adapter"), "Found 0 joint solutions out of %lu iterations", static_cast<unsigned long>(seed_states_.size()));
    return false;
  }
  else
  {
    RCLCPP_DEBUG(rclcpp::get_logger("moveit_state_adapter"),"Found %lu joint solutions out of %lu iterations", static_cast<unsigned long>(joint_poses.size()),
              static_cast<unsigned long>(seed_states_.size()));
    return true;
  }
}

bool isInSingularity(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group) 
{

  const static double MIN_DETERMINANT_VALUE = 0.00001;
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  state.getJacobian(group, state.getLinkModel(group->getLinkModelNames().back()), reference_point_position, jacobian);

  return std::abs(jacobian.determinant()) < MIN_DETERMINANT_VALUE;

}

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

bool MoveitStateAdapter::isInLimits(const std::vector<double> &joint_pose) const
{
  return joint_group_->satisfiesPositionBounds(joint_pose.data());
}

bool MoveitStateAdapter::getFK(const std::vector<double>& joint_pose, Eigen::Isometry3d& pose) const
{
  bool rtn = false;
  robot_state_->setJointGroupPositions(group_name_, joint_pose);
  if (isValid(joint_pose))
  {
    if (robot_state_->knowsFrameTransform(tool_frame_))
    {
      pose = toIsometry(world_to_root_.frame * robot_state_->getFrameTransform(tool_frame_));
      //pose.
      rtn = true;
    }
    else
    {
      RCLCPP_ERROR(logger,"Robot state does not recognize tool frame: %s", tool_frame_.c_str());
      rtn = false;
    }
  }
  else
  {
    RCLCPP_ERROR(logger,"Invalid joint pose passed to get forward kinematics");
    rtn = false;
  }

  return rtn;
}

// in isInCollision we also check for the singularities -- adds a lot of time to the calculation, commenting for now
bool MoveitStateAdapter::isValid(const std::vector<double>& joint_pose) const
{
  // Logical check on input sizes
  if (joint_group_->getActiveJointModels().size() != joint_pose.size())
  {
    RCLCPP_ERROR(logger,"Size of joint pose: %lu doesn't match robot state variable size: %lu",
             static_cast<unsigned long>(joint_pose.size()),
             static_cast<unsigned long>(joint_group_->getActiveJointModels().size()));
    return false;
  }

  return isInLimits(joint_pose) && !isInCollision(joint_pose);
}

bool MoveitStateAdapter::isValid(const Eigen::Isometry3d& pose) const
{
  // TODO: Could check robot extents first as a quick check
  std::vector<double> dummy;
  return getIK(pose, dummy);
}

int MoveitStateAdapter::getDOF() const
{
  return joint_group_->getVariableCount();
}

bool MoveitStateAdapter::isValidMove(const double* from_joint_pose,
                                     const double* to_joint_pose, double dt) const
{

  for (int i = 0; i < getDOF(); ++i)
  {
    double dtheta = std::abs(from_joint_pose[i] - to_joint_pose[i]);
    double max_dtheta = dt * velocity_limits_[i];
    if (dtheta > max_dtheta)
      return false;
  }

  return true;
}

std::vector<double> MoveitStateAdapter::getJointVelocityLimits() const
{
  return velocity_limits_;
}

void MoveitStateAdapter::setState(const moveit::core::RobotState& state)
{
  // ROS_ASSERT_MSG(static_cast<bool>(robot_state_), "'robot_state_' member pointer is null. Have you called "
  //                                                 "initialize()?");

  assert(robot_state_ && "robot_state_ member pointer is null. Have you called initialize()?");
  
  *robot_state_ = state;
  planning_scene_->setCurrentState(state);
}

}  // descartes_moveit
