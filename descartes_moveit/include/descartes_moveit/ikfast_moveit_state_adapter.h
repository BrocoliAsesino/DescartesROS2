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

#ifndef IKFAST_MOVEIT_STATE_ADAPTER_H
#define IKFAST_MOVEIT_STATE_ADAPTER_H

#include "descartes_moveit/moveit_state_adapter.h"

namespace descartes_moveit
{
class IkFastMoveitStateAdapter : public descartes_moveit::MoveitStateAdapter
{
public:
  virtual ~IkFastMoveitStateAdapter()
  {
  }

// ======================================================================================================================================
//   We keep this method since it has the same signature than the one in the base class RobotModel. Cannot remove since the class inherits
//   from MoveitStateAdapter that in turns inherits from RobotModel and in there the initialize method is pure virtual, removing it would 
//   make this class abstract.

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);


// ======================================================================================================================================
//   New initialize method to include the pointer to a rclcpp::Node so that the RobotModelLoader works. In Moveit2 the RobotModelLoader               
//   takes an extra parameter (rclcpp::Node) to load the robot model than it did in Moveit. Implemented in MoveitStateAdapter. 
//   This method is overridden since it was implemented in the base class RobotModel. 

  virtual bool initialize(const rclcpp::Node::SharedPtr& node, 
                          const std::string& robot_description, 
                          const std::string& group_name,
                          const std::string& world_frame, 
                          const std::string& tcp_frame) override;
// ======================================================================================================================================


  virtual bool getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double> >& joint_poses) const;

  virtual bool getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                     std::vector<double>& joint_pose) const;

  virtual bool getFK(const std::vector<double>& joint_pose, Eigen::Isometry3d& pose) const;

  /**
   * @brief Sets the internal state of the robot model to the argument. For the IKFast impl,
   * it also recomputes the transformations to/from the IKFast reference frames.
   */
  void setState(const moveit::core::RobotState& state);
  void updateTcpFrame(std::string& tool_frame_, std::string& ikfast_tool_frame);

protected:
  bool computeIKFastTransforms(const rclcpp::Node::SharedPtr& node);
  rclcpp::Node::SharedPtr  node_ = nullptr;

  /**
   * The IKFast implementation commonly solves between 'base_link' of a robot
   * and 'tool0'. We will commonly want to take advantage of an additional
   * fixed transformation from the robot flange, 'tool0', to some user defined
   * tool. This prevents the user from having to manually adjust tool poses to
   * account for this.
   */
  descartes_core::Frame tool0_to_tip_;

  /**
   * Likewise this parameter is used to accomodate transformations between the base
   * of the IKFast solver and the base of the MoveIt move group.
   */
  descartes_core::Frame world_to_base_;
};

}  // end namespace 'descartes_moveit'
#endif
