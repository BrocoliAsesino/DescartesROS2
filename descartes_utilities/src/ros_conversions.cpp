/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
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
/*
 *  ros_conversions.cpp
 *
 *  Created on: Feb 28, 2016
 *  Author: Jonathan Meyer
 */

#include <console_bridge/console.h>
#include "descartes_utilities/ros_conversions.h"
#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp/duration.hpp>

/**
 * @brief Given two sets of joint values representing two robot joint poses, this function computes the
 *        minimum amount of time required to interpolate between the points assuming that each joint can
 *        move no faster than a constant 'max_vel' speed.
 * @return The minimum time required to interpolate between poses.
 */

 /**  ALEX
 * @brief Had to modify the way to add up the time since the Duration variables would not want to add up. 
          Weird behavior, i gave up easily cause it was just an addition, if problems arise with the timings it s definetly my fault
 * @return anger 
 */
static double minTime(const std::vector<double>& pose_a, const std::vector<double>& pose_b, double max_vel)
{
  std::vector<double> diff;
  diff.reserve(pose_a.size());

  std::transform(pose_a.begin(), pose_a.end(), pose_b.begin(), std::back_inserter(diff), [max_vel](double a, double b)
                 {
                   return std::abs(a - b) / max_vel;
                 });
  return *std::max_element(diff.begin(), diff.end());
}

bool descartes_utilities::toRosJointPoints(const descartes_core::RobotModel& model,
                                           const std::vector<descartes_core::TrajectoryPtPtr>& joint_traj,
                                           double default_joint_vel,
                                           std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& out)
{
  if (default_joint_vel <= 0.0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("descartes_utilities"), "Invalid value for default joint velocity. Must be > 0 (radians/second)");
    return false;
  }

  const static double max_default_joint_velocity = 100.0;  // (radians / s); approx 1000 rpm
  if (default_joint_vel > max_default_joint_velocity)
  {
    RCLCPP_ERROR(rclcpp::get_logger("descartes_utilities"), "Default joint velocity of %f exceeds assumed limit of %f.", default_joint_vel,
             max_default_joint_velocity);
    return false;
  }

  // rclcpp::Duration from_start(1.0, 0.0);
  double from_start_timer = 0.0;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> ros_trajectory;
  ros_trajectory.reserve(joint_traj.size());

  std::vector<double> joint_point;
  std::vector<double> dummy;

  for (std::size_t i = 0; i < joint_traj.size(); ++i)
  {
    if (!joint_traj[i])
    {
      RCLCPP_ERROR(rclcpp::get_logger("descartes_utilities"), "Input trajectory contained null pointer at index %lu", static_cast<unsigned long>(i));
      return false;
    }

    descartes_core::TrajectoryPt& pt = *joint_traj[i];

    // std::cout << "IN ROS_CONVERSIONS: Is timing set for point " << i; 
    // printf(" %s ",  pt.getTiming().isSpecified() ? "Yes" : "No");
    // std::cout << std::endl; 
    // double timing_upper = pt.getTiming().upper;
    // std::cout << "Timing set for point: " << i << " is = " << pt.getTiming().upper<< " and the double is " << timing_upper << std::endl;

    if (!pt.getNominalJointPose(dummy, model, joint_point))
    {
      RCLCPP_ERROR(rclcpp::get_logger("descartes_utilities"), "Failed to extract joint positions from input trajectory at index %lu",
               static_cast<unsigned long>(i));
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint ros_pt;
    ros_pt.positions = joint_point;
    ros_pt.velocities.resize(joint_point.size(), 0.0);
    ros_pt.accelerations.resize(joint_point.size(), 0.0);
    ros_pt.effort.resize(joint_point.size(), 0.0);

    if (pt.getTiming().isSpecified())
    {
      
      // std::cout << "Current seconds " << from_start_timer << std::endl;
      from_start_timer = from_start_timer + pt.getTiming().upper;
      // std::cout << "Target seconds " << from_start_timer << std::endl;
      // original was just the following: 
      // from_start = from_start + rclcpp::Duration(timing_upper, 0.0);
      
      rclcpp::Duration from_start(std::chrono::nanoseconds(static_cast<int64_t>(from_start_timer * 1e9)));

      // std::cout << "From start " << from_start.seconds() << std::endl;
      ros_pt.time_from_start = from_start;
    }

    else
    {
      double dt;
      if (i == 0)
        {dt = 0.0;}
      else
        {dt = minTime(joint_point, ros_trajectory.back().positions, default_joint_vel);}

      from_start_timer = from_start_timer + dt;
      rclcpp::Duration from_start(std::chrono::nanoseconds(static_cast<int64_t>(from_start_timer * 1e9)));

      ros_pt.time_from_start = from_start;
    }

    // ros_pt.time_from_start = from_start;
    ros_trajectory.push_back(ros_pt);
  }

  out = ros_trajectory;
  return true;
}
