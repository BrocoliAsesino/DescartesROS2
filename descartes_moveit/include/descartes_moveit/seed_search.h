/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2015, Southwest Research Institute
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
 *
 */

/**
 * @file seed_search.h
 * @author Jonathan Meyer
 * @date April 2015
 *
 * @brief This file defines methods that can be used to generate seed values for iterative
 *        numerical solvers used by Moveit for a generic robot.
 *
 * Users can call the findSeedStates() function with a robot state object, a
 * particular Moveit move group, and a series of joint pairs. These joint pairs
 * should define arm configurations such as elbow up or elbow down. For many 6 DOF
 * robots, joints 2 & 3 and joints 4 & 6 (starting counting at 1) will form elbow
 * and wrist configurations.
 */

#ifndef SEED_SEARCH_H
#define SEED_SEARCH_H

#include <moveit/robot_state/robot_state.h>
#include <rclcpp/logging.hpp>

namespace descartes_moveit
{

auto logger2 = rclcpp::get_logger("descartes_moveit2");
namespace seed
{
/**
 * @brief Returns a sequence of seed states for iterative inverse kinematic solvers to use
 *        when 'sampling' the solution space of a pose. These seeds are generated by
 *        iterating through all possible joint permutations of each pair of joints passed
 *        in by the user.
 * @param state Shared pointer to robot state used to perform FK/IK
 * @param group_name Name of the move group for which to generate seeds
 * @param tool_frame The name of the tool link in which to work with FK/IK
 * @param pairs A sequence of joint pairs used to generate the seed states.
 * @return A vector of seed states
 */
std::vector<std::vector<double> > findSeedStatesByPairs(moveit::core::RobotState& state, const std::string& group_name,
                                                        const std::string& tool_frame,
                                                        const std::vector<std::pair<unsigned, unsigned> >& pairs);

/**
 * @brief findIndustrialSixDOFSeeds() is a specialization of findSeedStatesByPairs()
 *        that searches for seed states over joints 2,3 and 4,6 (1 indexed). These
 *        joints often form elbow and wrist configurations.
 */
inline std::vector<std::vector<double> > findIndustrialSixDOFSeeds(moveit::core::RobotState& state,
                                                                   const std::string& group_name,
                                                                   const std::string& tool_frame)
{
  return findSeedStatesByPairs(state, group_name, tool_frame, { { 1, 2 }, { 3, 5 } });
}

/**
 * @brief Uses moveit's underlying random function to find n random joint configurations
 *        that satisfy model bounds.
 * @param state Shared pointer to robot state used to perform FK/IK
 * @param group_name Name of the move group for which to generate seeds
 * @param n Number of random seeds to create
 * @return n random valid positions of 'group_name' in moveit config defined by state
 */
std::vector<std::vector<double> > findRandomSeeds(moveit::core::RobotState& state, const std::string& group_name,
                                                  unsigned n);

}  // end namespace seed
}  // end namespace descartes_moveit

#endif  // SEED_SEARCH_H
