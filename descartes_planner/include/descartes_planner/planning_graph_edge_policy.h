/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
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

#ifndef PLANNING_GRAPH_EDGE_POLICY_H
#define PLANNING_GRAPH_EDGE_POLICY_H

#include "descartes_planner/ladder_graph.h"
#include <numeric>

#include <unordered_map>
#include <moveit/robot_state/robot_state.h>

namespace descartes_planner
{

struct DefaultEdgesWithTime
{
 DefaultEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits)
    : results_(n_start)
    , edge_scratch_(n_end)
    , max_dtheta_(dof)
    , delta_buffer_(dof)
    , dof_(dof)
    , count_(0)
    , has_edges_(false)
  {
   std::transform(joint_vel_limits.cbegin(), joint_vel_limits.cend(), max_dtheta_.begin(), [upper_tm] (double v) {
                    return v * upper_tm;
                  });
  }

  inline void consider(const double* const start, const double* const stop, size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      if (delta_buffer_[i] > max_dtheta_[i]) return;
    }

    auto cost = std::accumulate(delta_buffer_.cbegin(), delta_buffer_.cend(), 0.0);
    edge_scratch_[count_].cost = cost;
    edge_scratch_[count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  inline void next(const size_t i)
  {
    results_[i].assign(edge_scratch_.cbegin(), edge_scratch_.cbegin() + count_);
    has_edges_ = has_edges_ || count_ > 0;
    count_ = 0;
  }

  inline std::vector<LadderGraph::EdgeList>& result() noexcept { return results_; }

  inline bool hasEdges() const noexcept { return has_edges_; }

  std::vector<LadderGraph::EdgeList> results_;
  LadderGraph::EdgeList edge_scratch_; // pre-allocated space to work in
  std::vector<double> max_dtheta_;
  std::vector<double> delta_buffer_;
  size_t dof_;
  unsigned count_;
  bool has_edges_;
};

struct CustomEdgesWithTime : public DefaultEdgesWithTime
{
  CustomEdgesWithTime(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits,
                      descartes_planner::CostFunction fn
                      ) 
    : DefaultEdgesWithTime(n_start, n_end, dof, upper_tm, joint_vel_limits),
      custom_cost_fn(fn)
  {}

  inline void consider(const double * const start, const double * const stop, const size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      // std::cout << "joint angle difference : " << delta_buffer_[i] << " max angle change allowed = " << max_dtheta_[i] << std::endl; 
      if (delta_buffer_[i] > max_dtheta_[i]){
        // std::cout << "ignoring that solution ... " << std::endl;
        return;
      } 
    }

    double cost = custom_cost_fn(start, stop);
    // std::cout << "cost is : " << cost << " with " << custom_cost_fn << " + " << determinant << std::endl;
    edge_scratch_[count_++] = {cost, static_cast<unsigned>(index)};
  }

  descartes_planner::CostFunction custom_cost_fn; // TODO: Header doesn't stand on its own
};

// ============================================ New Structure ============================================================
//   Modifications added for being able to account for the manipulabity cost without compromising in computing time if it were added in the 
//   Cost Function. The issue with having it in the Cost Function is that it has to instantiate each time the robot state to find the determinant
//   and ultimately the manipulabity. In that case, the number of times that this happens is however many joint solutions we have in one rung to
//   the power of however many solutions we have in the next and all this times the number of points in the trajectory. As you can see this would 
//   be extremely time consuming. 
//   In this approach we instantiated it once in the initialization of this struct. This is instantiated in a method that is parallelized, each 
//   thread being a different rung in the graph. Which means that robot model only gets instantiated the number of waypoints in the traj and is 
//   updated for every joint solution in the target rung. Not only that but since here we are calculating edge weights and the manipulation cost that
//   interest us is just of the target pose, we create a cache since every node/joint solution in the start rung will be evaluated vs the target rung
//   and thus the manipulability of the target joint solution will not need to be recalculated for each combination, wa can simply store the 
//   calculated ones and retrieve them when necessary. This helps speeding up the process considerably. 
struct CustomEdgesWithTimeManipulability : public DefaultEdgesWithTime
{
  descartes_planner::CostFunction custom_cost_fn; // TODO: Header doesn't stand on its own
  moveit::core::RobotState robot_state; // Declare as a member variable
  std::unordered_map<std::string, double> determinant_cache; // Declare as a member variable
  const moveit::core::JointModelGroup* joint_model_group; // Declare as a member variable
  double costFactorManipulability_;
  std::string tcp_frame; // Declare as a member variable
  double velocity_limit_factor_ = 1.0;

  CustomEdgesWithTimeManipulability(const size_t n_start,
                      const size_t n_end,
                      const size_t dof,
                      const double upper_tm,
                      const std::vector<double>& joint_vel_limits,
                      descartes_planner::CostFunction fn,
                      moveit::core::RobotModelConstPtr robot_model_ptr,
                      const moveit::core::JointModelGroup* joint_model_group, // Add parameter
                      double& costFactorManipulability,
                      const std::string& tcp_frame_name,
                      double velocity_limit_factor
                      ) // Add parameter
    : DefaultEdgesWithTime(n_start, n_end, dof, upper_tm, joint_vel_limits),
      custom_cost_fn(fn),
      robot_state(robot_model_ptr), // Initialize robot_state
      joint_model_group(joint_model_group), // Initialize joint_model_group
      costFactorManipulability_(costFactorManipulability),
      tcp_frame(tcp_frame_name), // Initialize tcp_frame
      velocity_limit_factor_(velocity_limit_factor)

  {
    std::unordered_map<std::string, double> determinant_cache;
    moveit::core::RobotState robot_state(robot_model_ptr); 
    // std::cout << "cost manip 1/m" << std::endl;
  }

  double calculateDeterminant(const double* stop)
  {
    // std::cout << "calculating det" << std::endl;
    
    
    std::string group_name = joint_model_group -> getName();

    
    robot_state.setJointGroupPositions(group_name, stop);
    robot_state.update();

    Eigen::MatrixXd jacobian;
    robot_state.getJacobian(joint_model_group, robot_state.getLinkModel(tcp_frame), Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);

    double det = (jacobian * jacobian.transpose()).determinant();

    if (det <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("customCostFunction"), "Invalid Jacobian!");
        return std::numeric_limits<double>::infinity();
    }
    // else if (det < 0.0){
    //     return std::numeric_limits<double>::infinity();
    // }
  
    else
    {
        double m = std::sqrt(det);
        // double cost = 1.0/(1.0 + m);
        // double cost = 1/m;
        double cost = -log(m);
        // std::cout << cost << std::endl;

        return costFactorManipulability_*cost;
    }
  }

  std::string createKey(const double* stop, size_t dof)
  {
    std::string key;
    for (size_t i = 0; i < dof; ++i)
    {
      key += std::to_string(stop[i]) + ",";
    }
    return key;
}

  inline void consider(const double * const start, const double * const stop, const size_t index) noexcept
  {
    for (size_t i = 0; i < dof_; ++i)
    {
      delta_buffer_[i] = std::abs(start[i] - stop[i]);
      // if (delta_buffer_[i] > max_dtheta_[i]*(0.9-0.1*i)){
      if (delta_buffer_[i] > max_dtheta_[i]*(velocity_limit_factor_)){
        return;
      } 
    }
    
    std::string key = createKey(stop, dof_);
    double determinant = 0; 

    // Check if the determinant is already in the cache
    auto it = determinant_cache.find(key);
    if (it != determinant_cache.end()){
      // std::cout << "Cache hit, use the stored value" << std::endl;
      determinant = it->second;
    }
    else
    {
      // Cache miss, calculate and store the value
      determinant = calculateDeterminant(stop); 
      determinant_cache[key] = determinant;
    }

    double cost = custom_cost_fn(start, stop) + determinant;
    // std::cout << "cost is : " << cost << " with " << custom_cost_fn << " + " << determinant << " + "<< costVelocityLimits << std::endl;
    edge_scratch_[count_++] = {cost, static_cast<unsigned>(index)};
  }

};

// ====================================================================================================================================

struct DefaultEdgesWithoutTime
{
  DefaultEdgesWithoutTime(const size_t n_start,
                       const size_t n_end,
                       const size_t dof)
     : results_(n_start)
     , dof_(dof)
     , count_(0)
     , layer_(0)
  {
    for (auto& edges : results_)
    {
      edges.resize(n_end);
    }
  }

  inline bool hasEdges() const { return true; }

  // no-op
  inline void next(const size_t) { layer_++; count_ = 0; }

  inline std::vector<LadderGraph::EdgeList>& result() noexcept { return results_; }

  inline void consider(const double* const start, const double* const stop, const size_t index) noexcept
  {
    double cost = 0.0;
    for (size_t i = 0; i < dof_; ++i)
      cost += std::abs(start[i] - stop[i]);

    results_[layer_][count_].cost = cost;
    results_[layer_][count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  std::vector<LadderGraph::EdgeList> results_;
  size_t dof_;
  size_t count_;
  size_t layer_;
};

struct CustomEdgesWithoutTime : public DefaultEdgesWithoutTime
{
  CustomEdgesWithoutTime(const size_t n_start,
                       const size_t n_end,
                       const size_t dof,
                          descartes_planner::CostFunction fn)
    : DefaultEdgesWithoutTime(n_start, n_end, dof), custom_cost_fn(fn)
  {}

  inline void consider(const double* const start, const double* const stop, const size_t index) noexcept
  {
    results_[layer_][count_].cost = custom_cost_fn(start, stop);
    results_[layer_][count_].idx = static_cast<unsigned>(index);
    count_++;
  }

  descartes_planner::CostFunction custom_cost_fn; // TODO: Header doesn't stand on its own
};

}

#endif // PLANNING_GRAPH_EDGE_POLICY_H
