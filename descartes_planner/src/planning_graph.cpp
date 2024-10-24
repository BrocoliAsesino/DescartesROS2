/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
 * planning_graph.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: Dan Solomon
 *      Author: Jonathan Meyer
 */

#include "descartes_planner/planning_graph.h"
#include "descartes_planner/ladder_graph_dag_search.h"
#include "descartes_planner/planning_graph_edge_policy.h"
// #include <ros/console.h>
#include <console_bridge/console.h>
#include <omp.h>


using namespace descartes_core;
using namespace descartes_trajectory;

namespace descartes_planner
{

PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback)
  : graph_(model->getDOF()), robot_model_(std::move(model)), custom_cost_function_(cost_function_callback)
{}

// ======================================================================================================================================
//   Added new constructor method to be able to pass the manipulability cost factor to descartes_planner::LadderGraph graph_. 
PlanningGraph::PlanningGraph(RobotModelConstPtr model, CostFunction cost_function_callback, double& costFactorManipulability, double& velocity_limit_factor)
  : graph_(model->getDOF()), robot_model_(std::move(model)), custom_cost_function_(cost_function_callback), costFactorManipulability_(costFactorManipulability),
  velocity_limit_factor_(velocity_limit_factor)
{}

// ======================================================================================================================================
//   Modifications added for being able to calculate partial trajectories when the IK solver is not able to solve every waypoint.
//   The solution vector is resized with the min number of consecutive waypoints from 0 that their solution has been found. 
bool PlanningGraph::insertGraph(const std::vector<TrajectoryPtPtr>& points)
{
  if (points.size() < 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("planning_graph"), ": must provide at least 2 input trajectory points.");
    return false;
  }

  if (graph_.size() > 0) clear();

  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Calculating Joint Solutions ...");
  std::vector<std::vector<std::vector<double>>> all_joint_sols;
  
  size_t partial_solution_size = points.size();
  if (!calculateJointSolutions(points.data(), points.size(), all_joint_sols))
  {
    partial_solution_size = all_joint_sols.size();
    std::cout << "Calculated joint solutions found :  " << partial_solution_size << std::endl;
    // return false;  //// removed the return status so that we keep executing the method in the case of partial trajectories 
  }

  if (partial_solution_size < 2){
    std::cout << "----------------------------- ALERT!! ----------------------------- " << std::endl;
    std::cout << "Path provided unatainable due to unreachable poses or collisions " << std::endl;
    std::cout << "----------------------------- SUGESTION 1 ----------------------------- " << std::endl;
    std::cout << "Verify the poses can be achieved and that no object in the planning scene" << std::endl;
    std::cout << "is in collision this includes the robot but also the rest of the elements" << std::endl;
    std::cout << "----------------------------- SUGESTION 2 ----------------------------- " << std::endl;
    std::cout << "Check that the first pose is attainable, if it isn t the partial solution size is 0" << std::endl;
    return false;
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Joint Solutions Calculated !");
  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Creating Graph, inserting all joint solutions for each pose as vertex in the rungs...");
  
  // insert into graph as vertices
  // graph_.resize(points.size());
  graph_.resize(partial_solution_size); //// we change the resizing, if all solutions are found it will be the same size as points                                
  
  // for (std::size_t i = 0; i < points.size(); ++i)
  for (std::size_t i = 0; i < partial_solution_size; ++i)
  {
    graph_.assignRung(i, points[i]->getID(), points[i]->getTiming(), all_joint_sols[i]);
  }


  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Graph Finished ! All data in the rungs");
  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Computing and calculating edges ...");

  int num_threads = omp_get_num_procs() - 2;
  omp_set_num_threads(num_threads);
  
  auto start_time = std::chrono::high_resolution_clock::now();

  #pragma omp parallel for
  for (std::size_t i = 0; i < graph_.size() - 1; ++i)
  {
    computeAndAssignEdges(i, i + 1);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  std::cout << "~~ Time taken to calculate edges: " << duration << " microseconds" << "\n" <<std::endl;
  
  RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Edges computed and assigned !");
  return true;
}

bool PlanningGraph::addTrajectory(TrajectoryPtPtr point, TrajectoryPt::ID previous_id, TrajectoryPt::ID next_id)
{
  assert(previous_id.is_nil() || graph_.indexOf(previous_id).second); // These tests ensure that the user points
  assert(next_id.is_nil() || graph_.indexOf(next_id).second);         // actually exist. Maybe should be part of
                                                                       // the normal interface? TODO
  auto ns = graph_.indexOf(next_id);

  // Next & prev can be 'null' indicating end & start of trajectory
  std::vector<std::vector<std::vector<double>>> poses;
  calculateJointSolutions(&point, 1, poses); // TODO: If there are no points, return false?

  // Insert new point into graph
  auto insert_idx = ns.second ? ns.first : graph_.size();
  graph_.insertRung(insert_idx);
  graph_.assignRung(insert_idx, point->getID(), point->getTiming(), poses[0]);

  // Build edges from prev point, if applicable
  if (!previous_id.is_nil())
  {
    auto prev_idx = insert_idx - 1;
    computeAndAssignEdges(prev_idx, insert_idx);
  }

  // Build edges to next point, if applicable
  if (!next_id.is_nil())
  {
    auto next_idx = insert_idx + 1;
    computeAndAssignEdges(insert_idx, next_idx);
  }

  return true;
}

bool PlanningGraph::modifyTrajectory(TrajectoryPtPtr point)
{
  auto s = graph_.indexOf(point->getID());
  if (!s.second) return false; // no such point
  auto idx = s.first;

  // we will need to recompute some vertices now
  std::vector<std::vector<std::vector<double>>> poses;
  calculateJointSolutions(&point, 1, poses); // TODO: If there are no points, return false?

  // clear vertices & edges of 'point'
  graph_.clearVertices(idx);
  graph_.clearEdges(idx);
  graph_.assignRung(idx, point->getID(), point->getTiming(), poses[0]);

  // If there is a previous point, compute new edges
  if (!graph_.isFirst(idx))
  {
    auto prev_idx = idx - 1;
    computeAndAssignEdges(prev_idx, idx);
  }

  // If there is a next point, compute new edges
  if (!graph_.isLast(idx))
  {
    auto next_idx = idx + 1;
    computeAndAssignEdges(idx, next_idx);
  }

  return true;
}

bool PlanningGraph::removeTrajectory(const TrajectoryPt::ID& point)
{
  // Remove a point from the graph
  auto s = graph_.indexOf(point);
  if (!s.second) return false;

  auto in_middle = !graph_.isFirst(s.first) && !graph_.isLast(s.first);

  // remove the vertices & edges associated with this point
  graph_.removeRung(s.first);

  // recompute edges from previous rung to next rung, if applicable
  if (in_middle)
  {
    auto prev_idx = s.first - 1;
    auto next_idx = s.first; // We erased a point, so the indexes have collapsed by one
    computeAndAssignEdges(prev_idx, next_idx);
  }

  return true;
}

bool PlanningGraph::getShortestPath(double& cost, std::list<JointTrajectoryPt>& path)
{
  DAGSearch search (graph_);
  cost = search.run();
  if (cost == std::numeric_limits<double>::max()) return false;

  auto path_idxs = search.shortestPath();
  const auto dof = graph_.dof();

  for (size_t i = 0; i < path_idxs.size(); ++i)
  {
    const auto idx = path_idxs[i];
    const auto* data = graph_.vertex(i, idx);
    const auto& tm = graph_.getRung(i).timing;

    auto pt = JointTrajectoryPt(std::vector<double>(data, data + dof), tm);
    path.push_back(std::move(pt));
  }

  std::cout << "Computed path of length " << path_idxs.size() << " with cost " << cost << "\n" << std::endl;
  
  return true;
}

// ======================================================================================================================================
//   Modifications added for being able to keep calculating the IK of the waypoints even if some of them failed. In order to not decrease
//   computational speed when solving the IKs we use omp parallel in the for loop. This allows the for loop to be executed in different 
//   threads, the only issue being that it does not go through the iterations in order. That is why we allow to calculate everything and 
//   then find the min index that failed.
bool PlanningGraph::calculateJointSolutions(const TrajectoryPtPtr* points, const std::size_t count,
                                            std::vector<std::vector<std::vector<double>>>& poses) const
{
  poses.resize(count);
  bool success = true;
  
  std::vector<std::size_t> failure_indices(count, count); // vector of size of the waypoints and filled with the indew of the last waypoint
  auto start_time = std::chrono::high_resolution_clock::now();

  int num_threads = omp_get_num_procs() - 2;
  omp_set_num_threads(num_threads);


  #pragma omp parallel for 
  // #pragma omp parallel for ordered shared(success) // removed so that we would calculate every solution even if some of them fail 
  for (std::size_t i = 0; i < count; ++i)
  { 
    // if (success) // removed so that we would calculate every solution even if some of them fail

      std::vector<std::vector<double>> joint_poses;
      RCLCPP_DEBUG(rclcpp::get_logger("planning_graph"), "Getting Joint Poses for point with ID = %lu (%zu)..." , points[i]->getID().value(), i);
      
      points[i]->getJointPoses(*robot_model_, joint_poses);

      if (joint_poses.empty())
      {
        RCLCPP_ERROR(rclcpp::get_logger("planning_graph"),  " IK failed for input trajectory point number = %zu" , i);

        success = false;
        failure_indices[i] = i; // replaced the value of the last index by the index of the waypoint that failed
      }

      poses[i] = std::move(joint_poses);
   
  }
 
  auto min_failure_index = *std::min_element(failure_indices.begin(), failure_indices.end()); // find minimum index that failed

  if (min_failure_index < count && min_failure_index > 0){

    poses.resize(min_failure_index - 1); // resize the poses vector to the highest consecutive number of waypoints from 0 that was successfully found 
                                        //// the -1 is because the min index will be a failure already so we take the one before
    RCLCPP_ERROR(rclcpp::get_logger("planning_graph"),  "Unable to solve the IK of all waypoints, the partial solution constains the first " \
                                                        "%zu points of the trajectory out of %zu" , min_failure_index - 1, count);
  }


  else if(min_failure_index < count && min_failure_index == 0){
    poses.resize(0);
    success = false;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
 
  std::cout << "~~ Time taken to calculate joint solutions: " << duration << " microseconds" << std::endl;

  return success;
}

// ======================================================================================================================================
//   Modifications added for being able to account for the manipulabity cost without compromising in computing time if it were added in the 
//   Cost Function. We have added a new struct CustomEdgesWithTimeManipulability. To see implementation go to planning_graph_edge_policy.h
void PlanningGraph::computeAndAssignEdges(const std::size_t start_idx, const std::size_t end_idx)
{
  assert(end_idx > start_idx);
  assert(end_idx - start_idx == 1);

  const auto& joints1 = graph_.getRung(start_idx).data;
  const auto& joints2 = graph_.getRung(end_idx).data;
  const auto& tm = graph_.getRung(end_idx).timing;
  const auto dof = robot_model_->getDOF();

  const auto start_size = joints1.size() / dof;
  const auto end_size = joints2.size() / dof;

  bool b;
  std::vector<LadderGraph::EdgeList> edges;
  

  if (!custom_cost_function_ && tm.isSpecified())
  {
    DefaultEdgesWithTime builder (start_size, end_size, dof, tm.upper, robot_model_->getJointVelocityLimits());
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }

  // ================================================================================
  // ================================================================================
  else if (custom_cost_function_ && tm.isSpecified() && costFactorManipulability_ != 0.0)
  {
    
    CustomEdgesWithTimeManipulability builder (start_size, end_size, dof, tm.upper, robot_model_->getJointVelocityLimits(), custom_cost_function_, robot_model_->moveit_robot_model_,
                                 robot_model_->robot_model_joint_group_, costFactorManipulability_, "tcp", velocity_limit_factor_);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  // ================================================================================
  else if (custom_cost_function_ && tm.isSpecified() && costFactorManipulability_ == 0.0)
  {
    CustomEdgesWithTime builder (start_size, end_size, dof, tm.upper, robot_model_->getJointVelocityLimits(), custom_cost_function_);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  // ================================================================================
  // ================================================================================

  else if (!custom_cost_function_ && !tm.isSpecified())
  {
    DefaultEdgesWithoutTime builder (start_size, end_size, dof);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }
  else
  {
    CustomEdgesWithoutTime builder (start_size, end_size, dof, custom_cost_function_);
    edges = calculateEdgeWeights(builder, joints1, joints2, dof, b);
  }

  graph_.assignEdges(start_idx, std::move(edges));
  if (!b) RCLCPP_ERROR(rclcpp::get_logger("planning_graph"),"No edges between user input points at index %lu and %lu", start_idx, end_idx);
}

template<typename EdgeBuilder>
std::vector<LadderGraph::EdgeList> PlanningGraph::calculateEdgeWeights(EdgeBuilder&& builder, const std::vector<double>& start_joints,
                                                                       const std::vector<double>& end_joints, const size_t dof,
                                                                       bool& has_edges) const
{
  const auto from_size = start_joints.size();
  const auto to_size = end_joints.size();
  const auto n_start_points = from_size / dof;
  const auto n_end_points = to_size / dof;

  for (size_t i = 0; i < n_start_points; i++) // from rung
  {
    const auto start_index = i * dof;

    for (size_t j = 0; j < n_end_points; j++) // to rung
    {
      const auto end_index = j * dof;

      builder.consider(&start_joints[start_index], &end_joints[end_index], j);
    }

    builder.next(i);
  }

  has_edges = builder.hasEdges();
  return std::move(builder.result());
}

} /* namespace descartes_planner */
