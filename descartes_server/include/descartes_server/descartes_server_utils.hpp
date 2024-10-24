#ifndef DESCARTES_TRAJ_PLANNER_UTILS_HPP
#define DESCARTES_TRAJ_PLANNER_UTILS_HPP

#include "descartes_moveit/ikfast_moveit_state_adapter.h"
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_core/robot_model.h"
#include "descartes_core/trajectory_timing_constraint.h"
#include "descartes_planner/dense_planner.h"
#include "descartes_utilities/ros_conversions.h"
#include "descartes_server/srv/descartes_traj_planner.hpp"
#include "descartes_server/srv/piece_portee_traj_planner.hpp"
#include "descartes_server/srv/init_planning_scene.hpp"
#include "descartes_server/srv/add_collision_object.hpp"
#include "descartes_server/srv/remove_collision_object.hpp"


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <utility>
#include <cmath>
#include <omp.h>

/*
  ------------------------------------------------ Structure Declaration ------------------------------------------------------------
*/ 

// ----------------------------------- Struct: Parameters -----------------------------------------------------------
// This struct holds the parameters for trajectory planning, including offsets, increments, velocities, 
// initial and final configurations, and various planner related parameters (for Descartes and underlying for moveit) 
// like robot name, group name, cost factors, type of process (normal planner or piece portee)...
struct Parameters {
    // Process parameters
    double orientation_offset_x;
    double orientation_offset_y;
    double orientation_offset_z;
    double position_offset_x = 0.0;
    double position_offset_y = 0.0;
    double position_offset_z = 0.0;
    double position_increment;
    double orientation_increment;
    double velocity; // meters/second
    double pure_rotation_velocity; // radians/second
    std::vector<double> initial_config;
    std::vector<double> final_config;
    double time_from_start, time_to_init_config, time_to_final_config;

    // Configuration parameters
    std::string robot_description;
    std::string group_name;
    std::string world_frame;
    std::string tcp_frame;
    std::string variable_tcp;
    std::string ikfast_tool_frame;
    double costFactorCSpaceDistance, costFactorDistanceLimits, costFactorManipulability, costFactorNearestCollision, costFactorCostChangePose;
    double velocityLimitFactor;
    double margin = 0.0002; 
    bool isPiecePortee = false; 
    geometry_msgs::msg::TransformStamped transform_tool_base_;
    geometry_msgs::msg::TransformStamped transform_tool_pt_;
    geometry_msgs::msg::TransformStamped wobj_pt;
};

void logParameters(Parameters& parameters);


/*
  ------------------------------------------------ ResultCode Declaration ------------------------------------------------------------
*/ 
enum class ResultCode
{
  SUCCESFUL_PLANNING = 0,
  FAILED_TO_INITIALIZE_ROBOT_MODEL = -1,
  FAILED_TO_INITIALIZE_PLANNER = -2,
  FAILURE_SOLVING_VALID_PATH = -3,
  FAILURE_RETRIEVING_PATH = -4,
  FAILURE_CONVERTING_TO_ROS_PTS = -5,
  PARTIAL_TRAJECTORY_SOLUTION = -6,
};

std::string ResultComment(ResultCode type);

/*
  ------------------------------------------------ Function Declarations ------------------------------------------------------------
*/

// -----------------------------------------  Utils  -----------------------------------------
// These utility functions provide mathematical and ROS-specific conversions and computations used in trajectory planning.

// Converts degrees to radians.
double degreesToRadians(double degrees);

// Converts radians to degrees.
double RadiansToDegrees(double radians);

// Computes upper and lower bounds based on a nominal value and an offset.
std::pair<double, double> computeBounds(double nominal_value, double offset);

// Calculates the timing (duration) needed to move between two waypoints based on velocity.
bool computeTiming(const geometry_msgs::msg::Pose& waypoint, const geometry_msgs::msg::Pose& next_waypoint, double& velocity, double& dt);

// Converts a ROS TransformStamped message to an Eigen::Isometry3d object
Eigen::Isometry3d transformStampedToIsometry(const geometry_msgs::msg::TransformStamped& tf);


// -----------------------------------------  Trajectory Utils  -----------------------------------------
// These trajectory utility functions provide ways of adding initial and final joint configurations and 
// creating descartes trajectory points from 

// Functions for adding initial and final configurations
bool addInitialConfig(std::vector<double>& initial_config, std::vector<descartes_core::TrajectoryPtPtr>& descartesWayPoints, double& time_to_init_config);
bool addFinalConfig(std::vector<double>& final_config, std::vector<descartes_core::TrajectoryPtPtr>& descartesWayPoints, double& time_to_final_config);

// Initial function to convert a vector of Poses (waypoints) to Descartes Cartesian Trajectory points.
// Computes the timing btween poses to respect the velocity constraint added in the params for each point 
// and passes it to the buildCartesianPoint
bool waypointsToDescartesTrajPts(const std::vector<geometry_msgs::msg::Pose>& trajectory_poses, Parameters& parameters, 
                  std::vector<descartes_core::TrajectoryPtPtr>& descartes_traject_pts_);

// Function to build a Cartesian point for Descartes trajectory.
// Takes the timing and the tolerances, depending on these last ones, a different Descartes Cartesian Trajectory Point constructor 
// will be called  
bool buildCartesianPoint(const Eigen::Isometry3d& waypoint, double dt, Parameters& parameters, descartes_core::TrajectoryPtPtr& decart_waypoint_sol);

// Function to create a Descartes Cartesian point with specific parameters
bool makeCartesianPoint(descartes_trajectory::TolerancedFrame& tolerancedFrame, descartes_core::TimingConstraint& dt, double& orientation_increment, 
                        double& position_increment, descartes_core::TrajectoryPtPtr& decart_waypoint_sol);

// Function to create a Cartesian point with full constructor parameters
bool makeCartesianPointFullConstructor(const geometry_msgs::msg::TransformStamped& tool_base, const geometry_msgs::msg::TransformStamped& tool_pt, 
                                        descartes_trajectory::TolerancedFrame& tolerancedFrame, double& orientation_increment, double& position_increment,
                                        descartes_core::TimingConstraint& dt, descartes_core::TrajectoryPtPtr& decart_waypoint_sol);

// Main trajectory planner function
int traj_planner(const rclcpp::Node::SharedPtr& node, Parameters& parameters, const std::vector<geometry_msgs::msg::Pose>& trajectory_poses, trajectory_msgs::msg::JointTrajectory& joint_solutions);


#endif // DESCARTES_TRAJ_PLANNER_UTILS_HPP