#ifndef CUSTOM_COST_FUNCTION_CLASS_HPP
#define CUSTOM_COST_FUNCTION_CLASS_HPP

#include <cmath>
#include <vector>
#include <Eigen/Dense> 
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>


#include <chrono>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <moveit/planning_interface/planning_scene_interface.h>


class CustomCostFunctionClass 
{
public:
    CustomCostFunctionClass(std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr, Parameters& params);

    void addBoxToPlanningScene();
    double CustomCostFunction(const double* start, const double* stop);
    double CostCSpaceDistanceL1(const double* start, const double* stop);
    double CostCSpaceLogDistanceToLimits(const double* start);
    double computeForValues(double q, double qmin, double qmax);
    double CostNearestCollision(const double* start, const std::string &joint_group_name);
    double CostChangePose(const double* start, const double* stop, const std::string &joint_group_name);
    descartes_planner::CostFunction getCostFunction();

private:
    std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr_; //It will be a pointer to the IkFastMoveitStateAdapter instance stored in the RobotModelPtr created in traj_planner
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group;
    planning_scene::PlanningScenePtr planning_scene;
    double margin_;
    const std::string joint_group_name_;
    const std::string tcp_frame_;
    double factorC1_, factorC2_, factorC4_, factorC5_;
    int contador = 0; 
};

#endif // CUSTOM_COST_FUNCTION_CLASS_HPP
