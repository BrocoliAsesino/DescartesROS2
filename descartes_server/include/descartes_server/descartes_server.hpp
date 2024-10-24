#ifndef DESCARTES_SERVER_HPP
#define DESCARTES_SERVER_HPP

#include "descartes_moveit/ikfast_moveit_state_adapter.h"

#include "descartes_server/srv/descartes_traj_planner.hpp"
#include "descartes_server/srv/piece_portee_traj_planner.hpp"
#include "descartes_server/srv/init_planning_scene.hpp"
#include "descartes_server/srv/add_collision_object.hpp"
#include "descartes_server/srv/remove_collision_object.hpp"

#include "descartes_server/descartes_server_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_eigen/tf2_eigen.hpp>


/*
  ------------------------------------------------ Class Declaration ------------------------------------------------------------
*/

// ----------------------------------- Class: TrajectoryPlannerServerClass --------------------------------------------
// This class represents a trajectory planning server in a ROS2 node, capable of handling cartesian trajectory planning requests.
class TrajectoryPlannerServerClass : public rclcpp::Node {
public:
    TrajectoryPlannerServerClass();

private:
    rclcpp::Service<descartes_server::srv::DescartesTrajPlanner>::SharedPtr serviceDescartesTrajPlanner;
    rclcpp::Service<descartes_server::srv::PiecePorteeTrajPlanner>::SharedPtr servicePiecePortee;
    rclcpp::Service<descartes_server::srv::AddCollisionObject>::SharedPtr serviceAddCollisionObject;
    rclcpp::Service<descartes_server::srv::RemoveCollisionObject>::SharedPtr serviceRemoveCollisionObject;
    rclcpp::Service<descartes_server::srv::InitPlanningScene>::SharedPtr serviceInitPlanningScene;

    bool planning_scene_init_ = false;

    Parameters parameters_;
    rclcpp::Node::SharedPtr node_; 
    trajectory_msgs::msg::JointTrajectory joint_solutions;
    std::vector<std::string> movegroup_joints_;  
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    descartes_core::RobotModelPtr model;
    std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> ikFastMoveitStateAdapterPtr;

    void planTrajectoryCallback(const std::shared_ptr<descartes_server::srv::DescartesTrajPlanner::Request> request,
                                std::shared_ptr<descartes_server::srv::DescartesTrajPlanner::Response> response);

    void piecePorteeCallback(const std::shared_ptr<descartes_server::srv::PiecePorteeTrajPlanner::Request> request, 
                             std::shared_ptr<descartes_server::srv::PiecePorteeTrajPlanner::Response> response);

    void addCollisionObjectCallback(const std::shared_ptr<descartes_server::srv::AddCollisionObject::Request> request, 
                             std::shared_ptr<descartes_server::srv::AddCollisionObject::Response> response);

    void removeCollisionObjectCallback(const std::shared_ptr<descartes_server::srv::RemoveCollisionObject::Request> request, 
                             std::shared_ptr<descartes_server::srv::RemoveCollisionObject::Response> response);

    void initPlanningSceneCallback(const std::shared_ptr<descartes_server::srv::InitPlanningScene::Request> request, 
                             std::shared_ptr<descartes_server::srv::InitPlanningScene::Response> response);

    ResultCode traj_planner(const rclcpp::Node::SharedPtr& node, Parameters& parameters, const std::vector<geometry_msgs::msg::Pose>& trajectory_poses,
                trajectory_msgs::msg::JointTrajectory& joint_solutions);
};

#endif // DESCARTES_SERVER_HPP