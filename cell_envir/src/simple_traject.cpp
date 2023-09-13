#include <moveit/move_group_interface/move_group_interface.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"


class FollowJointTrajectory {
    public:
        FollowJointTrajectory(std::shared_ptr<rclcpp::Node> node_for_movegroup) : move_group_arm(node_for_movegroup, "arm_group"){

        }

        bool plan_trajectory(geometry_msgs::msg::Pose& target_pose){
            move_group_arm.setStartStateToCurrentState();
            move_group_arm.setPoseTarget(target_pose);
            
            auto error_code = move_group_arm.plan(plan);
            std::cout << "Already called plan method " << std::endl << std::flush;
            
            if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
            {
                std::cout << "Planning Failed! " << std::endl << std::flush;
                return false;
            }

            else {return true; }
        }

        bool execute_trajectory(){

            auto error_code = move_group_arm.execute(plan);
        
            std::cout << "The error code from the method EXECUTE is: " << error_code.val << std::endl << std::flush;


            if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
            {
                std::cout << "Execution Failed ! " << std::endl <<std::flush;
                return false;
            }

            else {return true; }
        }


    private:
        moveit::planning_interface::MoveGroupInterface move_group_arm;
        moveit::planning_interface::MoveGroupInterface::Plan plan;



};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_functionalities_service_node");

    FollowJointTrajectory motion_planning_instance(node);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.87572;
    target_pose.position.y = -0.157824;
    target_pose.position.z = 1.5;

    motion_planning_instance.plan_trajectory(target_pose);

    // rclcpp::spin_some(node);

    motion_planning_instance.execute_trajectory();

    rclcpp::shutdown();
    return 0;   
}