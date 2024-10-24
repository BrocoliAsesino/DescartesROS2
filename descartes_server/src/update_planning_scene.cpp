#include "descartes_server/update_planning_scene.hpp"

PlanningSceneUpdater::PlanningSceneUpdater(std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr)
    : IkFastMoveitStateAdapterPtr_(IkFastMoveitStateAdapterPtr)
{
    planning_scene_ = IkFastMoveitStateAdapterPtr_->getPlanningScene();
}

void PlanningSceneUpdater::addObjectToPlanningScene(std::string collision_object_id)
{
    // Create a CollisionObject
    collision_object.header.frame_id = planning_scene_->getPlanningFrame();
    collision_object.id = collision_object_id; 

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[0] = 0.1;  
    box_primitive.dimensions[1] = 0.03;  
    box_primitive.dimensions[2] = 0.1;  

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 1.0;
    box_pose.position.y = 0.1;
    box_pose.position.z = 1.2;
    box_pose.orientation.w = 1.0;  

    collision_object.primitives.push_back(box_primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene of movegroup
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
    // Add the collision object to the planning scene of descartes
    planning_scene_->processCollisionObjectMsg(collision_object);
}

void PlanningSceneUpdater::addMeshObjectToPlanningScene(std::string collision_object_id, const shape_msgs::msg::Mesh& mesh, geometry_msgs::msg::Pose& object_pose)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_scene_->getPlanningFrame();
    collision_object.id = collision_object_id; 

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene of movegroup
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // Add the collision object to the planning scene of Descartes
    planning_scene_->processCollisionObjectMsg(collision_object);
}

void PlanningSceneUpdater::removeObjectFromPlanningScene(std::string collision_object_id)
{
    collision_object.header.frame_id = planning_scene_->getPlanningFrame();
    collision_object.id = collision_object_id; 
    collision_object.operation = collision_object.REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // Remove the collision object to the planning scene of Descartes
    planning_scene_->processCollisionObjectMsg(collision_object);
}
