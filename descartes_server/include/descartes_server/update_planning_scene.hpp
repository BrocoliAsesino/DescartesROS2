#ifndef PLANNING_SCENE_UPDATER_HPP
#define PLANNING_SCENE_UPDATER_HPP

#include <memory>
#include <string>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>

class PlanningSceneUpdater 
{
public:
    PlanningSceneUpdater(std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr);

    void addObjectToPlanningScene(std::string collision_object_id);
    void addMeshObjectToPlanningScene(std::string collision_object_id, const shape_msgs::msg::Mesh& mesh, geometry_msgs::msg::Pose& object_pose);
    void removeObjectFromPlanningScene(std::string collision_object_id);

private:
    moveit_msgs::msg::CollisionObject collision_object;
    std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr_;
    planning_scene::PlanningScenePtr planning_scene_;
};

#endif // PLANNING_SCENE_UPDATER_HPP
