#include "descartes_server/descartes_server_utils.hpp"  
#include "descartes_server/custom_cost_function.hpp"

CustomCostFunctionClass::CustomCostFunctionClass(
    std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr, Parameters& params) 
    : IkFastMoveitStateAdapterPtr_(IkFastMoveitStateAdapterPtr),
      robot_state(IkFastMoveitStateAdapterPtr_->getRobotState()),
      margin_(params.margin),
      joint_group_name_(params.group_name),
      tcp_frame_(params.tcp_frame),
      factorC1_(params.costFactorCSpaceDistance),
      factorC2_(params.costFactorDistanceLimits),
      factorC4_(params.costFactorNearestCollision),
      factorC5_(params.costFactorCostChangePose) 
{ 
    joint_model_group = IkFastMoveitStateAdapterPtr_->getJointModelGroup();
    planning_scene = IkFastMoveitStateAdapterPtr_->getPlanningScene();
}

// for testing to remove later
void CustomCostFunctionClass::addBoxToPlanningScene()
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_scene->getPlanningFrame();
    collision_object.id = "box"; 

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
    planning_scene->processCollisionObjectMsg(collision_object);
}

/**@brief Computes the custom cost for a given start and stop joint configuration.
 * The cost is a combination of the C-space distance, log distance to joint limits, and manipulability.
 */
double CustomCostFunctionClass::CustomCostFunction(const double* start, const double* stop)
{
    double cost1, cost2, cost4, cost5;
    
    if (factorC1_ != 0.0) { cost1 = CostCSpaceDistanceL1(start, stop); }
    else { cost1 = 0; }
    
    if (factorC2_ != 0.0) { cost2 = CostCSpaceLogDistanceToLimits(start); }
    else { cost2 = 0; }

    if (factorC4_ != 0.0) { cost4 = CostNearestCollision(start, joint_group_name_); }
    else { cost4 = 0; }

    if (factorC5_ != 0.0) { cost5 = CostChangePose(start, stop, joint_group_name_); }
    else { cost5 = 0; }
    
    double total_cost = factorC1_ * cost1 + factorC2_ * cost2 + factorC4_ * cost4 + factorC5_ * cost5; 

    return total_cost;
}

/**@brief CostCSpaceDistanceL1  
 * Computes the L1 distance (sum of absolute differences) between the start and stop joint configurations.
 */
double CustomCostFunctionClass::CostCSpaceDistanceL1(const double* start, const double* stop)
{
    double cost = 0;

    for (int i = 0; i < IkFastMoveitStateAdapterPtr_->getDOF(); ++i)
    {
        double weight = 1.0 + 0.3 * i; 
        cost += weight * std::abs(stop[i] - start[i]);
    }

    return cost;
}

/**@brief CostCSpaceLogDistanceToLimits  
 * Computes the cost based on the logarithmic distance of the joint values to their respective limits.
 */
double CustomCostFunctionClass::CostCSpaceLogDistanceToLimits(const double* start)
{
    double cost = 0;
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getJointModels();

    for(size_t i = 0; i < joint_models.size() - 1; i++) {
        const auto& joint_bounds = joint_models[i]->getVariableBounds();
        double min_position = joint_bounds[i].min_position_;
        double max_position = joint_bounds[i].max_position_;

        if (i == 3 || i == 5) {
            min_position -= 2 * M_PI;
            max_position += 2 * M_PI;
        }
        
        cost += computeForValues(start[i], min_position, max_position);
    }

    return cost;
}

/**@brief Computes the logarithmic distance for a given joint value with respect to its minimum and maximum limits.
 */
double CustomCostFunctionClass::computeForValues(double q, double qmin, double qmax)
{
    if (qmax - q <= margin_ || q - qmin <= margin_)
        return 1000;
    
    double term1 = (qmax - q) / margin_;
    double term2 = (q - qmin) / margin_;

    double ret = 0.5 * (
        std::max(0.0, -std::log(term1)) + 
        std::max(0.0, -std::log(term2))
    );

    return ret;
}

/**@brief CostNearestCollision  
 * Computes the cost based on the nearest collision for a given joint configuration.
 * The cost is inversely proportional to the distance to the nearest collision.
 */
double CustomCostFunctionClass::CostNearestCollision(const double* start, const std::string &joint_group_name)
{
    moveit::core::RobotState startConfig_robot_state = *robot_state;

    startConfig_robot_state.setJointGroupPositions(joint_group_name, start);
    startConfig_robot_state.update();

    collision_detection::CollisionRequest collision_request;
    collision_request.distance = true;
    collision_request.cost = true;

    collision_detection::CollisionResult collision_result;

    planning_scene->checkCollision(collision_request, collision_result, startConfig_robot_state);

    double cost = (1.0 / (1.0 + collision_result.distance));

    return cost;
}

double CustomCostFunctionClass::CostChangePose(const double* start, const double* stop, const std::string &joint_group_name)
{
    moveit::core::RobotState startConfig_robot_state = *robot_state;

    startConfig_robot_state.setJointGroupPositions(joint_group_name, start);
    startConfig_robot_state.update();
    Eigen::Isometry3d start_pose = startConfig_robot_state.getGlobalLinkTransform("tcp");

    startConfig_robot_state.setJointGroupPositions(joint_group_name, stop);
    startConfig_robot_state.update();
    Eigen::Isometry3d end_pose = startConfig_robot_state.getGlobalLinkTransform("tcp");

    Eigen::Vector3d pos_diff_vec = end_pose.translation() - start_pose.translation();
    double pos_diff = pos_diff_vec.norm();

    Eigen::Quaterniond start_quat(start_pose.rotation());
    Eigen::Quaterniond end_quat(end_pose.rotation());
    start_quat.normalize();
    end_quat.normalize();

    double dot_product = start_quat.dot(end_quat);
    dot_product = std::max(-1.0, std::min(1.0, dot_product));

    double angle_diff = 2 * acos(std::abs(dot_product));

    double cost = pos_diff + angle_diff;

    return cost;
}

/**@brief getCostFunction  
 * Returns a lambda function that computes the custom cost for a given start and stop joint configuration.
 */
descartes_planner::CostFunction CustomCostFunctionClass::getCostFunction()
{
    return [this](const double* start, const double* stop) -> double {
        return this->CustomCostFunction(start, stop);
    };
}






    

   
   
