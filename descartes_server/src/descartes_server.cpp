#include "descartes_server/descartes_server.hpp"
// #include "descartes_server/descartes_server_utils.hpp"
#include "descartes_server/custom_cost_function.hpp"
#include "descartes_server/update_planning_scene.hpp"

/*
  -------------------------------------------------------- Trajectory Planner  --------------------------------------------------------------------
*/

ResultCode TrajectoryPlannerServerClass::traj_planner(const rclcpp::Node::SharedPtr& node, Parameters& parameters, const std::vector<geometry_msgs::msg::Pose>& trajectory_poses,
                trajectory_msgs::msg::JointTrajectory& joint_solutions)
{
    joint_solutions.points.clear();
    // Before calling this method, the planning scene must be initialize -> just call the service "descartes_initialize_planning_scene"
    if (!planning_scene_init_){
      RCLCPP_ERROR(node->get_logger(), "Must initialize descartes planning scene first, call service /descartes_initialize_planning_scene");
      return ResultCode::FAILED_TO_INITIALIZE_ROBOT_MODEL;
    }

    logParameters(parameters);

    size_t dof = ikFastMoveitStateAdapterPtr->getJointModelGroup()->getActiveJointModels().size();
    ikFastMoveitStateAdapterPtr->updateTcpFrame(parameters.variable_tcp, parameters.ikfast_tool_frame);
    // std::cout << "Number of active joints in the group: " << dof << std::endl;


    // ---------------------------------- for the CustomCostFunction -----------------------------------
    CustomCostFunctionClass custom_cost_function(ikFastMoveitStateAdapterPtr, parameters);    
    descartes_planner::CostFunction cost_fn = custom_cost_function.getCostFunction();

    // ---------------------------------- For testing collsion checking -----------------------------------
    // custom_cost_function.addBoxToPlanningScene();


    model->setCheckCollisions(true);
    RCLCPP_INFO(node->get_logger(), "Collisions status: %s " , model->getCheckCollisions() ? "true" : "false");

    // Instantiate the Planner
    descartes_planner::DensePlanner planner;

    // Initialize the Planner
    if (!planner.initialize(model, cost_fn, parameters.costFactorManipulability, parameters.velocityLimitFactor))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize planner");
    return ResultCode::FAILED_TO_INITIALIZE_PLANNER;  
  }
  
  // Create the Descartes Trajectory Points from a pose vector
  std::vector<descartes_core::TrajectoryPtPtr> descartes_traject_pts_;
  waypointsToDescartesTrajPts(trajectory_poses, parameters, descartes_traject_pts_);


  // Add initial and final config if defined
  if (parameters.initial_config.size() == dof) {addInitialConfig(parameters.initial_config, descartes_traject_pts_, parameters.time_to_init_config);} 
  else {RCLCPP_WARN(node->get_logger(), "Initial configuration not set ... skipping addInitialConfig.");}

  if (parameters.final_config.size() == dof) {addFinalConfig(parameters.final_config, descartes_traject_pts_, parameters.time_to_final_config);} 
  else {RCLCPP_WARN(node->get_logger(), "Final configuration not set ... skipping addFinalConfig.");}
    
  
  /**@brief planPath()  
 * This function takes your input trajectory and expands it into a large kinematic "graph". Failures at this point 
 * indicate that the input path may not have solutions at a given point (because of reach/collision) or has two 
 * points with no way to connect them.
 */
  if (!planner.planPath(descartes_traject_pts_))
  {
    RCLCPP_ERROR(node->get_logger(), "Could not solve for a valid path, reasons are either invalid poses or no way to tranverse the graph. Look up for reason.");
    return ResultCode::FAILURE_SOLVING_VALID_PATH;
  }

  else
  {
    RCLCPP_DEBUG(node->get_logger(), "Path solved!");
  }

   /**@brief getPath()  
 * After expanding the graph, we now call 'getPath()' which searches the graph for a minimum cost path and returns
 * the result. Failures here (assuming planPath was good) indicate that your path has solutions at every waypoint 
 * but constraints prevent a solution through the whole path. Usually this means a singularity is hanging out in the
 * middle of your path: the robot can solve all the points but not in the same arm configuration.
 */

  std::vector<descartes_core::TrajectoryPtPtr> result;
  if (!planner.getPath(result))
  {
    RCLCPP_ERROR(node->get_logger(), "Could not retrieve path");
    return ResultCode::FAILURE_RETRIEVING_PATH;
  }

  else
  {
    RCLCPP_DEBUG(node->get_logger(), "Path retrieved!");    
  }

  
  // -------------------------------------------------------------------------------------------------------------------------------
  // TODO: get from params server : 
  const moveit::core::JointModelGroup* joint_group = ikFastMoveitStateAdapterPtr->getJointModelGroup();
  std::vector<std::string> names;

  for (int i = 0 ; i < ikFastMoveitStateAdapterPtr->getDOF() ; i++){
    names.push_back(joint_group->getJointModelNames()[i]);
    // std::cout << names[i] << std::endl; 

  }
  
  // Create a JointTrajectory
  joint_solutions.joint_names = names;

  // Define a default velocity. Descartes points without specified timing will use this value to limit the
  // fastest moving joint. This usually affects the first point in your path the most.
  // const static double default_joint_vel = 0.5; // rad/s 
  // NOTE :: NOW WE PASS IT IN THE PARAMETERS, DEFAULT STILL 0.5 rad/s

  if (!descartes_utilities::toRosJointPoints(*model, result, parameters.pure_rotation_velocity, joint_solutions.points))
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to convert Descartes trajectory to joint points");
    return ResultCode::FAILURE_CONVERTING_TO_ROS_PTS;
  }


  for (auto& point : joint_solutions.points)
    {
        point.effort.clear();
    }

  if (result.size() < trajectory_poses.size()){
    return ResultCode::PARTIAL_TRAJECTORY_SOLUTION;
  }

  return ResultCode::SUCCESFUL_PLANNING;
}

/*
  -------------------------------------------------------- Class Definition --------------------------------------------------------------------
*/

TrajectoryPlannerServerClass::TrajectoryPlannerServerClass() : Node("descartes_planner_server_node")
{

  //These two not needed for now
  this->declare_parameter("ikfast_base_frame",  "robot_base");
  this->declare_parameter("ikfast_tool_frame",  "tcp");

  this->declare_parameter("robot_description_name", "robot_description");
  this->declare_parameter("group_name",             "default_group");
  this->declare_parameter("world_frame",            "world");
  this->declare_parameter("tcp_frame",              "tcp");

  this->declare_parameter("costFactorCSpaceDistance",          1.0);
  this->declare_parameter("costFactorDistanceLimits",          1.0);
  this->declare_parameter("costFactorManipulability",          1.0);
  this->declare_parameter("costFactorNearestCollision",        1.0);
  this->declare_parameter("costFactorCostChangePose",          1.0);

  this->declare_parameter("jointVelocityLimitFactor",          1.0);
  this->declare_parameter("pure_rotation_rad_per_sec",         0.5);
    
  serviceDescartesTrajPlanner = this->create_service<descartes_server::srv::DescartesTrajPlanner>("descartes_plan_trajectory",
                    std::bind(&TrajectoryPlannerServerClass::planTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));

  servicePiecePortee = this->create_service<descartes_server::srv::PiecePorteeTrajPlanner>("descartes_piece_portee",
                    std::bind(&TrajectoryPlannerServerClass::piecePorteeCallback, this, std::placeholders::_1, std::placeholders::_2));

  serviceAddCollisionObject = this->create_service<descartes_server::srv::AddCollisionObject>("descartes_add_collision_object",
                    std::bind(&TrajectoryPlannerServerClass::addCollisionObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

  serviceRemoveCollisionObject = this->create_service<descartes_server::srv::RemoveCollisionObject>("descartes_remove_collision_object",
                    std::bind(&TrajectoryPlannerServerClass::removeCollisionObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

  serviceInitPlanningScene = this->create_service<descartes_server::srv::InitPlanningScene>("descartes_initialize_planning_scene",
                    std::bind(&TrajectoryPlannerServerClass::initPlanningSceneCallback, this, std::placeholders::_1, std::placeholders::_2));

  model = std::make_shared<descartes_moveit::IkFastMoveitStateAdapter>();
  ikFastMoveitStateAdapterPtr = std::dynamic_pointer_cast<descartes_moveit::IkFastMoveitStateAdapter>(model);
}

/*
  -------------------------------------------------------- Service Servers Definitions --------------------------------------------------------------------
*/

void TrajectoryPlannerServerClass::planTrajectoryCallback(const std::shared_ptr<descartes_server::srv::DescartesTrajPlanner::Request> request, std::shared_ptr<descartes_server::srv::DescartesTrajPlanner::Response> response)
{

  parameters_.isPiecePortee = false;

  parameters_.orientation_offset_x = request -> orientation_offset_x;
  parameters_.orientation_offset_y = request -> orientation_offset_y;
  parameters_.orientation_offset_z = request -> orientation_offset_z;
  parameters_.position_increment = request -> position_increment;
  parameters_.orientation_increment = request -> orientation_increment;
  parameters_.velocity =  request -> velocity; 
  parameters_.initial_config = request -> initial_configuration;
  parameters_.final_config = request -> final_configuration;
  parameters_.time_from_start = request ->time_from_start;
  parameters_.time_to_init_config = request ->time_to_init_config;
  parameters_.time_to_final_config = request ->time_to_final_config;
  
  
  this->get_parameter_or("robot_description_name", parameters_.robot_description, std::string("robot_description"));
  this->get_parameter_or("group_name", parameters_.group_name, std::string("default_group"));
  this->get_parameter_or("world_frame", parameters_.world_frame, std::string("world"));
  this->get_parameter_or("tcp_frame", parameters_.tcp_frame, std::string("default_tool_frame"));
  this->get_parameter_or("ikfast_tool_frame", parameters_.ikfast_tool_frame, std::string("tool0"));
  
  if (!(request->tcp_name.empty())){
    parameters_.variable_tcp = request->tcp_name;
  }
  else{parameters_.variable_tcp = parameters_.tcp_frame;}

  this->get_parameter("costFactorCSpaceDistance",          parameters_.costFactorCSpaceDistance);
  this->get_parameter("costFactorDistanceLimits",          parameters_.costFactorDistanceLimits);
  this->get_parameter("costFactorManipulability",          parameters_.costFactorManipulability);
  this->get_parameter("costFactorNearestCollision",        parameters_.costFactorNearestCollision);
  this->get_parameter("costFactorCostChangePose",          parameters_.costFactorCostChangePose);

  this->get_parameter("jointVelocityLimitFactor",          parameters_.velocityLimitFactor);
  this->get_parameter_or("pure_rotation_rad_per_sec",      parameters_.pure_rotation_velocity, 0.5);


  //pass the same node the class inherits from
  response -> result_code = static_cast<int>(TrajectoryPlannerServerClass::traj_planner(shared_from_this(), parameters_, 
                                            request->trajectory_poses, joint_solutions));
  response -> joint_solutions = joint_solutions;
  
}

void TrajectoryPlannerServerClass::initPlanningSceneCallback(const std::shared_ptr<descartes_server::srv::InitPlanningScene::Request> request, std::shared_ptr<descartes_server::srv::InitPlanningScene::Response> response){
    this->get_parameter_or("robot_description_name", this->parameters_.robot_description, std::string("robot_description"));
    this->get_parameter_or("group_name", this->parameters_.group_name, std::string("default_group"));
    this->get_parameter_or("world_frame", this->parameters_.world_frame, std::string("world"));
    this->get_parameter_or("tcp_frame", this->parameters_.tcp_frame, std::string("default_tool_frame"));

    const rclcpp::Node::SharedPtr node = shared_from_this();

    // This constructor is different from the one in Descartes1, with the addition of a parameter to point at a rclcpp::Node 
    if (!model->initialize(node, this->parameters_.robot_description, this->parameters_.group_name, this->parameters_.world_frame, this->parameters_.tcp_frame))
    {
        RCLCPP_ERROR(node->get_logger(),"Could not initialize robot model");
        response -> init_status = false;
        return;
    }

    planning_scene_init_ = true;
    response -> init_status = planning_scene_init_; 
}

void TrajectoryPlannerServerClass::addCollisionObjectCallback(const std::shared_ptr<descartes_server::srv::AddCollisionObject::Request> request, std::shared_ptr<descartes_server::srv::AddCollisionObject::Response> response){
    PlanningSceneUpdater updatePlanningSceneInstance(ikFastMoveitStateAdapterPtr);

    updatePlanningSceneInstance.addMeshObjectToPlanningScene(request->uuid, request->mesh_msg, request->pose);

    response -> addition_status = true; 
    std::cout << "Collision object " << request->uuid << " added" << std::endl;
}

void TrajectoryPlannerServerClass::removeCollisionObjectCallback(const std::shared_ptr<descartes_server::srv::RemoveCollisionObject::Request> request, std::shared_ptr<descartes_server::srv::RemoveCollisionObject::Response> response){
    PlanningSceneUpdater updatePlanningSceneInstance(ikFastMoveitStateAdapterPtr);
    
    updatePlanningSceneInstance.removeObjectFromPlanningScene(request->collision_object_id);

    response -> removal_status = true; 
    std::cout << "Collision object " << request->collision_object_id << " removed" << std::endl;
}

void TrajectoryPlannerServerClass::piecePorteeCallback(const std::shared_ptr<descartes_server::srv::PiecePorteeTrajPlanner::Request> request, 
                                                              std::shared_ptr<descartes_server::srv::PiecePorteeTrajPlanner::Response> response)
{
  parameters_.isPiecePortee = true;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::sleep_for(std::chrono::seconds(1));
  
  parameters_.orientation_offset_x = request -> orientation_offset_x;
  parameters_.orientation_offset_y = request -> orientation_offset_y;
  parameters_.orientation_offset_z = request -> orientation_offset_z;
  parameters_.position_increment = request -> position_increment;
  parameters_.orientation_increment = request -> orientation_increment;
  parameters_.velocity =  request -> velocity; 
  parameters_.initial_config = request -> initial_configuration;
  parameters_.final_config = request -> final_configuration;

  
  this->get_parameter_or("robot_description_name", parameters_.robot_description, std::string("robot_description"));
  this->get_parameter_or("group_name", parameters_.group_name, std::string("default_group"));
  this->get_parameter_or("world_frame", parameters_.world_frame, std::string("world"));
  this->get_parameter_or("tcp_frame", parameters_.tcp_frame, std::string("default_tool_frame"));

  this->get_parameter("costFactorCSpaceDistance",          parameters_.costFactorCSpaceDistance);
  this->get_parameter("costFactorDistanceLimits",          parameters_.costFactorDistanceLimits);
  this->get_parameter("costFactorManipulability",          parameters_.costFactorManipulability);
  this->get_parameter("costFactorNearestCollision",        parameters_.costFactorNearestCollision);
  this->get_parameter("costFactorCostChangePose",          parameters_.costFactorCostChangePose);

  // needed transforms: 
  parameters_.transform_tool_base_.transform = request->gripping_pose_to_cad_frame;

  try {
    // Transform from the tcp_frame to the CAD's reference
    // parameters_.transform_tool_base_ = tf_buffer_->lookupTransform(parameters_.tcp_frame, request->cad_link, tf2::TimePointZero);
    parameters_.wobj_pt = tf_buffer_->lookupTransform("world", request->stationary_pt_link, tf2::TimePointZero);
  } 
  
  catch (tf2::TransformException &ex) {
    parameters_.wobj_pt.transform = request->world_to_stationary_pt;
    RCLCPP_ERROR(this->get_logger(),"DID NOT WORK %s, using the geometry_msgs/Transform", ex.what());
  }

  //pass the same node the class inherits from
  response -> result_code = static_cast<int>(traj_planner(shared_from_this(), parameters_, request->trajectory_poses, joint_solutions));
  response -> joint_solutions = joint_solutions;
}


/*
  -------------------------------------------------------- main --------------------------------------------------------------------
*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<TrajectoryPlannerServerClass>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}





