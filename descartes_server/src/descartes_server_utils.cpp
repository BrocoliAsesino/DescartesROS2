#include "descartes_server/descartes_server_utils.hpp"
/*
  -----------------------------------------------|                      |-----------------------------------------------------------
  -----------------------------------------------| Function Definitions |-----------------------------------------------------------
  -----------------------------------------------|                      |-----------------------------------------------------------
*/

/*
  -------------------------------------------------------- Initial/Final Configurations  --------------------------------------------------------------------
*/
void logParameters(Parameters& parameters){
  std::cout << "\n" 
            << "Descartes Parametrization : " << "\n"
            << "# Orientation_offset_x: " << parameters.orientation_offset_x << "  "
            << "Orientation_offset_y: " << parameters.orientation_offset_y << "  "
            << "Orientation_offset_z: " << parameters.orientation_offset_z << "\n"
            << "# Position_offset_x: " << parameters.position_offset_x << "  "
            << "Position_offset_y: " << parameters.position_offset_y << "  "
            << "Position_offset_z: " << parameters.position_offset_z << "\n"
            << "# Linear velocity: " << parameters.velocity << "\n"
            << "# Orientation_increment: " << parameters.orientation_increment << "  "
            << "Position_increment: " << parameters.position_increment << "\n"
            << "# robot_description: " << parameters.robot_description << "\n"
            << "# group_name: " << parameters.group_name << "\n"
            << "# world_frame: " << parameters.world_frame << "\n"
            << "# tcp_frame: " << parameters.tcp_frame << "\n"
            << "# costFactorCSpaceDistance: " << parameters.costFactorCSpaceDistance << "  "
            << "costFactorDistanceLimits: " << parameters.costFactorDistanceLimits << "  "
            << "costFactorManipulability: " << parameters.costFactorManipulability << "  "
            << "costFactorNearestCollision: " << parameters.costFactorNearestCollision << "  "
            << "costFactorCostChangePose: " << parameters.costFactorCostChangePose << "\n"
            << "# velocityLimitFactor: " << parameters.velocityLimitFactor << "\n" << std::endl;
}

std::string ResultComment(ResultCode type) {
   switch(type) {
      case ResultCode::FAILED_TO_INITIALIZE_ROBOT_MODEL: return "Failed to initialize robot model.";
      case ResultCode::FAILED_TO_INITIALIZE_PLANNER: return "Failed to initialize planner";
      case ResultCode::FAILURE_SOLVING_VALID_PATH: return "Could not solve for a valid path";
      case ResultCode::FAILURE_RETRIEVING_PATH: return "Could not retrieve path";
      case ResultCode::FAILURE_CONVERTING_TO_ROS_PTS: return "Unable to convert Descartes trajectory to ROS joint points";
      case ResultCode::PARTIAL_TRAJECTORY_SOLUTION: return "Could not solve the IK of full trajectory, some points in collision or out of reach, passing partial trajectory";
      
      default: return "unexpected result code";
   }
}

double degreesToRadians(double degrees){
  return degrees * M_PI / 180.0;
}

double RadiansToDegrees(double radians){
  return radians * 180 / M_PI ;
}

std::pair<double, double> computeBounds(double nominal_value, double offset){
  return std::make_pair(nominal_value - offset, nominal_value + offset);
}

bool computeTiming(const geometry_msgs::msg::Pose& waypoint, const geometry_msgs::msg::Pose& next_waypoint, double& velocity, double& dt){
  
    double dx = next_waypoint.position.x - waypoint.position.x;
    double dy = next_waypoint.position.y - waypoint.position.y;
    double dz = next_waypoint.position.z - waypoint.position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    dt = distance / velocity;

    if (dt < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("descartes_traj_planner"), "Invalid TimingConstraint. Cannot create Cartesian point.");
        return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("descartes_traj_planner"), "the distance is %lf, the time thus, for a velocity of %lf is: %lf", distance, velocity, dt);

    return true;
}

Eigen::Isometry3d transformStampedToIsometry(const geometry_msgs::msg::TransformStamped& tf)
{
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();

    // Extract translation
    iso.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                        tf.transform.translation.y,
                                        tf.transform.translation.z);

    // Convert rotation to Eigen Quaternion and then set it to the isometry
    Eigen::Quaterniond quat(tf.transform.rotation.w,
                            tf.transform.rotation.x,
                            tf.transform.rotation.y,
                            tf.transform.rotation.z);
    iso.linear() = quat.toRotationMatrix();

    return iso;
}

/*
  -------------------------------------------------------- Initial/Final Configurations  --------------------------------------------------------------------
*/

bool addInitialConfig(std::vector<double>& initial_config, std::vector<descartes_core::TrajectoryPtPtr>& descartesWayPoints, double& time_to_init_config){
    // std::cout << "initial config:::::" << initial_config[0] << " " << initial_config[1] << " " << initial_config[2]  << " " << initial_config[3] << " "
    // << initial_config[4] << " " << initial_config[5]  << std::endl;

    // std::cout << "which in degrees is:::::" << RadiansToDegrees(initial_config[0]) << " " << RadiansToDegrees(initial_config[1]) << " " << 
    // RadiansToDegrees(initial_config[2])  << " " << RadiansToDegrees(initial_config[3]) << " " << RadiansToDegrees(initial_config[4]) << " " << 
    // RadiansToDegrees(initial_config[5])  << std::endl;

    descartes_core::TimingConstraint timing(time_to_init_config); // time from current config of the robot to initial_config 
    descartesWayPoints.insert(descartesWayPoints.begin(), std::make_shared<descartes_trajectory::JointTrajectoryPt>(initial_config, timing));
    return true;
}

bool addFinalConfig(std::vector<double>& final_config, std::vector<descartes_core::TrajectoryPtPtr>& descartesWayPoints, double& time_to_final_config){
    descartes_core::TimingConstraint timing(time_to_final_config);
    descartesWayPoints.push_back(std::make_shared<descartes_trajectory::JointTrajectoryPt>(final_config, timing));
    return true;
}

/*
  -------------------------------------------------------- Waypoints to Descartes Trajectory Points --------------------------------------------------------------------
*/

bool waypointsToDescartesTrajPts(const std::vector<geometry_msgs::msg::Pose>& trajectory_poses, Parameters& parameters, std::vector<descartes_core::TrajectoryPtPtr>& descartes_traject_pts_){

    descartes_core::TrajectoryPtPtr decart_waypoint_sol;

    for (size_t i = 0; i < trajectory_poses.size(); ++i){
        
        double dt;

        if (i == 0){
          dt = parameters.time_from_start; // this is the time that would take to get to the first point in the trajectory from the initial pose if set or from the 
                    // current pose of the robot to the first point. If the initial pose = the first pose of the robot, it will take 2s to start the trajectory
        }

        else if (i < trajectory_poses.size()) {
            computeTiming(trajectory_poses[i-1], trajectory_poses[i], parameters.velocity, dt);  // Compute timing between current waypoint and the next one
        }
        
        Eigen::Isometry3d waypoint;
        tf2::fromMsg(trajectory_poses[i], waypoint);

        if (!buildCartesianPoint(waypoint, dt, parameters, decart_waypoint_sol)){
            RCLCPP_ERROR(rclcpp::get_logger("test"), "Could not create Toleranced Frame");
        }

        else { descartes_traject_pts_.push_back(decart_waypoint_sol);}
    }

  return true; 
}

bool buildCartesianPoint(const Eigen::Isometry3d& waypoint, double dt, Parameters& parameters, descartes_core::TrajectoryPtPtr& decart_waypoint_sol){

    descartes_trajectory::TolerancedFrame frame;
    descartes_core::TimingConstraint timing(dt);

    if (!parameters.orientation_offset_x && !parameters.orientation_offset_y && (parameters.orientation_offset_z == 180 || parameters.orientation_offset_z == 180.0)){
      
      // std::cout << "Axial Symmetric  CONSTRUCTOR" << std::endl;
      decart_waypoint_sol = std::make_shared<descartes_trajectory::AxialSymmetricPt>(waypoint, degreesToRadians(parameters.orientation_increment), 
                                descartes_trajectory::AxialSymmetricPt::Z_AXIS, timing);

      return true;
    }

    // No rotation or position offset
    // No increment. Necessary, if not, the lower bound frame would be used for planning not the nominal. 
    // it is just the way it is coded in cart_trajectory_py.cpp in the "uniform" function, since the 
    // increment is 0 and it creates the frames by doing lower_bound + increment)
    else if ((!parameters.orientation_offset_x && !parameters.orientation_offset_y && !parameters.orientation_offset_z && !parameters.position_offset_x && !
        parameters.position_offset_y && !parameters.position_offset_z) || (parameters.orientation_increment == 0 && parameters.position_increment == 0)){

      frame = descartes_trajectory::TolerancedFrame(waypoint);
    }

    
  
    else {
      
      Eigen::Vector3d t = waypoint.translation();
      Eigen::Matrix3d m_rotation = waypoint.rotation();
      Eigen::Vector3d v_rotation = m_rotation.eulerAngles(0, 1, 2); 

      // Convert orientation offset from degrees to radians
      Eigen::Vector3d orientation_offset_radians;
      orientation_offset_radians(0) = degreesToRadians(parameters.orientation_offset_x);
      orientation_offset_radians(1) = degreesToRadians(parameters.orientation_offset_y);
      orientation_offset_radians(2) = degreesToRadians(parameters.orientation_offset_z);

      // Compute position bounds
      auto [x_pos_lower, x_pos_upper] = computeBounds(t(0), parameters.position_offset_x);
      auto [y_pos_lower, y_pos_upper] = computeBounds(t(1), parameters.position_offset_y);
      auto [z_pos_lower, z_pos_upper] = computeBounds(t(2), parameters.position_offset_z);

      // Compute orientation bounds
      auto [x_orient_lower, x_orient_upper] = computeBounds(v_rotation(0), orientation_offset_radians(0));
      auto [y_orient_lower, y_orient_upper] = computeBounds(v_rotation(1), orientation_offset_radians(1));
      auto [z_orient_lower, z_orient_upper] = computeBounds(v_rotation(2), orientation_offset_radians(2));

      // Initialize PositionTolerance and OrientationTolerance object variables
      descartes_trajectory::PositionTolerance pos_tol(x_pos_lower, x_pos_upper, y_pos_lower, y_pos_upper, z_pos_lower, z_pos_upper);
      descartes_trajectory::OrientationTolerance orient_tol(x_orient_lower, x_orient_upper, y_orient_lower, y_orient_upper, z_orient_lower, z_orient_upper);

      frame = descartes_trajectory::TolerancedFrame(waypoint, pos_tol, orient_tol);
    }

    if (!parameters.isPiecePortee){
      if (!makeCartesianPoint(frame, timing, parameters.orientation_increment, parameters.position_increment, decart_waypoint_sol)){
        RCLCPP_ERROR(rclcpp::get_logger("descartes_traj_planner"), "Could not create the Cartesian Trajectory Point.");
        return false;  
      }
    }

    else {
      if (!makeCartesianPointFullConstructor(parameters.transform_tool_base_, parameters.wobj_pt, frame, parameters.orientation_increment, 
                                              parameters.position_increment, timing, decart_waypoint_sol)){
        RCLCPP_ERROR(rclcpp::get_logger("descartes_traj_planner"), "Could not create the Cartesian Trajectory Point.");
        return false;  
      } 
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("descartes_traj_planner"), "Successfully created the Cartesian Trajectory Point.");
    return true;  
}

bool makeCartesianPoint(descartes_trajectory::TolerancedFrame& tolerancedFrame, descartes_core::TimingConstraint& dt, double& orientation_increment, 
                                                                      double& position_increment, descartes_core::TrajectoryPtPtr& decart_waypoint_sol){

    decart_waypoint_sol = std::make_shared<descartes_trajectory::CartTrajectoryPt>(tolerancedFrame, position_increment, degreesToRadians(orientation_increment), dt);
    
    return true;
}


bool makeCartesianPointFullConstructor(const geometry_msgs::msg::TransformStamped& tool_base, const geometry_msgs::msg::TransformStamped& wobj_pt, 
                                        descartes_trajectory::TolerancedFrame& tolerancedFrame, double& orientation_increment, double& position_increment,
                                        descartes_core::TimingConstraint& dt, descartes_core::TrajectoryPtPtr& decart_waypoint_sol){
    
    const descartes_core::Frame identity_frame(Eigen::Isometry3d::Identity());
    const descartes_core::Frame wobj_base_frame(transformStampedToIsometry(wobj_pt));
    const descartes_core::Frame tool_base_frame(transformStampedToIsometry(tool_base));

    decart_waypoint_sol = std::make_shared<descartes_trajectory::CartTrajectoryPt>(wobj_base_frame, identity_frame, tool_base_frame, tolerancedFrame,
                                                                 position_increment, degreesToRadians(orientation_increment), dt);
    
    return true;
}

