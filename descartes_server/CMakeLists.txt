cmake_minimum_required(VERSION 3.8)
project(descartes_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)

find_package(descartes_core REQUIRED)
find_package(descartes_trajectory REQUIRED)
find_package(descartes_moveit REQUIRED)
find_package(descartes_planner REQUIRED)
find_package(descartes_utilities REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(OpenMP)



        ##### Generate the interface #### 

### (Converts interface definitions into source code)
## This function takes interface files (.msg, .srv, .action) and generates 
## the source code needed to use these interfaces in various programming languages (C++, Python, etc.).

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DescartesTrajPlanner.srv"
  "srv/PiecePorteeTrajPlanner.srv"
  "srv/InitPlanningScene.srv"
  "srv/AddCollisionObject.srv"
  "srv/RemoveCollisionObject.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs shape_msgs
)

### Get the typesupport target for C++
## This line generates a CMake target that represents the ROS interface type support in C++. 
## The first argument (cpp_typesupport_target) is a variable that will hold the name of the generated target. 
## The second argument is the package name, and the third argument specifies the type support language ("rosidl_typesupport_cpp" for C++).

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


        ##### Generate the executables #### 

set(SOURCES
    src/descartes_server.cpp
    src/custom_cost_function.cpp
    src/update_planning_scene.cpp
    src/descartes_server_utils.cpp)

add_executable(descartes_trajectory_planner_server ${SOURCES})

target_include_directories(descartes_trajectory_planner_server
    PUBLIC include)

ament_target_dependencies(descartes_trajectory_planner_server
    ament_cmake
    rclcpp
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    descartes_utilities
    geometry_msgs
    tf2
    trajectory_msgs
    tf2_ros
    moveit_msgs
    geometric_shapes
    shape_msgs)

target_link_libraries(descartes_trajectory_planner_server 
  OpenMP::OpenMP_CXX
  ${cpp_typesupport_target}
)


## Info for rosidl_target_interfaces: 
### Usage: to link generated message/service/action interfaces with a specific target
### Arguments:
####  ${PROJECT_NAME} -> The function needs this parameter to find the interfaces that were generated for this package.
####  rosidl_typesupport_cpp -> determines for which language the generated interfaces should be linked.



        ##### Install directories and targets #### 

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch srv src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
    descartes_trajectory_planner_server
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
