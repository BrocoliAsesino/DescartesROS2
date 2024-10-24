cmake_minimum_required(VERSION 3.5)
project(descartes_core)

# Compile options
add_compile_options(-std=c++17 -Wall -Wextra)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
#find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


#### Create library target ####
add_library(${PROJECT_NAME}
  src/trajectory_id.cpp
)

#### Include Header files to the library target ####
target_include_directories(${PROJECT_NAME}
    PUBLIC include)

#### Link dependencies to library target ####
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  rclcpp
  moveit_core
)



#### Install library and headers ####
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#### Export dependencies for downstream packages ####
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  moveit_core
  Boost
  Eigen3
)

# Package the project
ament_package()
