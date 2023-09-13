cmake_minimum_required(VERSION 3.5)
project(descartes_core)

# Compile as C++11
add_compile_options(-std=c++14 -Wall -Wextra)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
#find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
 # ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}
  src/trajectory_id.cpp
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_core
)

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBRARIES}
)

# Install library and headers
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

# Export dependencies for downstream packages
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
