cmake_minimum_required(VERSION 3.5)
project(descartes_trajectory)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(descartes_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the library
add_library(${PROJECT_NAME}
  src/axial_symmetric_pt.cpp
  src/cart_trajectory_pt.cpp
  src/joint_trajectory_pt.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  descartes_core
  moveit_core
  rclcpp
)

# Install the library and headers
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# If testing is enabled
if(BUILD_TESTING)
  # Add your tests here
endif()

# Finalize the package
ament_package()
