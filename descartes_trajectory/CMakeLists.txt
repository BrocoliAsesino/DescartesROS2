cmake_minimum_required(VERSION 3.5)
project(descartes_trajectory)



add_compile_options(-std=c++17 -Wall -Wextra)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(descartes_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)


#### Create library target ####
add_library(${PROJECT_NAME}
  src/axial_symmetric_pt.cpp
  src/cart_trajectory_pt.cpp
  src/joint_trajectory_pt.cpp
)

#### Include Header files to the library target ####
target_include_directories(${PROJECT_NAME}
    PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  moveit_core
  descartes_core
  Eigen3
)


# If testing is enabled
if(BUILD_TESTING)
  # Add your tests here
endif()

# Finalize the package
ament_package()
