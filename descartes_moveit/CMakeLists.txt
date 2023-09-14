cmake_minimum_required(VERSION 3.5)
project(descartes_moveit)

# Compile options
add_compile_options(-std=c++17 -Wall -Wextra)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(descartes_core REQUIRED)
find_package(descartes_trajectory REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  #${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${descartes_trajectory_INCLUDE_DIRS}
)

# Descartes MoveIt library
add_library(${PROJECT_NAME}
  src/ikfast_moveit_state_adapter.cpp
  src/moveit_state_adapter.cpp
  src/seed_search.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  descartes_core
  descartes_trajectory
  moveit_core
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES moveit_adapter_plugins.xml
        DESTINATION share/ament_index/resource_index/pluginlib/${PROJECT_NAME}/)


# Export dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(descartes_core)
ament_export_dependencies(descartes_trajectory)
ament_export_dependencies(moveit_core)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

