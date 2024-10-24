cmake_minimum_required(VERSION 3.5)
project(descartes_utilities)

add_compile_options(-std=c++17 -Wall -Wextra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(descartes_core REQUIRED)
find_package(descartes_trajectory REQUIRED)
find_package(trajectory_msgs REQUIRED)



add_library(${PROJECT_NAME}
  src/ros_conversions.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC include)
    
ament_target_dependencies(${PROJECT_NAME}
  descartes_core
  descartes_trajectory
  trajectory_msgs
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
  descartes_core
  descartes_trajectory
  trajectory_msgs
)

ament_package()

