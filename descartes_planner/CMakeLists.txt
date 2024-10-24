cmake_minimum_required(VERSION 3.5)
project(descartes_planner)

# Compile options
add_compile_options(-std=c++17 -Wall -Wextra)

message(STATUS "Processing CMakeLists.txt for descartes_planner")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(descartes_core REQUIRED)
find_package(descartes_trajectory REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "Descartes will use OpenMP")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(WARNING "OpenMP not found!")
endif()


add_library(${PROJECT_NAME}
  src/dense_planner.cpp
  src/ladder_graph_dag_search.cpp
  src/planning_graph.cpp
  src/plugin_init.cpp
  src/sparse_planner.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  rclcpp
  descartes_core
  descartes_trajectory
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
    OpenMP::OpenMP_CXX
)


## OpenMP -> employed to parallelize computationally heavy processes, providing enhanced performance on multi-core processors. 
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
#target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)


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

install(
  FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  rclcpp
  descartes_core
  descartes_trajectory
  pluginlib
)

ament_package()
