cmake_minimum_required(VERSION 3.0.2)
project(neura_motion_planning_challenge)

## Compile as C++17 for std::optional support
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  roscpp
  std_msgs
  std_srvs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface roscpp std_msgs std_srvs trajectory_msgs
  #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

# Build the motion planning library (no main function)
add_library(motion_planning_lib
  src/Planning/MotionPlanning.cpp
  src/Trajectory/TrajectoryIO.cpp
  src/Trajectory/TrajectoryParser.cpp
  src/Trajectory/TrajectoryVisualizer.cpp
  src/DataStructure/PlanMetaData.cpp
  src/DataStructure/CartMotionPlanningData.cpp
  src/Utility/ValidatorAndOptimizer.cpp
  src/Utility/PlanComparator.cpp
  src/Sampling/UniformSampler.cpp
  src/InverseKinematics/JacobianIKSolver.cpp
)
add_dependencies(motion_planning_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planning_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  /opt/ros/noetic/lib/libmoveit_trajectory_processing.so
)

# Build task 1 executable
add_executable(task_one src/executables/Task_One.cpp)
add_dependencies(task_one ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_one
  motion_planning_lib
  ${catkin_LIBRARIES}
)

# Build task 2 executable (trajectory analysis/validation)
add_executable(task_two src/executables/Task_Two.cpp)
add_dependencies(task_two ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_two
  motion_planning_lib
  ${catkin_LIBRARIES}
)

# Build task optional executable (numerical Cartesian path planning)
add_executable(task_optional src/executables/Task_Optional.cpp)
add_dependencies(task_optional ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_optional
  motion_planning_lib
  ${catkin_LIBRARIES}
)
