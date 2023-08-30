cmake_minimum_required(VERSION 3.0.2)
project(bentobot_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# find catkin specific packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  bentobot_mcu_bridge
)

# find external packages
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp bentobot_mcu_bridge
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add Kalman Filter library
add_library(KalmanFilter
  src/KalmanFilter.cpp
)
target_link_libraries(KalmanFilter
  ${EIGEN3_LIBRARIES}
)

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(odom_node ${catkin_EXPORTED_TARGETS})