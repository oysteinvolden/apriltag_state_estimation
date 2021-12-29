cmake_minimum_required(VERSION 2.8.3)
project(mekf)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    message_filters
    message_generation
    eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    sensor_msgs
    message_filters
    message_generation
    eigen_conversions
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/sensor_message_handler.cpp src/mekf.cpp src/node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
