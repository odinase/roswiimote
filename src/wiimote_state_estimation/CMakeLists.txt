cmake_minimum_required(VERSION 2.8.3)
project(wiimote_state_estimation)

add_compile_options(-std=c++17)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  Eigen3
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs 
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Libraries
add_library(kf src/kf.cpp)
add_dependencies(kf ${catkin_EXPORTED_TARGETS})

## Executables
add_executable(kf_node src/kf_node.cpp)
add_dependencies(kf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_node ${catkin_LIBRARIES} kf)
