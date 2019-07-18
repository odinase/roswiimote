cmake_minimum_required(VERSION 2.8.3)
project(wiimote_state_estimation)

SET(CMAKE_CXX_COMPILER /usr/bin/g++-7)
add_compile_options(-std=c++17 -Wno-ignored-attributes)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
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
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Libraries
add_library(kf src/kf.cpp)
add_dependencies(kf ${catkin_EXPORTED_TARGETS})
#target_compile_options(kf PRIVATE -Wno-ignored-attributes)

add_library(filter_method src/filter_method.cpp)
add_dependencies(filter_method ${catkin_EXPORTED_TARGETS})

## Executables
add_executable(state_estimation_node src/state_estimation_node.cpp)
add_dependencies(state_estimation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_estimation_node ${catkin_LIBRARIES} kf filter_method)
#target_compile_options(state_estimation_node PRIVATE -Wno-ignored-attributes)
