cmake_minimum_required(VERSION 2.8.3)
project(knowrob_poller)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  json_prolog
  std_msgs
  geometry_msgs
  iai_pepper_demo_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES knowrob_poller
  CATKIN_DEPENDS roscpp json_prolog std_msgs geometry_msgs iai_pepper_demo_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}/knowrob_poller.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
