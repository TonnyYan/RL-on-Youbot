cmake_minimum_required(VERSION 2.8.3)
project(traitement_pcl)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  PCL REQUIRED
  tf
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(plane_delete src/plane_delete.cpp)

add_dependencies(plane_delete plane_delete_generate_messages_cpp)

target_link_libraries(plane_delete ${catkin_LIBRARIES} ${PCL_LIBRARIES})



