cmake_minimum_required(VERSION 2.8.3)
project(traitement_pcl)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(plane_delete src/plane_delete.cpp)
 add_dependencies(plane_delete plane_delete_generate_messages_cpp)
 target_link_libraries(plane_delete
   ${catkin_LIBRARIES}
 )
