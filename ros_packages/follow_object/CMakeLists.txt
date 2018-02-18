cmake_minimum_required(VERSION 2.8.3)
project(follow_object)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
learning_follow
)
find_package(PkgConfig)
pkg_check_modules(GAML gaml-libsvm REQUIRED)

catkin_package(
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(follow_object src/follow_object.cpp src/follow_object_gaml.cpp)
target_include_directories(follow_object PUBLIC ${GAML_INCLUDE_DIRS})
target_compile_options    (follow_object PUBLIC ${GAML_CFLAGS_OTHER} -std=c++11)
target_link_libraries     (follow_object ${catkin_LIBRARIES} ${GAML_LIBRARIES})
add_dependencies          (follow_object follow_object_generate_messages_cpp)

