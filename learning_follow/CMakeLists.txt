cmake_minimum_required(VERSION 2.8.3)
project(learning_follow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PkgConfig)
pkg_check_modules(GAML gaml-libsvm REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable            (learnergaml src/learning_follow.cpp src/learnergaml.cpp  src/generationRandom.cpp)
target_include_directories(learnergaml PUBLIC ${GAML_INCLUDE_DIRS})
target_compile_options    (learnergaml PUBLIC ${GAML_CFLAGS_OTHER} -std=c++11)
target_link_libraries     (learnergaml ${catkin_LIBRARIES} ${GAML_LIBRARIES})
add_dependencies          (learnergaml learnergaml_generate_messages_cpp)


add_dependencies      (getR       getR_generate_messages_cpp)
add_dependencies      (getThetas  getThetas_generate_messages_cpp)
add_dependencies      (moveThetas moveThetas_generate_messages_cpp)


