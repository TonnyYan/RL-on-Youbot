cmake_minimum_required(VERSION 2.8.3)
project(commande_bras)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 #add_library(commande_clavier
 #  src/${commande_bras}/commande_clavier.cpp
 #)

## Declare a cpp executable
add_executable(commande_clavier src/commande_clavier.cpp)
target_compile_options(commande_clavier PUBLIC -std=c++11)
add_executable(ecoute_pince src/ecoute_pince.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(commande_clavier commande_bras_generate_messages_cpp)
add_dependencies(ecoute_pince ecoute_pince_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(commande_clavier
  ${catkin_LIBRARIES}
)
target_link_libraries(ecoute_pince
  ${catkin_LIBRARIES}
)
