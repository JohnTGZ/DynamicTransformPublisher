cmake_minimum_required(VERSION 3.0.2)
project(dynamic_transform_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  std_msgs
  geometry_msgs

  tf
  tf2
  tf2_ros

  ddynamic_reconfigure
)


################################################
## Declare ROS messages, services and actions ##
################################################
# add_service_files(
#   DIRECTORY
#     srv
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
    std_msgs tf2_geometry_msgs
    tf tf2 tf2_ros  
    dynamic_reconfigure 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamic_transform_publisher src/dynamic_transform_publisher.cpp)
target_link_libraries(dynamic_transform_publisher ${catkin_LIBRARIES})

#EXECUTABLES

#Camera calibration node
add_executable(dynamic_transform_publisher_node src/dynamic_transform_publisher_node.cpp)
target_link_libraries(
  dynamic_transform_publisher_node 
  dynamic_transform_publisher 
  ${catkin_LIBRARIES})
add_dependencies(
  dynamic_transform_publisher_node 
  ${catkin_EXPORTED_TARGETS} ) 

#############
## Install ##
#############

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#   PATTERN ".svn" EXCLUDE
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamic_transform_publisher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#
