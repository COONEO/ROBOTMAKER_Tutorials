cmake_minimum_required(VERSION 2.8.3)
project(pose_initial)

find_package(catkin REQUIRED COMPONENTS tf nav_msgs sensor_msgs)
#find_package(PCL 1.2 REQUIRED)
	
catkin_package(
 # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS system_lib tf nav_msgs sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
  SYSTEM $ENV{HAF_LIBS}/include/boost1_54
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#)


add_executable(${PROJECT_NAME}
  src/pose_initial.cpp
 )
 

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb3")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(${PROJECT_NAME}
#    ros_messages_gencpp
#    ros_messages_blackbox_gen_cpp
#    ${catkin_EXPORTED_TARGETS}
#    ${CMAKE_CXX_FLAGS}
#)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_road_model_loader.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
