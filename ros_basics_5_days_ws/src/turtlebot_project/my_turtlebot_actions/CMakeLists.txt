cmake_minimum_required(VERSION 2.8.3)
project(my_turtlebot_actions)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Here go the packages needed to COMPILE the messages of topic, services and actions.
## in package.xml you have to state them as build
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  nav_msgs
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   record_odom.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   nav_msgs
 )

###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
## State here all the packages that will be needed by someone that executes something from your package
## All the packages stated here must be in the package.xml as exec_depend
catkin_package(
  CATKIN_DEPENDS rospy nav_msgs
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