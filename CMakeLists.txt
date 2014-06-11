cmake_minimum_required(VERSION 2.8.3)
project(graft)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    geometry_msgs
    message_generation
    nav_msgs
    rosconsole
    roscpp
    sensor_msgs
    tf
)

find_package(Eigen REQUIRED COMPONENTS Dense Cholesky)
include_directories(${Eigen_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  GraftState.msg
  GraftControl.msg
  GraftSensorResidual.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs # may not be needed
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime rosconsole roscpp geometry_msgs sensor_msgs nav_msgs tf
  DEPENDS eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare cpp library
add_library(GraftOdometryTopic src/GraftOdometryTopic.cpp)
add_dependencies(GraftOdometryTopic ${PROJECT_NAME}_gencpp)

add_library(GraftImuTopic src/GraftImuTopic.cpp)
add_dependencies(GraftImuTopic ${PROJECT_NAME}_gencpp)

add_library(GraftParameterManager src/GraftParameterManager.cpp)
add_dependencies(GraftParameterManager ${PROJECT_NAME}_gencpp)
target_link_libraries(GraftParameterManager GraftOdometryTopic GraftImuTopic)

add_library(GraftUKFVelocity src/GraftUKFVelocity.cpp)
add_dependencies(GraftUKFVelocity ${PROJECT_NAME}_gencpp)
target_link_libraries(GraftUKFVelocity GraftOdometryTopic GraftImuTopic)

add_library(GraftUKFAttitude src/GraftUKFAttitude.cpp)
add_dependencies(GraftUKFAttitude ${PROJECT_NAME}_gencpp)
target_link_libraries(GraftUKFAttitude GraftOdometryTopic GraftImuTopic)

add_library(GraftUKFAbsolute src/GraftUKFAbsolute.cpp)
add_dependencies(GraftUKFAbsolute ${PROJECT_NAME}_gencpp)
target_link_libraries(GraftUKFAbsolute GraftOdometryTopic GraftImuTopic)

## Declare a cpp executable
add_executable(graft_ukf_velocity src/graft_ukf_velocity.cpp)
target_link_libraries(graft_ukf_velocity GraftUKFVelocity GraftParameterManager GraftOdometryTopic GraftImuTopic ${catkin_LIBRARIES})

add_executable(graft_ukf_attitude src/graft_ukf_attitude.cpp)
target_link_libraries(graft_ukf_attitude GraftUKFAttitude GraftParameterManager GraftOdometryTopic GraftImuTopic ${catkin_LIBRARIES})

add_executable(graft_ukf_absolute src/graft_ukf_absolute.cpp)
target_link_libraries(graft_ukf_absolute GraftUKFAbsolute GraftParameterManager GraftOdometryTopic GraftImuTopic ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS GraftOdometryTopic GraftImuTopic GraftParameterManager GraftUKFVelocity graft_ukf_velocity
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
