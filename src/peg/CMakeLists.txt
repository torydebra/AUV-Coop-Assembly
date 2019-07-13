cmake_minimum_required(VERSION 2.8.3)
project(peg)

#to exclude two non existent header from cmat.h
add_definitions(-DCMAT_STATIC_BUFFERS -DCMAT_STANDALONE)


## set list of compile option to later use target_compile_options(my_target INTERFACE ${REPLACED_FLAGS}) in the added files
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Werror -Wextra")
## CMAKE_CXX_FLAGS expects a single string (parameters are separated by spaces)
## while target_compile_options uses a list (parameters are separated by semicolons).
#string(REPLACE " " ";" REPLACED_FLAGS ${CMAKE_CXX_FLAGS})

#add_compile_options(-Wall -pedantic)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  topic_tools
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  peg_msgs
  tf
  tf_conversions
  kdl_parser
  cv_bridge
  image_transport
  pcl_ros

)
#package not catkin
find_package(
  OpenCV REQUIRED
)
find_package(
  VISP REQUIRED
)
include_directories(${VISP_INCLUDE_DIRS})
find_package(
    PCL REQUIRED
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
    #FILES
    #toCoord_msg.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
   #DEPENDENCIES
   #std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES peg_pkg
   CATKIN_DEPENDS
     roscpp 
     rospy 
     std_msgs
     topic_tools
     tf
     tf_conversions
   DEPENDS
     roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
  /usr/local/lib/cmat
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/peg_pkg.cpp
# )
add_library(${PROJECT_NAME}_Conversions src/support/conversions.cpp)
add_library(${PROJECT_NAME}_Formulas src/support/formulas.cpp)
add_library(${PROJECT_NAME}_Prints src/support/prints.cpp)
add_library(${PROJECT_NAME}_Controller src/controller/controller.cpp)
add_library(${PROJECT_NAME}_RobotInterface src/rosInterfaces/robotInterface.cpp)
add_library(${PROJECT_NAME}_RobotVisionInterface src/rosInterfaces/robotVisionInterface.cpp)
add_library(${PROJECT_NAME}_WorldInterface src/rosInterfaces/worldInterface.cpp)
add_library(${PROJECT_NAME}_CoordInterfaceMissMan src/rosInterfaces/coordInterfaceMissMan.cpp)
add_library(${PROJECT_NAME}_CoordInterfaceCoord src/rosInterfaces/coordInterfaceCoord.cpp)
add_library(${PROJECT_NAME}_VisionInterfaceMissMan src/rosInterfaces/visionInterfaceMissMan.cpp)
add_library(${PROJECT_NAME}_VisionInterfaceVision src/rosInterfaces/visionInterfaceVision.cpp)

add_library(${PROJECT_NAME}_KdlHelper src/helper/kdlHelper.cpp)
add_library(${PROJECT_NAME}_JacobianHelper src/helper/jacobianHelper.cpp)
add_library(${PROJECT_NAME}_Logger src/helper/logger.cpp)

add_library(${PROJECT_NAME}_Detector src/vision/detector.cpp)
add_library(${PROJECT_NAME}_Tracker src/vision/tracker.cpp)
add_library(${PROJECT_NAME}_DepthCameraSupport src/vision/depthCameraSupport.cpp)

add_library(${PROJECT_NAME}_CollisionPropagator src/controller/collisionPropagator.cpp)
add_library(${PROJECT_NAME}_GraspConstrainer src/controller/graspConstrainer.cpp)


## ADD library new task here
add_library(${PROJECT_NAME}_Task src/tasks/task.cpp)
add_library(${PROJECT_NAME}_LastTask src/tasks/lastTask.cpp)
add_library(${PROJECT_NAME}_VehicleReachTask src/tasks/vehicleReachTask.cpp)
add_library(${PROJECT_NAME}_VehicleNullVelTask src/tasks/vehicleNullVelTask.cpp)
add_library(${PROJECT_NAME}_EEReachTask src/tasks/endEffectorReachTask.cpp)
add_library(${PROJECT_NAME}_JointLimitsTask src/tasks/jointLimitTask.cpp)
add_library(${PROJECT_NAME}_HorizontalAttitudeTask src/tasks/horizontalAttitudeTask.cpp)
add_library(${PROJECT_NAME}_FovEEToToolTask src/tasks/fovEEToToolTask.cpp)
add_library(${PROJECT_NAME}_VehicleConstrVelTask src/tasks/vehicleConstrainVelTask.cpp)
add_library(${PROJECT_NAME}_ObstacleAvoidVehicleTask src/tasks/obstacleAvoidVehicleTask.cpp)
add_library(${PROJECT_NAME}_ObstacleAvoidEETask src/tasks/obstacleAvoidEETask.cpp)
add_library(${PROJECT_NAME}_PipeReachTask src/tasks/pipeReachTask.cpp)
add_library(${PROJECT_NAME}_ArmShapeTask src/tasks/armShapeTask.cpp)
add_library(${PROJECT_NAME}_CoopTask src/tasks/coopTask.cpp)
add_library(${PROJECT_NAME}_ForceInsertTask src/tasks/forceInsertTask.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_CoordInterfaceMissMan
    ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_CoordInterfaceCoord
    ${catkin_EXPORTED_TARGETS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/peg_pkg_node.cpp)
#set target propr is for call the nome as rosrun pkg_name missManagerA and not pkg_name_missManagerA
add_executable(${PROJECT_NAME}_MissionManagerA src/missionManager.cpp)
set_target_properties(${PROJECT_NAME}_MissionManagerA PROPERTIES OUTPUT_NAME "MissionManagerA")
add_executable(${PROJECT_NAME}_MissionManagerB src/missionManager.cpp)
set_target_properties(${PROJECT_NAME}_MissionManagerB PROPERTIES OUTPUT_NAME "MissionManagerB")
add_executable(${PROJECT_NAME}_Coordinator src/coordinator.cpp)
set_target_properties(${PROJECT_NAME}_Coordinator PROPERTIES OUTPUT_NAME "Coordinator")
add_executable(${PROJECT_NAME}_MissionManagerVision src/missionManagerVision.cpp)
set_target_properties(${PROJECT_NAME}_MissionManagerVision PROPERTIES OUTPUT_NAME "MissionManagerVision")
#old TRY files
add_executable(${PROJECT_NAME}_Vision src/vision.cpp)
set_target_properties(${PROJECT_NAME}_Vision PROPERTIES OUTPUT_NAME "Vision")
#try for peg with force
add_executable(${PROJECT_NAME}_MissionManagerAPeg src/missionManagerPeg.cpp)
set_target_properties(${PROJECT_NAME}_MissionManagerAPeg PROPERTIES OUTPUT_NAME "MissionManagerAPeg")



## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(${PROJECT_NAME}_Coordinator
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_CoordInterfaceCoord
    linuxcmat
    ${PROJECT_NAME}_Formulas
    ${PROJECT_NAME}_Conversions
    ${PROJECT_NAME}_Logger
)

target_link_libraries(${PROJECT_NAME}_MissionManagerA
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_RobotInterface
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_CoordInterfaceMissMan
    ${PROJECT_NAME}_VisionInterfaceMissMan
    ${PROJECT_NAME}_Logger
    ${PROJECT_NAME}_Controller
    ${PROJECT_NAME}_Formulas
    ${PROJECT_NAME}_KdlHelper
    ${PROJECT_NAME}_JacobianHelper
    ${PROJECT_NAME}_CollisionPropagator
    ${PROJECT_NAME}_GraspConstrainer
    #put below new task (is it necessary? YES)
    ${PROJECT_NAME}_LastTask
    ${PROJECT_NAME}_VehicleReachTask
    ${PROJECT_NAME}_EEReachTask
    ${PROJECT_NAME}_JointLimitsTask
    ${PROJECT_NAME}_HorizontalAttitudeTask
    ${PROJECT_NAME}_FovEEToToolTask
    ${PROJECT_NAME}_VehicleNullVelTask
    ${PROJECT_NAME}_VehicleConstrVelTask
    ${PROJECT_NAME}_ObstacleAvoidVehicleTask
    ${PROJECT_NAME}_ObstacleAvoidEETask
    ${PROJECT_NAME}_PipeReachTask
    ${PROJECT_NAME}_ArmShapeTask
    ${PROJECT_NAME}_CoopTask
    ${PROJECT_NAME}_ForceInsertTask
)

target_link_libraries(${PROJECT_NAME}_MissionManagerAPeg
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_RobotInterface
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_CoordInterfaceMissMan
    ${PROJECT_NAME}_VisionInterfaceMissMan
    ${PROJECT_NAME}_Logger
    ${PROJECT_NAME}_Controller
    ${PROJECT_NAME}_Formulas
    ${PROJECT_NAME}_KdlHelper
    ${PROJECT_NAME}_JacobianHelper
    ${PROJECT_NAME}_CollisionPropagator
    #put below new task (is it necessary? YES)
    ${PROJECT_NAME}_LastTask
    ${PROJECT_NAME}_VehicleReachTask
    ${PROJECT_NAME}_EEReachTask
    ${PROJECT_NAME}_JointLimitsTask
    ${PROJECT_NAME}_HorizontalAttitudeTask
    ${PROJECT_NAME}_FovEEToToolTask
    ${PROJECT_NAME}_VehicleNullVelTask
    ${PROJECT_NAME}_VehicleConstrVelTask
    ${PROJECT_NAME}_ObstacleAvoidVehicleTask
    ${PROJECT_NAME}_ObstacleAvoidEETask
    ${PROJECT_NAME}_PipeReachTask
    ${PROJECT_NAME}_ArmShapeTask
    ${PROJECT_NAME}_CoopTask
    ${PROJECT_NAME}_ForceInsertTask

)

target_link_libraries(${PROJECT_NAME}_MissionManagerB
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_RobotInterface
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_CoordInterfaceMissMan
    ${PROJECT_NAME}_VisionInterfaceMissMan
    ${PROJECT_NAME}_Logger
    ${PROJECT_NAME}_Controller
    ${PROJECT_NAME}_Formulas
    ${PROJECT_NAME}_KdlHelper
    ${PROJECT_NAME}_JacobianHelper
    ${PROJECT_NAME}_CollisionPropagator
    ${PROJECT_NAME}_GraspConstrainer
    #put below new task (is it necessary? YES)
    ${PROJECT_NAME}_LastTask
    ${PROJECT_NAME}_VehicleReachTask
    ${PROJECT_NAME}_EEReachTask
    ${PROJECT_NAME}_JointLimitsTask
    ${PROJECT_NAME}_HorizontalAttitudeTask
    ${PROJECT_NAME}_FovEEToToolTask
    ${PROJECT_NAME}_VehicleNullVelTask
    ${PROJECT_NAME}_VehicleConstrVelTask
    ${PROJECT_NAME}_ObstacleAvoidVehicleTask
    ${PROJECT_NAME}_ObstacleAvoidEETask
    ${PROJECT_NAME}_PipeReachTask
    ${PROJECT_NAME}_ArmShapeTask
    ${PROJECT_NAME}_CoopTask
    ${PROJECT_NAME}_ForceInsertTask
)

target_link_libraries(${PROJECT_NAME}_MissionManagerVision
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_RobotVisionInterface
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_VisionInterfaceVision
    linuxcmat
    ${PROJECT_NAME}_Logger
    ${PROJECT_NAME}_Tracker
    ${PROJECT_NAME}_Detector
    ${PROJECT_NAME}_DepthCameraSupport

)

target_link_libraries(${PROJECT_NAME}_Vision
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
    cv_bridge
    ${PROJECT_NAME}_RobotVisionInterface
    linuxcmat
    ${PROJECT_NAME}_WorldInterface
    ${PROJECT_NAME}_Logger
)

target_link_libraries(${PROJECT_NAME}_Detector
    ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}_Tracker
    ${VISP_LIBRARIES}
    ${PROJECT_NAME}_Conversions
    ${catkin_LIBRARIES}

)

target_link_libraries(${PROJECT_NAME}_DepthCameraSupport
    ${catkin_LIBRARIES}
    ${VISP_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_Conversions
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
    ${catkin_LIBRARIES}
    tf_conversions
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_Formulas
    m #it is math.h for atan2
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_Prints
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_Logger
    linuxcmat
    ${PROJECT_NAME}_Task
    ${PROJECT_NAME}_Prints
)

target_link_libraries(${PROJECT_NAME}_Controller
    linuxcmat
    ${PROJECT_NAME}_Prints
    ${PROJECT_NAME}_Task
)

target_link_libraries(${PROJECT_NAME}_RobotInterface
    tf
    ${PROJECT_NAME}_Conversions
    ${PROJECT_NAME}_Formulas
)

target_link_libraries(${PROJECT_NAME}_RobotVisionInterface
    ${catkin_LIBRARIES}
    tf
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_CoordInterfaceMissMan

)

target_link_libraries(${PROJECT_NAME}_CoordInterfaceCoord
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_WorldInterface
    tf
)

target_link_libraries(${PROJECT_NAME}_VisionInterfaceMissMan
    ${PROJECT_NAME}_Conversions
)
target_link_libraries(${PROJECT_NAME}_VisionInterfaceVision
    ${PROJECT_NAME}_Conversions
)



target_link_libraries(${PROJECT_NAME}_KdlHelper
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_JacobianHelper
  ${PROJECT_NAME}_Conversions
  ${PROJECT_NAME}_Formulas
)

target_link_libraries(${PROJECT_NAME}_LastTask
    ${PROJECT_NAME}_Task
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_CollisionPropagator
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_GraspConstrainer
    ${PROJECT_NAME}_Conversions
    linuxcmat
)

# to add compiler flags (-Wall -pedantic)
#target_compile_options(Main INTERFACE ${REPLACED_FLAGS})
#target_compile_options(Conversions INTERFACE ${REPLACED_FLAGS})
#target_compile_options(Controller INTERFACE ${REPLACED_FLAGS})
#target_compile_options(robotInterface INTERFACE ${REPLACED_FLAGS})
#target_compile_options(LastTask INTERFACE ${REPLACED_FLAGS})

### ADD lik libraries for new task here
#NOTE:: eigen e tf not to include, they are already include for all above
target_link_libraries(${PROJECT_NAME}_VehicleReachTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)
target_link_libraries(${PROJECT_NAME}_EEReachTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
    ${PROJECT_NAME}_Formulas
)

target_link_libraries(${PROJECT_NAME}_JointLimitsTask
    ${PROJECT_NAME}_Task
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_HorizontalAttitudeTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Formulas
)

target_link_libraries(${PROJECT_NAME}_FovEEToToolTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Formulas
    ${PROJECT_NAME}_Conversions
)
target_link_libraries(${PROJECT_NAME}_VehicleNullVelTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)
target_link_libraries(${PROJECT_NAME}_VehicleConstrVelTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_ObstacleAvoidVehicleTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_ObstacleAvoidEETask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)

target_link_libraries(${PROJECT_NAME}_PipeReachTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
    ${PROJECT_NAME}_Formulas
)

target_link_libraries(${PROJECT_NAME}_ArmShapeTask
    ${PROJECT_NAME}_Task
    linuxcmat
)

target_link_libraries(${PROJECT_NAME}_CoopTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
)

target_link_libraries (${PROJECT_NAME}_ForceInsertTask
    ${PROJECT_NAME}_Task
    linuxcmat
    ${PROJECT_NAME}_Conversions
    ${PROJECT_NAME}_Formulas

)


# to add compiler flags (-Wall -pedantic)
#target_compile_options(VehicleReachTask INTERFACE ${REPLACED_FLAGS})
#target_compile_options(EEReachTask INTERFACE ${REPLACED_FLAGS})


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
install(PROGRAMS script/keyboardCommand_A.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/keyboardCommandVeh_A.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/keyboardCommand_B.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/keyboardCommandVeh_B.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/keyboardCommandVeh_both.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/keyboardCommand_C.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS script/tfListenerPrecise.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_peg_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
