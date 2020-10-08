cmake_minimum_required(VERSION 2.8.3)
project(coppeliasim_ros_control)

# 
find_package(catkin REQUIRED COMPONENTS roscpp  controller_manager 
    transmission_interface control_toolbox joint_limits_interface urdf 
    hardware_interface)

################################################
# List sources.
################################################
set(${PROJECT_NAME}_headers
    include/simConst.h
    include/simLib.h
    include/simTypes.h

    include/simExtRosControl_server.h
    include/simExtRosControl_plugin.h
    include/MyRobot_simHW.h
)

set(${PROJECT_NAME}_sources
    src/ControlLoop/simLib.cpp
    src/ControlLoop/simExtRosControl_server.cpp
    src/ControlLoop/simExtRosControl_plugin.cpp
    src/ControlLoop/MyRobot_simHW.cpp
)

catkin_package(
               CATKIN_DEPENDS roscpp rosconsole  roslib
               DEPENDS roscpp rosconsole  roslib)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories("/opt/ros/melodic/lib")

add_library(simExtRosControl ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

add_dependencies(simExtRosControl ${catkin_EXPORTED_TARGETS})
target_link_libraries(simExtRosControl
    pthread
    dl
    roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    boost_system
   ${catkin_LIBRARIES}
)
