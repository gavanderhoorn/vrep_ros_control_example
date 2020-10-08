#pragma once
#include <ros/ros.h>

// API services:
#include <transmission_interface/transmission_parser.h>



namespace coppeliasim_ros_control
{
class RobotSimHW;
}

namespace controller_manager
{
class ControllerManager;
}

namespace ros
{
class CallbackQueue;
}

class ROS_server
{
public:
    static bool initialize();
    static void shutDown();

    static void instancePass();
    static void mainScriptAboutToBeCalled();

    static void simulationAboutToStart();
    static void simulationEnded();

private:
    ROS_server() {} // Make this class non-instantiable.

    static ros::NodeHandle* sm_node;
    static void spinOnce();

    // Control.
    static ros::CallbackQueue * sm_rosControlCallbackQueue;
    static coppeliasim_ros_control::RobotSimHW * sm_myRobotHw;
    static controller_manager::ControllerManager * sm_ctrlManager;
    static ros::AsyncSpinner * sm_spinner;
};
