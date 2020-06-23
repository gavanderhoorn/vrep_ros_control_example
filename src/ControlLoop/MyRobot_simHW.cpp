#include "MyRobot_simHW.h"
#include "simLib.h"

#include <string>
#include <iostream>


namespace MR
{


std::string MyRobot_simHW::sm_jointsName[MR_JOINTS_NUM] = {
    "wheel_left_joint",
    "wheel_right_joint",
    "mmrobot_joint1",
    "mmrobot_joint2",
    "mmrobot_joint3",
    "mmrobot_joint4",
    "mmrobot_joint5",
    "mmrobot_joint6",
    "mmrobot_joint7",
    "mmrobot_finger_joint1",
    "mmrobot_finger_joint2",
    "lift_joint"
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MyRobot_simHW::MyRobot_simHW() :
    hardware_interface::RobotHW()
{
    // Init arrays m_cmd[], m_pos[], m_vel[], m_eff[].
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        m_cmd[i] = 0.0;
        m_pos[i] = 0.0;
        m_vel[i] = 0.0;
        m_eff[i] = 0.0;
    }

    // Init and get handles of the joints to control.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
        m_simJointsHandle[i] = -1;

    // Register joint interfaces.
    registerHardwareInterfaces();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_simHW::init()
{
    // Get joint handles.
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        int simJointsHandle = simGetObjectHandle(sm_jointsName[i].c_str());

        if (simJointsHandle == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get handle for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_simJointsHandle[i] = simJointsHandle;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyRobot_simHW::registerHardwareInterfaces()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        // Joint state interface.
        hardware_interface::JointStateHandle jointStateHandle(sm_jointsName[i], &m_pos[i], &m_vel[i], &m_eff[i]);
        m_jointState_interface.registerHandle(jointStateHandle);

        // Joint command interface (in MyRobot's case this is a velocity interface).
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &m_cmd[i]);
        m_jointVelocity_interface.registerHandle(jointVelocityHandle);
    }

    registerInterface(&m_jointState_interface);
    registerInterface(&m_jointVelocity_interface);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_simHW::read()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        float pos,
              vel,
              eff;

        if (simGetJointPosition(m_simJointsHandle[i], &pos) == -1 ||
            simGetObjectFloatParameter(m_simJointsHandle[i], 2012, &vel) == -1 || // Velocity.
            simGetJointForce(m_simJointsHandle[i], &eff) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }

        m_pos[i] = pos;
        m_vel[i] = vel;
        m_eff[i] = eff;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyRobot_simHW::write()
{
    for (int i = 0; i < MR_JOINTS_NUM; ++i)
    {
        if (simSetJointTargetVelocity(m_simJointsHandle[i], m_cmd[i]) == -1)
        {
            ROS_ERROR_STREAM("MR robot interface not able to get state for '" << sm_jointsName[i] << "'." << std::endl);

            return false;
        }
    }

    return true;
}

} // namespace MR.
