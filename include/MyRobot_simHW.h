#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace MR
{


enum MrJointsEnum
{
    WHEEL_LEFT_JOINT = 0,
    WHEEL_RIGHT_JOINT,
    MMROBOT_JOINT1,
    MMROBOT_JOINT2,
    MMROBOT_JOINT3,
    MMROBOT_JOINT4,
    MMROBOT_JOINT5,
    MMROBOT_JOINT6,
    MMROBOT_JOINT7,
    MMROBOT_FINGER_JOINT1,
    MMROBOT_FINGER_JOINT2,
    LIFT_JOINT,

    MR_JOINTS_NUM
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief This is the hardware interface for MyRobot simulated in vrep.
class MyRobot_vrepHW : public hardware_interface::RobotHW
{
public:
    MyRobot_vrepHW();

    bool init();

    bool read();
    bool write();

protected:
    static std::string sm_jointsName[MR_JOINTS_NUM];

    // Vrep handles.
    int m_vrepJointsHandle[MR_JOINTS_NUM];

    // Interfaces.
    double m_cmd[MR_JOINTS_NUM];
    double m_pos[MR_JOINTS_NUM];
    double m_vel[MR_JOINTS_NUM];
    double m_eff[MR_JOINTS_NUM];

    hardware_interface::JointStateInterface m_jointState_interface;
    hardware_interface::VelocityJointInterface m_jointVelocity_interface;

    void registerHardwareInterfaces();
};


} // namespace MR.
