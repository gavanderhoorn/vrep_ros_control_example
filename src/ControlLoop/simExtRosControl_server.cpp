#include "simLib.h"
#include "MyRobot_simHW.h"
#include "simExtRosControl_server.h"

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

ros::NodeHandle* ROS_server::sm_node = NULL;

// Control.
MR::MyRobot_simHW * ROS_server::sm_myRobotHw = 0;
controller_manager::ControllerManager * ROS_server::sm_ctrlManager = 0;

ros::CallbackQueue * ROS_server::sm_rosControlCallbackQueue = 0;
ros::AsyncSpinner * ROS_server::sm_spinner = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ROS_server::initialize()
{
	int argc = 0;
	char** argv = NULL;
    ros::init(argc,argv,"MR_RosControl");

	if(!ros::master::check())
		return(false);
	
    sm_node = new ros::NodeHandle;
    assert(sm_node);

    // Control.
    // NOTE: we create a callback queue only handling ros_control callbacks (not the standard callbacks which remain
    //       handled by the global callback queue). The ros_control callback queue is spinned in a thread of its own
    //       (but the standard messages remain handled by ros::spinOnce() in coppeliasim's main thread).
    //       We adopt this architecture because:
    //          * by design ros_control needs to have the control update and the control callbacks handled in separate
    //            threads,
    //          * by design coppeliasim needs to have the standard callbacks handled into the main thread,
    //          * by design coppeliasim's regular API is not thread safe (and thus accessing the API from a secondary thread as
    //            well as from the main thread, may break things).
    sm_rosControlCallbackQueue = new ros::CallbackQueue();
    assert(sm_rosControlCallbackQueue);
    sm_node->setCallbackQueue(sm_rosControlCallbackQueue);

    sm_myRobotHw = new MR::MyRobot_simHW();
    assert(sm_myRobotHw);

    sm_ctrlManager = new controller_manager::ControllerManager(sm_myRobotHw, *sm_node);
    assert(sm_ctrlManager);

    sm_spinner = new ros::AsyncSpinner(1, sm_rosControlCallbackQueue); // The associated thread stops spinning when object AsyncSpinner is deleted.
    assert(sm_spinner);
    sm_spinner->start();


	return(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::shutDown()
{
    // Control.
    assert(sm_spinner);
    assert(sm_ctrlManager);
    assert(sm_myRobotHw);
    assert(sm_rosControlCallbackQueue);

    delete sm_spinner;
    delete sm_ctrlManager;
    delete sm_myRobotHw;
    delete sm_rosControlCallbackQueue;

	// Shut down:
	ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::instancePass()
{ // When simulation is not running, we "spinOnce" here:
    int simState=simGetSimulationState();
    if ((simState&sim_simulation_advancing)==0)
        spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::mainScriptAboutToBeCalled()
{ // When simulation is running, we "spinOnce" here:
    spinOnce();

    assert(sm_myRobotHw);
    assert(sm_ctrlManager);

    // Update ros_control ControllerManager.
    float simulationTime_sim = simGetSimulationTime();
    static float simulationTime_km1_sim = simulationTime_sim;

    bool isSimulationRunning = simGetSimulationState() == sim_simulation_advancing_running;

    if (simulationTime_km1_sim != simulationTime_sim && isSimulationRunning)
    {
        ros::Time simulationTime;     simulationTime.fromSec(simulationTime_sim);
        ros::Time simulationTime_km1; simulationTime_km1.fromSec(simulationTime_km1_sim);

        sm_myRobotHw->read();
        sm_ctrlManager->update(simulationTime, simulationTime - simulationTime_km1);
        sm_myRobotHw->write();
    }

    simulationTime_km1_sim = simulationTime_sim;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::simulationAboutToStart()
{
    assert(sm_myRobotHw);
    sm_myRobotHw->init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::simulationEnded()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ROS_server::spinOnce()
{
	// Disable error reporting (it is enabled in the service processing part, but we don't want error reporting for publishers/subscribers)
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

	//Process all requested services and topic subscriptions
    ros::spinOnce(); //check what happened if thgis commented as the HWroscontrol has asyn spinner, so it should not be affected

	// Restore previous error report mode:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved);
}

