/*
 * spectator_node.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: hasan
 */




#include "ros/ros.h"
#include "std_msgs/String.h"

#include "spectator/spectator.h"
#include "spectator/cmd_handler.h"

#include "spectator/get_float_value.h"
#include "spectator/get_uint_value.h"
#include "spectator/set_target.h"
#include "spectator/empty.h"
#include "spectator/get_str.h"
#include "spectator/get_str_list.h"
#include "spectator/snap_around.h"
#include "spectator/info.h"

#include <signal.h>

using namespace std;

using namespace spectator;

using namespace client_interface;


extern boost::shared_ptr<SnapAround> snapper;

boost::shared_ptr<Spectator> spec;		// we need to get rid of these global variables later
vector<string> ids;
vector<double> homing_offsets;

bool GetAngle(spectator::get_float_value::Request &req, spectator::get_float_value::Response &resp) {



	return spec->GetAngles(resp.val1, resp.val2);

	//ROS_INFO("angles are (%f %f) \n", resp.val1, resp.val2);
}

bool GetStatus(spectator::get_uint_value::Request &req, spectator::get_uint_value::Response &resp) {

	int status[2];

	bool flag = spec->GetStatus(status);

	if(flag == false)
		return false;

	resp.val1 = (unsigned int) status[0];
	resp.val2 = (unsigned int) status[1];

	return true;

	//ROS_INFO("angles are (%f %f) \n", resp.val1, resp.val2);
}

bool GotoAngle(spectator::set_target::Request &req, spectator::set_target::Response &resp) {


	if(req.joint_name == "pan")
		return spec->PanMoveAbs(req.target_value);

	else if(req.joint_name == "tilt")
		return spec->TiltMoveAbs(req.target_value);

	return spec->JointMoveAbs(req.joint_name, req.target_value);;

}

bool Homing(spectator::empty::Request &req, spectator::empty::Response &resp) {


	//ROS_DEBUG("%s %f", req.joint_name.c_str(), req.target_value);


	ROS_DEBUG("TEST");
	spec->Homing(homing_offsets[0], homing_offsets[1]);

	return true;

}

bool Shutdown(spectator::empty::Request &req, spectator::empty::Response &resp) {


	//ROS_DEBUG("%s %f", req.joint_name.c_str(), req.target_value);


	double val1, val2;
	spec->GetAngles(val1, val2);

	if(1) {
		int status[2];
		if (val1 < -90.0 || val1 > 90.0)
			spec->PanMoveAbs(0.0);
		do {
			ros::Duration(0.5).sleep();
			spec->GetStatus(status);

			//ROS_INFO("pan status %x", status[0]);
		}
		while((status[0] & 0x10) == 0x10);
	}

	ros::Duration(0.5).sleep();
	for (int i=0; i<2; i++)
		spec->DisableJoint(stoi(ids[i]));

	//ROS_DEBUG("flag is %d op is disable joint", flag);


	mosqpp::lib_cleanup();

	ROS_INFO("going to terminate");

	//my_location::Finalize();


	ros::shutdown();

	return true;

}

bool GetSerialDevice(spectator::get_str::Request &req, spectator::get_str::Response &resp) {


	//ROS_DEBUG("%s %f", req.joint_name.c_str(), req.target_value);
	Spectator::state_t state;

	if(spec == NULL)
		return false;

	spec->GetCurrentState(state);
	if(state == Spectator::NOT_CONN)
		return false;

	resp.str = spec->GetPort();

	return true;

}

bool GetAvailableSerialDeviceList(spectator::get_str_list::Request &req, spectator::get_str_list::Response &resp) {


	//ROS_DEBUG("%s %f", req.joint_name.c_str(), req.target_value);

	resp.list = Spectator::GetUSB_SerialList();
	return true;

}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // All the default sigint handler does is call shutdown()



	ROS_INFO("going to terminate");

	//my_location::Finalize();


	ros::shutdown();
}

int main(int argc, char **argv) {


	ros::init(argc, argv, "spectator_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ros::Publisher info_pub = nh.advertise<spectator::info>("spectator/info", 1000);

	ros::ServiceServer service1 = nh.advertiseService("spectator/get_angle", GetAngle);
	ros::ServiceServer service2 = nh.advertiseService("spectator/get_status", GetStatus);
	ros::ServiceServer service3 = nh.advertiseService("spectator/goto_angle", GotoAngle);
	ros::ServiceServer service4 = nh.advertiseService("spectator/homing", Homing);
	ros::ServiceServer service5 = nh.advertiseService("spectator/shutdown", Shutdown);
	ros::ServiceServer service6 = nh.advertiseService("spectator/get_serial_device", GetSerialDevice);
	ros::ServiceServer service7 = nh.advertiseService("spectator/get_available_serial_device_list", GetAvailableSerialDeviceList);

	CmdHandler *cmdHandler;
	int rc;


	signal(SIGINT, mySigintHandler);
	signal(SIGTERM, mySigintHandler);

	mosqpp::lib_init();





	nh.getParam("/ROBOT/hardware_interface/ids", ids);

	if(ids.size() != 2) {
		ROS_ERROR("main:Error in getting ids");
		return -1;
	}

	vector<float> velocities;
	nh.getParam("/ROBOT/hardware_interface/velocities", velocities);

	if(velocities.size() != 2) {
		ROS_ERROR("main:Error in getting velocities");
		return -1;
	}

	ROS_DEBUG("velocities (%f %f) \n", velocities[0], velocities[1]);

	if(velocities[0] > MAX_VELOCITY || velocities[1] > MAX_VELOCITY) {
		ROS_ERROR("Joint velocity out of range");
		return -1;
	}

	nh.getParam("/ROBOT/hardware_interface/homing_offsets", homing_offsets);

	int homing_on_start = 0;
	nh.getParam("/ROBOT/homing_on_start", homing_on_start);


	if(homing_offsets.size() != 2) {
		ROS_ERROR("main:Error in getting homing_offsets");
		return -1;
	}


	spec.reset(new Spectator(nh, homing_offsets));
	snapper.reset(new SnapAround(*spec));

	ros::Timer timer = nh.createTimer(ros::Duration(SnapAround::period), SnapAround::TimerCallback);


	Spectator::state_t state;
	spec->GetCurrentState(state);
	if(state == Spectator::NOT_CONN) {
		ROS_ERROR("Serial device not found");
		exit(1);
	}


	vector<double> lowerLimit{-180.0, -45.0};
	vector<double> upperLimit{180.0, 45.0};

	spec->SetLimits(lowerLimit, upperLimit);


	ros::Time lasttime=ros::Time::now();
//	ros::Duration(3).sleep();




	double angle;
	if (!spec->GetPanAngle(angle)) {
		ROS_ERROR("Error in getting angle");
	}

	ros::Time currtime=ros::Time::now();
	ros::Duration diff=currtime-lasttime;


	bool flag;

	int alarm;


	try {

		for (int i=0; i<2; i++) {
			flag= spec->EnableJoint(stoi(ids[i]));
			ROS_DEBUG("flag is %d op is Enalbe joint", i);
			ros::Duration(INTER_CMD_DELAY).sleep();
		}

		if(homing_on_start) {
			spec->Homing(homing_offsets[0], homing_offsets[1]);
			ros::Duration(1, 0).sleep();
		}
		else
			spec->SetCurrentState(Spectator::READY);



		string currentStateStr;
		int status[2] = {0, 0};
		int preStatus[2] = {0, 0};

		do {
			for (int i=0; i<2; i++) {

				flag = spec->GetStatus(stoi(ids[i]), status[i]);


				ROS_INFO("node %d flag is %d status is %x", i, flag, status);

				ros::Duration(INTER_CMD_DELAY).sleep();

				if((status[i] & 0x0200) || (status[i] & 0x0004)) {
					flag = spec->GetAlarm(stoi(ids[i]), alarm);
					ROS_DEBUG("node %d flag is %d alarm is %x", i, flag, alarm);

					ros::Duration(INTER_CMD_DELAY).sleep();

					flag = spec->AlarmReset(stoi(ids[i]));
					ROS_DEBUG("node %d flag is %d", i, flag);
					ros::Duration(INTER_CMD_DELAY).sleep();


				}


			}

			if(spec->GetCurrentState() == Spectator::HOMING) {
				ros::Duration(1, 0).sleep();

				if ((preStatus[0] == status[0]) && (preStatus[1] == status[1]) && !(status[0] & 0x10) && !(status[1] & 0x10))
					spec->SetCurrentState(Spectator::READY);

				for(int i=0; i<2; i++)
					preStatus[i] = status[i];
			}

			spec->GetCurrentState(currentStateStr);

			if(spec->GetCurrentState() == Spectator::HOMING)
				ROS_WARN("Current state is %s", currentStateStr.c_str());

		}
		while(spec->GetCurrentState() != Spectator::READY);

		ROS_WARN("Current state is %s ... Lets go ...", currentStateStr.c_str());


		//const double velocity = 2.0;

		//ROS_DEBUG_STREAM ("Angle is " << angle << " after " << diff << " time");

		/*
		flag = spec->SetVelocities(velocities);

		if(flag == false)
			return -1;

			*/

		std_msgs::String msg;
		double val1, val2;
		cmdHandler = new CmdHandler(*spec, "spectator_node", "127.0.0.1", 1883);

		while(ros::ok()) {
			rc = cmdHandler->loop();
			if(rc)
				cmdHandler->reconnect();



			//ROS_DEBUG("going to publish");

			if (spec->GetAngles(val1, val2)) {
				stringstream ss;
				spectator::info myMsg;	// use this msg to send snapping info to client_interface  and video maker so that
				myMsg.panAngle = val1;	// client interface and video maker can save image start/stop video
				myMsg.tiltAngle = val2;

				SnapAround::state_t state = snapper->GetState();
				myMsg.snapperIsActive = (state != SnapAround::INACTIVE ? 1 : 0);

				DB_Handler::capture_mode_t captureMode = snapper->GetCaptureMode();
				if(state != SnapAround::WAITING) {
					myMsg.snapperWaitingMode = int(SNAPPER_NOT_WATING);

				}
				else {

					if (captureMode == DB_Handler::IMAGE_MODE)
						myMsg.snapperWaitingMode = int(SNAPPER_WAITING_FOR_IMAGE);
					else if (captureMode == DB_Handler::VIDEO_MODE)
						myMsg.snapperWaitingMode = int(SNAPPER_WAITING_FOR_VIDEO);
					else {
						ROS_ERROR("Error in setting snapper waiting mode");
					}

				}

				info_pub.publish(myMsg);

			}
			else
				ROS_ERROR("Error in getting angle to publish");





			ros::spinOnce();
			ros::Duration(0.1).sleep();

		}



	}

	catch (const exception &e) {
		cout << e.what() << endl;


		mosqpp::lib_cleanup();
		return -1;
	}

	/*
	 * right now the program is terminated when ctrl-c is pressed but in that case
	 * mosqpp::lib_cleanup() is not called
	 * handle SININT signal so that lib_cleanup code can be executed
	 *
	 */

	spec->DisableJoint(stoi(ids[0]));	// ignore the return
	spec->DisableJoint(stoi(ids[1]));




	mosqpp::lib_cleanup();


	return 0;

}


