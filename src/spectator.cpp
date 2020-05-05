/*
 * spectator.cpp
 *
 *  Created on: Sep 27, 2018
 *      Author: usrc
 */




#include "ros/ros.h"
#include "std_msgs/String.h"



#include "spectator/spectator.h"

#include <string>

using namespace std;
namespace spectator {

const string Spectator::REAL = "real";
const string Spectator::ROBOT_PREFIX = "ROBOT_JOINT_";
const string Spectator::STATE_DESC[Spectator::MAX_STATE] = {"not connected", "connected", "homing", "ready"};


Spectator::Spectator(ros::NodeHandle& _nh, const vector<double>& _homingOffsets)
	: nh(_nh)
	, homingOffsets(_homingOffsets)
	, currentState(NOT_CONN)
{

	_nh.param("/ROBOT/mode", mode, string("simu"));

	ROS_INFO("%s mode", mode.c_str());

	_nh.param("/ROBOT/port", port, string("/dev/ttyUSB1"));




	vector<string> ids;
	_nh.getParam("/ROBOT/hardware_interface/ids", ids);

	if(ids.size() != 2)
		THROW(SpException,"ids not present in the config file");

	int panId, tiltId;

	panId = std::stoi(ids[0].c_str());
	tiltId = std::stoi(ids[1].c_str());

	uint32_t baud = 9600;

	vector<string> portList;



	//ROS_DEBUG("size is %d \n", list.size());
	pan = new Joint(panId, *this);
	tilt = new Joint(tiltId, *this);

	pan->SetCCW(PAN_CCW);
	tilt->SetCCW(TILT_CCW);

	double angle;
	bool flag = false;

	while (flag == false) {
		if(port == "auto")
			portList = Spectator::GetUSB_SerialList();
		else
			portList.push_back(port);


		for(auto & p:portList) {
			//ROS_DEBUG("one port is %s \n", p.c_str());


			mySerial.reset(new serial::Serial(p, baud, serial::Timeout::simpleTimeout(TIME_OUT_IN_MS)));
			flag = pan->GetAngle(angle);

			if(flag) {
				currentState = CONN;
				port = p;
				ROS_DEBUG("state is %s %s", STATE_DESC[currentState], port.c_str());
				break;
			}

		}

		ros::Duration(1).sleep();



	}




	ROS_DEBUG("FLAG for get angle is %d", flag);



	ROS_INFO("pan id %d tilt id %d", panId, tiltId);

	ROS_INFO("port is %s", port.c_str());

}



Spectator::~Spectator() {
	delete pan;
	delete tilt;

}



Joint & Spectator::getJoint(const std::string & _name) {
	if(_name == pan->name)
		return *pan;
	else
		return *tilt;
}


bool Spectator::GetPanAngle(double & angle) const{

	return pan->GetAngle(angle);
}

bool Spectator::SetPanAngle(double angle) {
	return pan->SetAngle(angle);
}

bool Spectator::GetTiltAngle(double & angle) const{

	return tilt->GetAngle(angle);
}

bool Spectator::SetTiltAngle(double angle) {
	return tilt->SetAngle(angle);
}


bool Spectator::GetPanVelocity(double & v) const{

	return pan->GetVelocity(v);
}

bool Spectator::GetTiltVelocity(double & v) const{

	return tilt->GetVelocity(v);
}

bool Spectator::SetPanVelocity(double velocity) const{
	return pan->SetVelocity(velocity);
}

bool Spectator::SetTiltVelocity(double velocity) const{
	return tilt->SetVelocity(velocity);
}


bool Spectator::PanMoveAbs(double target) const {
	return pan->GotoAbs(target);
}

bool Spectator::TiltMoveAbs(double target) const {
	return tilt->GotoAbs(target);
}

bool Spectator::JointMoveAbs(const string & jointName, double target) {
	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);
	if(p == NULL)
		return false;

	return p->GotoAbs(target);
}



bool Spectator::PanStop() const{
	return pan->Stop();
}

bool Spectator::TiltStop() const{
	return tilt->Stop();
}

bool Spectator::GoLeft() const{
	double left, right;
	pan->GetLimit(left, right);
	ROS_DEBUG("going left");
	return pan->GotoAbs(left);


}

bool Spectator::GoLeft(double s) const {

	double pos;
	if( !pan->GetAngle(pos))
		return false;

	double left, right;
	pan->GetLimit(left, right);
	if (pos - s > left + EPSILON)
		return pan->GotoAbs(pos-s);
}

bool Spectator::GoRight(double s) const {

	double pos;

	if( !pan->GetAngle(pos))
		return false;

	double left, right;
	pan->GetLimit(left, right);
	if (pos + s < right - EPSILON)
		return pan->GotoAbs(pos+s);

}


bool Spectator::GoRight() const{
	double left, right;
	pan->GetLimit(left, right);
	ROS_DEBUG("going right");
	return pan->GotoAbs(right);

}

bool Spectator::GoUp(double s) const {

	double pos;

	if( !tilt->GetAngle(pos))
		return false;

	double top, bottom;
	tilt->GetLimit(bottom, top);

	if (pos + s < top - EPSILON)
		return tilt->GotoAbs(pos+s);

}


bool Spectator::GoUp() const{
	double top, bottom;
	tilt->GetLimit(bottom, top);
	ROS_DEBUG("going Up");
	return tilt->GotoAbs(top);

}

bool Spectator::GoDown(double s) const {

	double pos;

	if( !tilt->GetAngle(pos))
		return false;

	double top, bottom;
	tilt->GetLimit(bottom, top);
	if (pos - s > bottom + EPSILON)
		return tilt->GotoAbs(pos-s);

}


bool Spectator::GoDown() const{
	double top, bottom;
	tilt->GetLimit(bottom, top);
	ROS_DEBUG("going Down");
	return tilt->GotoAbs(bottom);

}

bool Spectator::GetAngles(double &panAngle, double &tiltAngle) const {


	bool flag = pan->GetAngle(panAngle);

	tiltAngle = 0.0;

	flag &= tilt->GetAngle(tiltAngle);

	return flag;

}

bool Spectator::GetStatus(const std::string& jointName, int & status) const {
	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);

	if(!p)
		return false;

	return p->GetStatus(status);
}

bool Spectator::GetAlarm(const std::string& jointName, int & alarm) const {
	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);

	if(!p)
		return false;

	return p->GetAlarm(alarm);
}


bool Spectator::GetStatus(int jointId, int & status) const {

	string jointName = ROBOT_PREFIX + std::to_string(jointId);

	//ROS_DEBUG("JOINT name is %s", jointName.c_str());
	return GetStatus(jointName, status);

}

bool Spectator::GetAlarm(int jointId, int & alarm) const {

	string jointName = ROBOT_PREFIX + std::to_string(jointId);

	//ROS_DEBUG("JOINT name is %s", jointName.c_str());
	return GetAlarm(jointName, alarm);

}

bool Spectator::AlarmReset(const std::string& jointName) const {
	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);

	if(!p)
		return false;

	return p->AlarmReset();
}


bool Spectator::AlarmReset(int jointId) const {

	string jointName = ROBOT_PREFIX + std::to_string(jointId);

	//ROS_DEBUG("JOINT name is %s", jointName.c_str());
	return AlarmReset(jointName);

}

bool Spectator::GetStatus(int * status) const {



	int test;
	int val[2];

	bool flag = pan->GetStatus(val[0]);

	flag &= tilt->GetStatus(val[1]);

	if (flag == false)
		return flag;

	try {
		for (int i=0; i<2; i++) {
			//ROS_DEBUG("ss %d is %s", i, ss.str().c_str());
			//stringstream ss;
			status[i] = val[i];
			//ROS_DEBUG("ss %d is %s", i, ss.str().c_str());
			//status[i] = std::stoi(ss.str(), NULL, 16);
		}
	}
	catch (const exception & e) {
		return false;
	}

	return true;

}

bool Spectator::EnableJoint(const std::string & jointName) const {

	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);

	if(!p)
		return false;

	return p->Enable();
}
bool Spectator::EnableJoint(int jointId) const {

	string jointName = ROBOT_PREFIX + std::to_string(jointId);

		//ROS_DEBUG("JOINT name is %s", jointName.c_str());
	return EnableJoint(jointName);

}


bool Spectator::DisableJoint(const std::string & jointName) const {

	Joint * p = pan->name == jointName ? pan : (tilt->name == jointName ? tilt : NULL);

	if(!p)
		return false;

	return p->Disable();
}
bool Spectator::DisableJoint(int jointId) const {

	string jointName = ROBOT_PREFIX + std::to_string(jointId);

		//ROS_DEBUG("JOINT name is %s", jointName.c_str());
	return DisableJoint(jointName);

}


bool Spectator::SetVelocities(const vector<float> & velocities) {


	bool flag = pan->SetVelocity(velocities[0]);
	if (flag == false) {
		ROS_ERROR("Error in setting pan velocity");
		return flag;
	}

	ros::Duration(INTER_CMD_DELAY).sleep();

	flag = tilt->SetVelocity(velocities[1]);
	if (flag == false) {
		ROS_ERROR("Error in setting tilt velocity");
		return flag;

	}

	ros::Duration(INTER_CMD_DELAY).sleep();
	flag = pan->SetHomeSearchVelocity(velocities[0]);
	if(flag == false) {
		ROS_ERROR("Error in setting pan home serarching velocity");
		return flag;

	}

	ros::Duration(INTER_CMD_DELAY).sleep();
	flag = tilt->SetHomeSearchVelocity(velocities[1]);
	if(flag == false) {
		ROS_ERROR("Error in setting tilt home serarching velocity");
		return flag;

	}



	return true;

}

bool Spectator::Homing() {
	if(homingOffsets.size() != 2)
		return false;

	return Homing(homingOffsets[0], homingOffsets[1]);
}

bool Spectator::Homing(double panInc, double tiltInc) {
	bool flag;

	currentState = HOMING;

	flag = pan->Homing();

	if(flag == false)
		return flag;

	ros::Duration(INTER_CMD_DELAY).sleep();

	flag = tilt->Homing();

	if(flag == false)
		return flag;

	flag = pan->GotoRelative(panInc);

	if(flag == false)
		return flag;

	flag = tilt->GotoRelative(tiltInc);

	if(flag == false)
		return flag;


	int panStatus=0, tiltStatus=0;
	bool panStopped = false;
	bool tiltStopped = false;
	int limit=1000;		// wait for 100 sec
	do {


		if(panStopped == false) {
			ros::Duration(INTER_CMD_DELAY).sleep();
			if( pan->GetStatus(panStatus) == false)
				return false;
		}


		else if(tiltStopped == false) {
			ros::Duration(INTER_CMD_DELAY).sleep();
			if( tilt->GetStatus(tiltStatus) == false)
				return false;
		}
		else
			break;


		panStopped = !(panStatus & MOVING_MASK);

		tiltStopped = !(tiltStatus & MOVING_MASK);


		limit --;

	}
	while(limit);

	ROS_INFO("limit is %d", limit);
	if(limit == 0)
		return false;

	int count = pan->DegToCount(panInc, 0);
	//ROS_INFO("In homing %d", count);
	pan->SetOffset(count);

	count = tilt->DegToCount(tiltInc, 0);
	tilt->SetOffset(count);

	return true;



}

void Spectator::SetLimits(const std::vector<double>& lowerLimit, const std::vector<double>& upperLimit) {

	if(pan != NULL)
		pan->SetLimit(lowerLimit[0], upperLimit[0]);

	if(tilt != NULL)
		tilt->SetLimit(lowerLimit[1], upperLimit[1]);
	return;
}


vector<string> Spectator::GetUSB_SerialList() {

	vector<serial::PortInfo> portList = serial::list_ports();

	vector<string> list;
	const string prefix = "/dev/ttyUSB";

	for(auto & p:portList) {
		if(p.port.substr(0, prefix.length()) == prefix) {
			list.push_back(p.port);
			ROS_INFO("device found %s", p.port.c_str());
		}

	}

	return list;


}

string Spectator::GetPort() const {
	return port;
}

void Spectator::GetCurrentState(string & curState) const {
	curState = STATE_DESC[currentState];
}

void Spectator::GetCurrentState(state_t & curState) const {
	curState = currentState;

}

Spectator::state_t Spectator::GetCurrentState() const {
	return currentState;
}

void Spectator::SetCurrentState(state_t curState){
	currentState = curState;

}

}

int test_spectator() {
    ROS_INFO("%s", "spectator");

	return 0;
}




