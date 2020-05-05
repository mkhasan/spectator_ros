/*
 * joint.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: usrc
 */

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "spectator/joint.h"
#include "spectator/spectator.h"

#include <string>
#include <sstream>

using namespace std;

namespace spectator {



const double LIMIT = 2*180;

#define ROS_DEBUG(...)

Joint::Joint(int _id, Spectator & _spectator):
		id(_id), spectator(_spectator), offset(0), signCCW(-1), lowerLimit(-1*LIMIT), upperLimit(LIMIT)
{
	stringstream ss;

	ss << Spectator::ROBOT_PREFIX << id;
	name = ss.str();


}

Joint::~Joint() {

}

double Joint::read() {
	//ROS_INFO("test val is %d name is %s", test, name.c_str());
	return angle;
}

void Joint::actuate(double pos) {
	//ROS_INFO("test val is %d name is %s", test, name.c_str());


	angle = (test==0 ? 1 : -1)*0.125;
	test = 1-test;
	return;


}

void Joint::SetOffset(int _offset) {
	offset = _offset;
}

void Joint::SetCCW(int sign) {
	signCCW = sign > 0 ? 1 : -1;

}

bool Joint::GotoAbs(double target) {

	if (target > upperLimit+EPSILON) {
		ROS_WARN("target too large %f %f", target, upperLimit);
		return false;
	}

	if(target < lowerLimit-EPSILON) {
		ROS_WARN("target too small");
		return false;

	}


	if(spectator.mode != Spectator::REAL)
		return angle = target;



	int count = int(target*signCCW*GEAR_RATIO/DEG_PER_COUNT) + offset;

	try {
		ExecuteServoCmd("FP", count);
	}

	catch (const exception & e) {
		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;
}

bool Joint::GotoRelative(double target) {

	int count = int(target*signCCW*GEAR_RATIO/DEG_PER_COUNT);

	ROS_INFO("in relative %d", count);
	try {
		ExecuteServoCmd("FL", count);
	}

	catch (const exception & e) {
		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;
}


bool Joint::GetAngle(double & _angle) const {

	if(spectator.mode != Spectator::REAL) {
		_angle = angle;
		return true;
	}


	int count;
	bool success = GetEncoderPosition(count);

	if(!success) {
		ROS_WARN("Error in getting current position of %s", name.c_str());
		return false;
	}

	ROS_DEBUG("offset is %d", offset);
	_angle = signCCW*(count-offset)*DEG_PER_COUNT/GEAR_RATIO;
	return true;
}

bool Joint::GetVelocity(double & v) const {


	if(spectator.mode != Spectator::REAL)
		THROW(SpException, "You\'r not supposed to be here");



	double velocity;

	try {
		this->ExecuteServoCmd("VE", &velocity);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	v = velocity;
	return true;
}


bool Joint::GetEncoderPosition(int & count) const{

	if(spectator.mode != Spectator::REAL)
		THROW(SpException, "You\'r not supposed to be here");


	try {
		this->ExecuteServoCmd("IP", &count);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	ROS_DEBUG("encoder postion is %d", count);
	return true;



}

// return signCCW*(count-offset)*DEG_PER_COUNT/GEAR_RATIO;

bool Joint::SetAngle(double _angle) {

	if(spectator.mode != Spectator::REAL)
		return angle = _angle;
	int count;

	if(!GetEncoderPosition(count))
		return false;

	offset = count - _angle*signCCW*GEAR_RATIO/DEG_PER_COUNT;

	ROS_DEBUG("SetAngle: Offset is %d", offset);
	return true;

}

bool Joint::SetVelocity(double _velocity) const{
	if(spectator.mode != Spectator::REAL)
		THROW(SpException, "You\'r not supposed to be here");



	try {
		this->ExecuteServoCmd("VE", _velocity);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;

}

bool Joint::SetHomeSearchVelocity(double _velocity) const{
	if(spectator.mode != Spectator::REAL)
		THROW(SpException, "You\'r not supposed to be here");



	try {
		this->ExecuteServoCmd("VC", _velocity);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;

}

void Joint::SetLimit(double lower, double upper) {
	lowerLimit = lower;
	upperLimit = upper;
}

void Joint::GetLimit(double &lower, double &upper) const {
	lower = lowerLimit;
	upper = upperLimit;
}

void Joint::ExecuteServoCmd(const char * cmd) const {
	const int MAX_LEN = 50;
	char ptr[MAX_LEN] = {0, };

	sprintf(ptr, "%d%c%c%c", id, cmd[0], cmd[1], '\r');

	boost::shared_ptr<serial::Serial> & mySerial = spectator.mySerial;

	if (mySerial->write(ptr) != strlen(ptr))
		THROW (SpException, "Error in writing");

	//string reply = mySerial->read(3);

	string reply="";
	string s;


	do {
		s = mySerial->read(1);
		if (s.length() > 0)
			reply += s;
	}
	while (s.length() && s != "\r");


	ROS_DEBUG("reply length is %d", reply.length());
	ROS_DEBUG(ptr);


	if(reply.length() < 3)
		THROW(SpException, "invalid reply");

	if(reply.at(0) != id+'0')
		THROW(SpException, "Wrong id in return");

	if(CheckAck(reply.at(1)) == false) {


		ptr[0] = id+'0';
		ptr[1] = 'A';
		ptr[2] = 'R';
		ptr[3] = '\r';
		ptr[4] = 0;

		mySerial->write(ptr);
		mySerial->read(MAX_LEN);

		THROW(SpException, "Command not recognized");

	}

}


void Joint::ExecuteServoCmd(const char * cmd, int param) const {
	const int MAX_LEN = 50;
	char ptr[MAX_LEN] = {0, };

	sprintf(ptr, "%d%c%c%d%c", id, cmd[0], cmd[1], param, '\r');

	boost::shared_ptr<serial::Serial> & mySerial = spectator.mySerial;

	if (mySerial->write(ptr) != strlen(ptr))
		THROW (SpException, "Error in writing");

	//string reply = mySerial->read(3);

	string reply="";
	string s;


	do {
		s = mySerial->read(1);
		if (s.length() > 0)
			reply += s;
	}
	while (s.length() && s != "\r");

	ROS_DEBUG("reply length is %d", reply.length());
	ROS_DEBUG("reply is %s", reply.c_str());


	if(reply.length() < 3)
		THROW(SpException, "invalid reply");

	if(reply.at(0) != id+'0')
		THROW(SpException, "Wrong id in return");

	if(CheckAck(reply.at(1)) == false) {


		ptr[0] = id+'0';
		ptr[1] = 'A';
		ptr[2] = 'R';
		ptr[3] = '\r';
		ptr[4] = 0;

		mySerial->write(ptr);
		mySerial->read(MAX_LEN);



		THROW(SpException, "Command not recognized");

	}

}


void Joint::ExecuteServoCmd(const char * cmd, double param) const {
	const int MAX_LEN = 50;
	char ptr[MAX_LEN] = {0, };

	sprintf(ptr, "%d%c%c%f%c", id, cmd[0], cmd[1], (float)param, '\r');

	boost::shared_ptr<serial::Serial> & mySerial = spectator.mySerial;

	if (mySerial->write(ptr) != strlen(ptr))
		THROW (SpException, "Error in writing");

	//string reply = mySerial->read(3);

	string reply="";
	string s;


	do {
		s = mySerial->read(1);
		if (s.length() > 0)
			reply += s;
	}
	while (s.length() && s != "\r");

	ROS_DEBUG("reply length is %d", reply.length());
	ROS_DEBUG(ptr);


	if(reply.length() < 3)
		THROW(SpException, "invalid reply");

	if(reply.at(0) != id+'0')
		THROW(SpException, "Wrong id in return");

	if(CheckAck(reply.at(1) == false)) {


		ptr[0] = id+'0';
		ptr[1] = 'A';
		ptr[2] = 'R';
		ptr[3] = '\r';
		ptr[4] = 0;

		mySerial->write(ptr);
		mySerial->read(MAX_LEN);

		THROW(SpException, "Command not recognized");

	}

}


void Joint::ExecuteServoCmd(const char* cmd, int * pResult) const {
	boost::shared_ptr<serial::Serial> & mySerial = spectator.mySerial;
	const int MAX_LEN = 50;
	char ptr[MAX_LEN]="";

	for (int i=0; i<20; i++)
		ptr[i] = 0;
	sprintf(ptr, "%d%c%c%c", id, cmd[0], cmd[1], '\r');

	//sprintf(ptr, "1FL20000%c",  '\r');
	if (mySerial->write(ptr) != strlen(ptr))
		THROW (SpException, "Error in writing");

	string reply="";
	string s;


	do {
		s = mySerial->read(1);
		if (s.length() > 0)
			reply += s;
	}
	while (s.length() && s != "\r");



	ROS_DEBUG("reply length is %d ", reply.length());
	ROS_DEBUG(reply.c_str());

	if(reply.length() < 5)
		THROW(SpException, "invalid reply");

	if(reply.at(0) != '0'+id)			// assuming 1 digit rs485 address
		THROW(SpException, "Wrong id in return");

	if(!(reply.at(1) == cmd[0] && reply.at(2) == cmd[1] && reply.at(3) == '='))
		THROW(SpException, "Command not recognized");

	int index = 4;
	int sign = 1;
	char valueStr[MAX_LEN];
	int k = 0;

	if(reply.at(index) == '-') {
		sign = -1;
		index ++;
	}

	while(index < reply.length()) {
		if(reply.at(index) == '\r')
			break;
		if(!(reply.at(index) >= '0' && reply.at(index) <= '9' ) && !(reply.at(index) >= 'A' && reply.at(index) <= 'F'))
			THROW(SpException, "Invalid reply");
		valueStr[k++] = reply.at(index++);
	}
	valueStr[k] = 0;

	ROS_DEBUG("valueStr is %s", valueStr);
	try {
		if((cmd[0] == 'S' && cmd[1] == 'C') || (cmd[0] == 'A' && cmd[1] == 'L'))
			*pResult = std::stoi(valueStr, NULL, 16);
		else
			*pResult = sign*std::stoi(valueStr);
		ROS_DEBUG("*pResult is %d", *pResult);

	}
	catch(...) {
		THROW(SpException, "Non integer value repled");
	}

}

void Joint::ExecuteServoCmd(const char* cmd, double * pResult) const {
	boost::shared_ptr<serial::Serial> & mySerial = spectator.mySerial;
	const int MAX_LEN = 50;
	char ptr[MAX_LEN]="";

	for (int i=0; i<20; i++)
		ptr[i] = 0;
	sprintf(ptr, "%d%c%c%c", id, cmd[0], cmd[1], '\r');

	//sprintf(ptr, "1FL20000%c",  '\r');
	if (mySerial->write(ptr) != strlen(ptr))
		THROW (SpException, "Error in writing");

	string reply="";
	string s;


	do {
		s = mySerial->read(1);
		if (s.length() > 0)
			reply += s;
	}
	while (s.length() && s != "\r");



	ROS_DEBUG("reply length is %d ", reply.length());
	ROS_DEBUG(reply.c_str());

	if(reply.length() < 5)
		THROW(SpException, "invalid reply");

	if(reply.at(0) != '0'+id)			// assuming 1 digit rs485 address
		THROW(SpException, "Wrong id in return");

	if(!(reply.at(1) == cmd[0] && reply.at(2) == cmd[1] && reply.at(3) == '='))
		THROW(SpException, "Command not recognized");

	int index = 4;
	int sign = 1;
	char valueStr[MAX_LEN];
	int k = 0;

	if(reply.at(index) == '-') {
		sign = -1;
		index ++;
	}

	while(index < reply.length()) {
		if(reply.at(index) == '\r')
			break;
		if(reply.at(index) < '0' || reply.at(index) > '9')
			THROW(SpException, "Invalid reply");
		valueStr[k++] = reply.at(index++);
	}
	valueStr[k] = 0;

	try {
		*pResult = sign*std::stod(valueStr);

	}
	catch(...) {
		THROW(SpException, "Non integer value repled");
	}

}

bool Joint::Stop() {

	if(spectator.mode != Spectator::REAL)
		return true;


	try {
		this->ExecuteServoCmd("ST");
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;


}

bool Joint::AlarmReset() {

	if(spectator.mode != Spectator::REAL)
		return true;


	try {
		this->ExecuteServoCmd("AR");
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;


}


bool Joint::GetStatus(int & status) const {

	int st;
	try {
		this->ExecuteServoCmd("SC", &st);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	status = st;
	return true;
}


bool Joint::GetAlarm(int & alarm) const {

	int al;
	try {
		this->ExecuteServoCmd("AL", &al);
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	alarm = al;
	return true;
}


bool Joint::Enable() const {

	int st;
	try {
		this->ExecuteServoCmd("ME");
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;
}

bool Joint::Disable() const {

	int st;
	try {
		this->ExecuteServoCmd("MD");
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;
}

bool Joint::Homing() {

	int st;
	try {
		this->ExecuteServoCmd("FH", (int) 1);
		this->offset = 0;
	}
	catch (const exception & e){

		ROS_ERROR("Joint %s reported error: %s", name.c_str(), e.what());
		return false;
	}

	return true;
}


}		// end of namespaace

