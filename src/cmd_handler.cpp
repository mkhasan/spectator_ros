/*
 * cmd_handler.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: usrc
 */


#include "spectator/cmd_handler.h"
#include "spectator/snap_around.h"
#include "client_interface/db_handler.h"
#include <boost/algorithm/string.hpp>


#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>



using namespace std;

using namespace spectator;

extern boost::shared_ptr<spectator::SnapAround> snapper;

CmdHandler::CmdHandler(Spectator & _spectator, const char *id, const char *host, int port)
	: spectator(_spectator)
	, dir(CENTER)
	, sel(CENTER)
	, skipCmd(0)
	, mosquittopp(id)
{
	int keepalive = 60;

	/* Connect immediately. This could also be done by calling
	 * mqtt_tempconv->connect(). */
	connect(host, port, keepalive);
};


CmdHandler::~CmdHandler()
{
	disconnect();
}

void CmdHandler::on_connect(int rc)
{
	cout << "Connected with code " << rc << endl;
	if(rc == 0){

		subscribe(NULL, RIGHT_STICK);
		subscribe(NULL, RIGHT_BUTTON_GROUP);
		subscribe(NULL, SPECTATOR_GET_ANGLES);
		subscribe(NULL, SPECTATOR_HOMING);
		subscribe(NULL, SPECTATOR_GOTO_ABS);
		subscribe(NULL, SPECTATOR_GOTO_PAN_ABS);
		subscribe(NULL, SPECTATOR_GOTO_TILT_ABS);
		subscribe(NULL, SPECTATOR_SNAP_AROUND);
		subscribe(NULL, SPECTATOR_SNAPPING_ABORT);
		subscribe(NULL, SPECTATOR_VIDEO_AROUND);



	}
}

void CmdHandler::on_message(const struct mosquitto_message *message)
{


	//cmdHandler->publish(NULL, "test", strlen("test"), "test");

	if(spectator.GetCurrentState() != Spectator::READY)
		return;

	if(!strcmp(message->topic, RIGHT_STICK) && skipCmd==0) {
		if(message->payload == NULL) {
			ROS_ERROR("CmdHandler::on_message msg format error");
			return;
		}

		HandleStick(message);
	}

	else if(!strcmp(message->topic, RIGHT_BUTTON_GROUP)) {	// if button msg is given then skip next SKIP_COUNT number of
															// button msg before receiving any msg
		if(message->payload == NULL) {
			ROS_ERROR("CmdHandler::on_message msg format error");
			return;
		}

		if(skipCmd == 0) {
			HandleButtonGroup(message);
			skipCmd = SKIP_COUNT;
		}
		else
			skipCmd --;

	}
	else if(!strcmp(message->topic, SPECTATOR_GET_ANGLES)) {

		double panAngle, tiltAngle;
		spectator.GetAngles(panAngle, tiltAngle);
		char buf[100];
		sprintf(buf, "%f %f", panAngle, tiltAngle);
		publish(NULL, SPECTATOR_POSITION, strlen(buf), buf);

	}
	else if(!strcmp(message->topic, SPECTATOR_HOMING)) {

		ROS_DEBUG("Homing...");
		spectator.Homing();

	}
	else if(!strcmp(message->topic, SPECTATOR_GOTO_ABS)) {
		char buf[100];

		if(message->payload == NULL) {
			ROS_ERROR("CmdHandler::on_message msg format error");
			return;
		}

		memset(buf, 0, 51*sizeof(char));

		memcpy(buf, message->payload, 50*sizeof(char));

		ROS_DEBUG(buf);
		double x, y;

		char * token = strtok(buf, " ");


		for(int count=0; token != NULL && count < 2; count++) {
			try {
				double * val = count == 0 ? &x : &y;
				*val = std::stod(token);

			}
			catch (...) {

				return;
			}

			token = strtok(NULL, " ");
		}

		ROS_DEBUG("Val1 %f val2 %f", x, y);

		spectator.PanMoveAbs(x);
		ros::Duration(INTER_CMD_DELAY).sleep();
		spectator.TiltMoveAbs(y);
		ros::Duration(INTER_CMD_DELAY).sleep();


	}
	else if(strcmp(message->topic, SPECTATOR_GOTO_PAN_ABS) == 0 || strcmp(message->topic, SPECTATOR_GOTO_TILT_ABS) == 0) {
		char buf[100];

		if(message->payload == NULL) {
			ROS_ERROR("CmdHandler::on_message msg format error");
			return;
		}

		memset(buf, 0, 51*sizeof(char));

		memcpy(buf, message->payload, 50*sizeof(char));

		ROS_DEBUG(buf);
		double x, y;


		char * token = strtok(buf, " ");


		for(int count=0; token != NULL && count < 1; count++) {
			try {
				double * val = &x;
				*val = std::stod(token);

			}
			catch (...) {

				return;
			}

			token = strtok(NULL, " ");
		}

		ROS_DEBUG("Val1 %f val2 %f", x, y);

		if(strcmp(message->topic, SPECTATOR_GOTO_PAN_ABS) == 0)
			spectator.PanMoveAbs(x);
		else
			spectator.TiltMoveAbs(x);

		ros::Duration(INTER_CMD_DELAY).sleep();
	}


	/*
	else if(strcmp(message->topic, SPECTATOR_SNAP_AROUND) == 0 ) {
		snapper->Activate(30.0, 3.0);

	}
	*/

	else if((!strcmp(message->topic, SPECTATOR_SNAP_AROUND) || (!strcmp(message->topic, SPECTATOR_VIDEO_AROUND))) && message->payloadlen > 0) {
		string msg((const char *)message->payload);


		vector<string> results;
		boost::split(results, msg, [](char c){return c == ' ';});

		try {
			if(results.size() < 2) {
				ROS_ERROR("SPECTATOR_SNAP OR VIDEO_AROUND: msg format error");
				return;
			}

			double stepAngle, waitingTimeSec;

			stepAngle = std::stod(results[0]);
			waitingTimeSec = std::stod(results[1]);

			ROS_DEBUG("SPECTATOR_SNAP OR VIDEO_AROUND: stepAngle %f, waitingTimeSec %f", stepAngle, waitingTimeSec);

			if(!strcmp(message->topic, SPECTATOR_SNAP_AROUND))
				snapper->SetCaptureMode(client_interface::DB_Handler::IMAGE_MODE);
			else
				snapper->SetCaptureMode(client_interface::DB_Handler::VIDEO_MODE);

			snapper->Activate(stepAngle, waitingTimeSec);

		}
		catch (const exception & e){
			ROS_ERROR("Error: ", e.what());
		}



	}

	else if(strcmp(message->topic, SPECTATOR_SNAPPING_ABORT) == 0 ) {
		snapper->Inactivate();
	}





}

void CmdHandler::HandleStick(const struct mosquitto_message *message) {

	char buf[100];

	memset(buf, 0, 51*sizeof(char));

	memcpy(buf, message->payload, 50*sizeof(char));

	ROS_DEBUG(buf);
	double x, y;


	char * token = strtok(buf, " ");


	for(int count=0; token != NULL && count < 2; count++) {
		try {
			double * val = count == 0 ? &x : &y;
			*val = std::stod(token);

		}
		catch (...) {

			return;
		}

		token = strtok(NULL, " ");



	}

	ROS_DEBUG("Val1 %f val2 %f", x, y);

	dir_t targetDir = GetDir(x, y);

	if(dir == targetDir)
		return;

	if(dir == LEFT || dir == RIGHT)
		spectator.PanStop();
	else if(dir == UP || dir == DOWN)
		spectator.TiltStop();

	ROS_DEBUG("Cur dir %d target dir %d", dir, targetDir);

	switch(targetDir) {
		case LEFT:
			spectator.GoLeft();
			break;
		case RIGHT:
			spectator.GoRight();
			break;
		case UP:
			spectator.GoUp();
			break;
		case DOWN:
			spectator.GoDown();
			break;

	}


	dir = targetDir;

}

void CmdHandler::HandleButtonGroup(const struct mosquitto_message *message) {

	char buf[100];

	memset(buf, 0, 51*sizeof(char));

	memcpy(buf, message->payload, 50*sizeof(char));

	ROS_DEBUG(buf);

	dir_t targetSel = GetSel(buf[0]);

	if(targetSel == CENTER)
		return;


	spectator.PanStop();
	spectator.TiltStop();

	ROS_INFO("target sel %d", targetSel);

	switch(targetSel) {
		case LEFT:
			spectator.GoLeft(ANGLE_STEP);
			break;
		case RIGHT:
			spectator.GoRight(ANGLE_STEP);
			break;
		case UP:
			spectator.GoUp(ANGLE_STEP);
			break;
		case DOWN:
			spectator.GoDown(ANGLE_STEP);
			break;

	}


	sel = targetSel;

}

CmdHandler::dir_t CmdHandler::GetDir(double x, double y) const {

	const double THRESHOLD = 0.2;
	if (std::abs(x) < THRESHOLD && std::abs(y) < THRESHOLD)
		return CENTER;

	ROS_DEBUG("Getdir: x %f, y %f ", x, y);

	if(std::abs(x) > std::abs(y)) {
		return x > 0.0 ? RIGHT : LEFT;
	}

	return y > 0.0 ? UP : DOWN;
}


CmdHandler::dir_t CmdHandler::GetSel(char ch) const {

	switch (ch) {
		case 'Y':
			return UP;
		case 'A':
			return DOWN;
		case 'X':
			return LEFT;
		case 'B':
			return RIGHT;
	}

	return CENTER;
}
