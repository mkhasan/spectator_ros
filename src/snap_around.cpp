/*
 * snap_around.cpp
 *
 *  Created on: Oct 2, 2019
 *      Author: kict
 */


#include "spectator/snap_around.h"


boost::shared_ptr<spectator::SnapAround> snapper;		// only one snapper is allowed


const double spectator::SnapAround::EPSION  = 0.05;
double spectator::SnapAround::period = 1.0;


void spectator::SnapAround::TimerCallback(const ros::TimerEvent & ) {

	static int prevMovingState = 0;
	static int currMovingState = 0;
	static int waiting = 0;
	int status[2] = {0, 0};
	const int LIMIT_ANGLE = 180.0;


	Spectator & spec = snapper->spec;

	state_t snapperState = snapper->GetState();

	if (snapperState == INACTIVE) {

		bool flag = snapper->spec.GetStatus(status);

		ROS_DEBUG("flag: %d status 0: %x status 1: %x", flag, status[0], status[1]);
		return;
	}



	if (!snapper->spec.GetStatus(status)) {
		ROS_ERROR("TimerCallback: Error in getting specatator status");
		return;
	}

	currMovingState = status[0];

	if((currMovingState == prevMovingState) && !(currMovingState & 0x10)) {		// snapper has stopped
		ROS_DEBUG("snapper state is %d", snapperState);

		double theta;

		switch(snapperState) {

			case INIT:
				snapper->targetAngle = (-1)*LIMIT_ANGLE;
				waiting = snapper->waitCount;
				snapper->SetState(MOVING);
				spec.PanMoveAbs(snapper->targetAngle);
				break;

			case MOVING:
			case STOPPING:

				if(!spec.GetPanAngle(theta)) {
					ROS_ERROR("TimerCallback: Error in getting specatator angle");
					snapper->SetState(INACTIVE);
					return;
				}

				if(std::abs(snapper->targetAngle-theta) > SnapAround::EPSION) {
					ROS_ERROR("TimerCallback: target angle not reaching ... curr state is %d!!!", snapperState);
					break;
				}

				ROS_DEBUG("TimerCallback: target angle reached");

				if(snapperState == STOPPING)
					snapper->SetState(INACTIVE);
				else
					snapper->SetState(WAITING);
				break;

			case WAITING:

				if(waiting > 0) {
					ROS_DEBUG("TimerCallback: waiting (remaing %d)", waiting );
					waiting --;
					break;
				}

				if(snapper->targetAngle+snapper->stepAngle + 5.0 > LIMIT_ANGLE) {	// too close to the target
					snapper->targetAngle = snapper->originalAngle;
					snapper->SetState(STOPPING);
					spec.PanMoveAbs(snapper->targetAngle);
					break;
				}

				ROS_DEBUG("TimerCallback: waiting done");

				snapper->targetAngle += snapper->stepAngle;
				waiting = snapper->waitCount;
				snapper->SetState(MOVING);
				spec.PanMoveAbs(snapper->targetAngle);
				break;

		}

	}

	prevMovingState = currMovingState;

}

spectator::SnapAround::SnapAround(spectator::Spectator & _spec)
	: spec(_spec)
	, state(INACTIVE)
	, captureMode(client_interface::DB_Handler::IMAGE_MODE)
{


}

bool spectator::SnapAround::Activate(double _stepAngle, double _waitTimeSec) {


	if (state != INACTIVE)
		return false;

	state = INIT;
	stepAngle = _stepAngle;
	waitTimeSec = _waitTimeSec;
	double wait = waitTimeSec/period;
	waitCount = int(wait);
	ROS_DEBUG("SnapAround::Activate: waitCount is %d", waitCount);


	if (wait > double(waitCount)+0.5)		// roundig to nearest interger
		waitCount ++;

	targetAngle = -180.0;


}

void spectator::SnapAround::Inactivate() {
	state = INACTIVE;
	spec.PanStop();

}

spectator::SnapAround::state_t spectator::SnapAround::GetState() const {
	return state;
}

void spectator::SnapAround::SetState(spectator::SnapAround::state_t _state) {
	state = _state;
}

void spectator::SnapAround::SetCaptureMode(client_interface::DB_Handler::capture_mode_t _captureMode) {
	captureMode = _captureMode;
}

client_interface::DB_Handler::capture_mode_t spectator::SnapAround::GetCaptureMode() const {
	return captureMode;
}
