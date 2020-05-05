/*
 * snap_around.h
 *
 *  Created on: Oct 2, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_SNAP_AROUND_H_
#define CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_SNAP_AROUND_H_

#include "ros/ros.h"

#include "spectator/spectator.h"
#include "client_interface/db_handler.h"

namespace spectator {

class SnapAround {

public:
	typedef enum {INACTIVE, INIT, WAITING, MOVING, STOPPING} state_t;

private:
	const static double EPSION;

	double stepAngle;
	double waitTimeSec;
	int waitCount;
	double originalAngle;
	double currAngle;
	double targetAngle;
	Spectator & spec;
	state_t state;
	client_interface::DB_Handler::capture_mode_t captureMode;


public:
	SnapAround(Spectator & spec);
	bool Activate(double stepAngle, double waitTimeSec);
	void Inactivate();
	state_t GetState() const;
	void SetState(state_t state);

	/*
	void Snap(double stepAngle, double waitTimeSec);
	void Reset();
	bool IsSnapping();
	*/


	static void TimerCallback(const ros::TimerEvent &);
	static double period;

	void SetCaptureMode(client_interface::DB_Handler::capture_mode_t captureMode);
	client_interface::DB_Handler::capture_mode_t GetCaptureMode() const;



};

}		// end of namespace



#endif /* CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_SNAP_AROUND_H_ */
