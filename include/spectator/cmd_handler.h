/*
 * cmd_handler.h
 *
 *  Created on: Sep 30, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_CMD_HANDLER_H_
#define CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_CMD_HANDLER_H_


#include "client_interface/client_interface.h"
#include "spectator/spectator.h"


#include <mosquittopp.h>

/*
#define RIGHT_STICK "KICT_MP/CLIENT/RIGHT_STICK"
#define RIGHT_BUTTON_GROUP "KICT_MP/CLIENT/RIGHT_BUTTON_GROUP"
#define SPECTATOR_POSITION "KICT_MP/SPECTATOR/POSITION"

#define SPECTATOR_GET_ANGLES "KICT_MP/CLIENT/GET_ANGLES"
#define SPECTATOR_GET_ANGLES_START "KICT_MP/CLIENT/GET_ANGLES_START"
#define SPECTATOR_GET_ANGLES_STOP "KICT_MP/CLIENT/GET_ANGLES_STOP"

#define IMU_START "KICT_MP/CLIENT/IMU_START"
#define IMU_STOP "KICT_MP/CLIENT/IMU_STOP"
#define IMU_VALUES "KICT_MP/IMU"

#define GPS_START "KICT_MP/CLIENT/GPS_START"
#define GPS_STOP "KICT_MP/CLIENT/GPS_STOP"
*/

//#define SPECTATOR_GET_ANGLES_START "KICT_MP/CLIENT/GET_IMU_START"


class CmdHandler : public mosqpp::mosquittopp
{

	const int SKIP_COUNT = 0;
	const double ANGLE_STEP = 10.0;
	typedef enum {LEFT, RIGHT, UP, DOWN, CENTER} dir_t;



	dir_t dir, sel;

	spectator::Spectator & spectator;

	dir_t GetDir(double x, double y) const;
	dir_t GetSel(char ch) const;

	int skipCmd;

	public:
		CmdHandler(spectator::Spectator & spectator, const char *id, const char *host, int port);
		~CmdHandler();

		void on_connect(int rc);
		void on_message(const struct mosquitto_message *message);

	private:
		void HandleStick(const struct mosquitto_message *message);
		void HandleButtonGroup(const struct mosquitto_message *message);

};




#endif /* CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_CMD_HANDLER_H_ */
