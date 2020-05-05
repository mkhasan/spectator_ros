/*
 * joint.h
 *
 *  Created on: Oct 2, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_JOINT_H_
#define CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_JOINT_H_


#define EPSILON 0.0001

namespace spectator {

class Spectator;




class Joint {


	const int RESOLUTION = 20000;
	const double DEG_PER_COUNT = (360.0/RESOLUTION);
	const double GEAR_RATIO = 4.0;
	const char ACK = '%';
	const char BF_ACK = '*';
	const char NACK = '?';

	friend class Spectator;

	double angle;
	bool realMode;

	int id;
	Spectator &spectator;

	int offset;
	int signCCW;		// determins if CCW rotation will increase angle or not

	double lowerLimit;
	double upperLimit;

public:

	int test;
	std::string name;

	Joint(int id, Spectator &spectator);
	~Joint();
	double read();
	void actuate(double effort);

	void SetOffset(int offset);
	void SetCCW(int sign);


	bool GetAngle(double & angle) const;

	bool GetVelocity(double & v) const;



	bool GetEncoderPosition(int & count) const;

	bool SetAngle(double angle);

	bool SetVelocity(double s) const;

	bool SetHomeSearchVelocity(double s) const;

	bool GotoAbs(double angle);

	bool GotoRelative(double angle);

	void ExecuteServoCmd(const char *cmd) const;

	void ExecuteServoCmd(const char *cmd, int param) const;

	void ExecuteServoCmd(const char *cmd, double param) const;

    void ExecuteServoCmd(const char *cmd, int * pResult) const;

	void ExecuteServoCmd(const char *cmd, double * pResult) const;

	void SetLimit(double lower, double upper);

	void GetLimit(double &lower, double &upper) const;

	bool Stop();

	bool AlarmReset();

	bool GetStatus(int & status) const;

	bool GetAlarm(int & alarm) const;

	bool Enable() const;

	bool Disable() const;

	bool Homing();

	inline int DegToCount(double _angle, int offset) {
		return offset+_angle*signCCW*GEAR_RATIO/DEG_PER_COUNT;
	}


private:
	inline bool CheckAck(char ch) const {
		return (ch == ACK || ch == BF_ACK);
	}
};

}	// end of namespace spectator


#endif /* CATKIN_WS_SRC_SPECTATOR_INCLUDE_SPECTATOR_JOINT_H_ */
