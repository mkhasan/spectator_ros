/*
 * spectator.h
 *
 *  Created on: Sep 27, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_SPECTATOR_CPP_INCLUDE_SPECTATOR_SPECTATOR_H_
#define CATKIN_WS_SRC_SPECTATOR_CPP_INCLUDE_SPECTATOR_SPECTATOR_H_




#include "serial_interface/serial.h"

#include "spectator/joint.h"

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include <string>
#include <vector>







#define PAN_CCW 1
#define TILT_CCW 1

#define MAX_VELOCITY 5.0

#define INTER_CMD_DELAY 0.01


namespace spectator {


class Spectator
{
public:
	static const std::string REAL;



private:
	const int MOVING_MASK = 0x0010;
	friend class Joint;

	const int TIME_OUT_IN_MS = 1000;

	ros::NodeHandle nh;

	std::vector<double> homingOffsets;



public:


	typedef enum {NOT_CONN, CONN, HOMING, READY, MAX_STATE} state_t;
	static const std::string STATE_DESC[MAX_STATE];
	static const std::string ROBOT_PREFIX;

private:
	state_t currentState;
private:
	Joint *pan, *tilt;
	std::string mode;
	std::string port;
	int padAddr;
	int tiltAddr;

	boost::shared_ptr<serial::Serial> mySerial;
public:
	Spectator(ros::NodeHandle& nh, const std::vector<double>&);

	~Spectator();

	Joint &getJoint(const std::string & name);

	bool GetAngles(double &panAngle, double &tiltAngle) const;
	bool GetPanAngle(double & angle) const;
	bool GetTiltAngle(double & angle) const;

	bool GetPanVelocity(double & v) const;
	bool GetTiltVelocity(double & v) const;

	bool SetAngles(double panAgnle, double tiltAngle);
	bool SetPanAngle(double angle);
	bool SetTiltAngle(double angle);

	bool SetPanVelocity(double velocity) const;
	bool SetTiltVelocity(double velocity) const;

	bool PanMoveAbs(double target) const;
	bool TiltMoveAbs(double target) const;

	bool JointMoveAbs(const std::string & jointName, double target);

	void SetLimits(const std::vector<double>& lowerLimit, const std::vector<double>& upperLimit);

	bool PanStop() const;
	bool TiltStop() const;

	bool GoLeft() const;
	bool GoLeft(double s) const;
	bool GoRight() const;
	bool GoRight(double s) const;
	bool GoUp() const;
	bool GoUp(double s) const;
	bool GoDown() const;
	bool GoDown(double s) const;

	bool GetStatus(const std::string& joint, int & status) const ;
	bool GetAlarm(const std::string& joint, int & alarm) const ;
	bool GetStatus(int jointId, int & status) const ;
	bool GetAlarm(int jointId, int & al) const ;

	bool AlarmReset(int jointId) const ;
	bool AlarmReset(const std::string& jointName) const;

	bool GetStatus(int *status) const ;

	bool EnableJoint(const std::string & jointName) const;
	bool EnableJoint(int jointId) const;

	bool DisableJoint(const std::string & jointName) const;
	bool DisableJoint(int jointId) const;

	bool SetVelocities(const std::vector<float> & velo);

	bool Homing(double panOffset, double tiltOffset);
	bool Homing();
	std::string GetPort() const;

	static std::vector<std::string>GetUSB_SerialList();

	void GetCurrentState(std::string &) const;
	void GetCurrentState(state_t &) const;
	state_t GetCurrentState() const;
	void SetCurrentState(state_t state);
};


class SpException : public std::exception
{
  // Disable copy constructors
  SpException& operator=(const SpException&);
  std::string file_;
  int line_;
  std::string e_what_;
  int errno_;
public:
  explicit SpException (std::string file, int line, int errnum)
    : file_(file), line_(line), errno_(errnum) {
      std::stringstream ss;
#if defined(_WIN32) && !defined(__MINGW32__)
      char error_str [1024];
      strerror_s(error_str, 1024, errnum);
#else
      char * error_str = strerror(errnum);
#endif
      ss << "Spectator Exception (" << errno_ << "): " << error_str;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  explicit SpException (std::string file, int line, const char * description)
    : file_(file), line_(line), errno_(0) {
      std::stringstream ss;
      ss << "Spectator Exception: " << description;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  virtual ~SpException() throw() {}
  SpException (const SpException& other) : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

  int getErrorNumber () const { return errno_; }

  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};


}
int test_spectator();

#endif /* CATKIN_WS_SRC_SPECTATOR_CPP_INCLUDE_SPECTATOR_CPP_SPECTATOR_H_ */
