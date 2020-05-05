/*
 * spectator_key.cpp
 *
 *  Created on: Sep 30, 2018
 *      Author: usrc
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "spectator/cmd_handler.h"



#include <termios.h>
#include <unistd.h>

#include <iostream>

#include <stdio.h>


using namespace std;

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_LEFT 68
#define KEY_RIGHT 67




class mqtt_publisher : public mosqpp::mosquittopp
{
	public:
	mqtt_publisher(const char *id, const char *host, int port):mosquittopp(id) {
		int keepalive = 60;

		/* Connect immediately. This could also be done by calling
		 * mqtt_tempconv->connect(). */
		connect(host, port, keepalive);

	}
	~mqtt_publisher() {
		disconnect();
	}

};



int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char **argv) {



	ros::init(argc, argv, "spectator_key");

	mosqpp::lib_init();
	mqtt_publisher publisher("spectator_key", "127.0.0.1", 1883);
	std::system("clear");
	string str;
	while (ros::ok()) {
	  int c = getch();   // call your non-blocking input function
	  std::system("clear");


	  switch (c) {
	  	  case KEY_UP:
	  		  cout << "Up" << endl;
	  		  str = "0.0 1.0";
	  		  break;

	  	  case KEY_DOWN:
	  		  cout << "Down" << endl;
	  		  str = "0.0 -1.0";
	  		  break;

	  	  case KEY_LEFT:
	  		  cout << "Left" << endl;
	  		  str = "-1.0 0.0";
	  		  break;

	  	  case KEY_RIGHT:
	  		  cout << "Right" << endl;
	  		  str = "1.0 0.0";
	  		  break;

	  	  default:
	  		  cout << "Center" << endl;
	  		  str = "0.0 0.0";

	  }

	  if(str.length()) {
		  publisher.publish(NULL, RIGHT_STICK, str.length(), str.c_str());
	  }
	  else {
		  switch(c) {
		  	  case 'g':  publisher.publish(NULL, RIGHT_STICK, str.length(), str.c_str());
		  }
	  }
	}

	mosqpp::lib_cleanup();
	ros::spin();


}




