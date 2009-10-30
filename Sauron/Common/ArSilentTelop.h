#pragma once
#include "Aria.h"

/// Mode for teleoping the robot with joystick + keyboard
class ArModeSilentTeleop : public ArModeTeleop
{
public:
  /// Constructor
	AREXPORT ArModeSilentTeleop(ArRobot *robot, const char *name, char key, char key2) :
	  ArModeTeleop(robot, name, key, key2){
	  }
	  AREXPORT virtual void help(void) {
	  }

	  AREXPORT virtual void userTask(void) {
	  }
};