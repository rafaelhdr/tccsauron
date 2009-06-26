/*
MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.7.1

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

All Rights Reserved.

MobileRobots Inc does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
MobileRobots
10 Columbia Drive
Amherst, NH 03031
800-639-9481

*/
#ifndef ARMULTIROBOT_H
#define ARMULTIROBOT_H

#include "Aria.h"
#include "ArNetworking.h"
#include "arnlInternal.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"
#include "ArMultiRobotFlags.h"

///  handles the server side of dispensing information for multi robot driving
/**
   This class handles the server side of dispensing information to the
   central server.  It has no callback for the packet because it
   broadcasts it in each user task set
**/
class ArServerHandlerMultiRobot
{
public:
  AREXPORT ArServerHandlerMultiRobot(ArServerBase *server, ArRobot *robot,
				     ArPathPlanningTask *pathTask,
				     ArBaseLocalizationTask *locTask,
				     ArMapInterface *arMap);
  AREXPORT ~ArServerHandlerMultiRobot();
  /// gets the information about the robot's radius and path radius
  AREXPORT void multiRobotInfo(ArServerClient *client, ArNetPacket *packet);
  /// Gets the robot's precedence class
  AREXPORT int getPrecedenceClass(void);
  /// Sets the robot's precedence class
  AREXPORT void setPrecedenceClass(int precedenceClass = 0);
  /// Override map name (if you set this it'll use this instead of the real map name)
  AREXPORT void overrideMapName(const char *overrideMapName);
  /// processes our config and updates everyone that it changed
  AREXPORT bool processFile(char *errorBuffer, size_t errorBufferLen);
protected:
  /// broadcasts the informationabout the position and path
  AREXPORT void userTask(void);
  /// Called when we get a new goal
  AREXPORT void newGoal(void);

  ArMutex myMutex;
  ArServerBase *myServer;
  ArRobot *myRobot;
  ArPathPlanningTask *myPathTask;
  ArBaseLocalizationTask *myLocTask;
  ArMapInterface *myMap;
  int myPrecedenceClass;
  std::string myOverrideMapName;

  ArNetPacket myMultiRobotInfoPacket;

  int myRobotRadius;
  int myPathRadius;
  int myRealRobotRadius;
  int myRobotRadiusAdjustment;
  int myPathRadiusAdjustment;
  int myPathMaxLength;
  int myPathResolution;
  int myPathResolutionAdjustment;
  int myShortPathLength;

  bool myUseLegacyMode;
  unsigned char myPathNum;
  unsigned char myMultiRobotCapabilityFlags1;

  ArFunctor2C<ArServerHandlerMultiRobot, 
	      ArServerClient *, ArNetPacket *> myMultiRobotInfoCB;
  ArRetFunctor2C<bool, ArServerHandlerMultiRobot, 
                            char *, size_t> myProcessFileCB;
  ArFunctorC<ArServerHandlerMultiRobot> myUserTaskCB;
  ArFunctorC<ArServerHandlerMultiRobot> myNewGoalCB;
  
};

/// This is the class that will make range data from multiple robots
class ArMultiRobotRangeDevice : public ArRangeDevice
{
public:
  /// Constructor 
  AREXPORT ArMultiRobotRangeDevice(ArServerBase *serverBase);
  /// Destructor
  AREXPORT ~ArMultiRobotRangeDevice();
  /** Override ArRangeDevice::applyTransform() to skip transform from local to global coordinates (already in global coords.)
      @internal
  */
  virtual void applyTransform(ArTransform trans, bool doCumulative) 
	{}  
protected:
  /// Process our robot poses packet
  void netRobotPoses(ArServerClient *client, ArNetPacket *packet, bool);
  /// Process our robot paths packet
  void netRobotPaths(ArServerClient *client, ArNetPacket *packet);
  /// Gets the other robot poses and radii in a threadsafe manner
public:
  AREXPORT std::list<ArMultiRobotPoseAndRadius> getOtherRobots(void);
  /// Gets the callback for other robots
  AREXPORT ArRetFunctor<std::list<ArMultiRobotPoseAndRadius> >*getOtherRobotsCB(void) { return &myOtherRobotsCB; }
protected:
  ArServerBase *myServer;
  
  std::list<ArMultiRobotPoseAndRadius> myOtherRobots;
  ArFunctor3C<ArMultiRobotRangeDevice, ArServerClient *, 
	      ArNetPacket *, bool> myNetRobotPosesCB;
  ArFunctor3C<ArMultiRobotRangeDevice, ArServerClient *, 
	      ArNetPacket *, bool> myNetRobotPosesWithRadiusCB;
  ArFunctor2C<ArMultiRobotRangeDevice, ArServerClient *,
	      ArNetPacket *> myNetRobotPathsCB;
  ArRetFunctorC<std::list<ArMultiRobotPoseAndRadius>, ArMultiRobotRangeDevice> myOtherRobotsCB;
};

#endif 
