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
#ifndef ARBASELOCALIZATIONTASK_H
#define ARBASELOCALIZATIONTASK_H

#include "Aria.h"
#include "ArMatrix.h"

/**
   This class has the similar items of ArLocalizationTask and
   ArSonarLocalizationTask, right now thats just one enum, later it
   may grow to become more.
**/

class ArBaseLocalizationTask: public ArASyncTask
{
public:
  ArBaseLocalizationTask(const char *name) : 
    myGetRobotHomeCB(this, &ArBaseLocalizationTask::getRobotHome),
    myGetPoseInterpPositionCB(this, 
			   &ArBaseLocalizationTask::getPoseInterpPosition)
  { 
    myName = name; 
  }
  /// State of the path plan, accessible using getState().
  enum LocalizationState
  {
    NOT_INITIALIZED,        /// Task not initialized
    LOCALIZATION_FAILED,    /// Failed localization.
    LOCALIZATION_SUCCESS,   /// Failed to reach goal because superseded.
    LOCALIZATION_INIT_COMPUTING, /// Localization init in progress.
    LOCALIZATION_MOVE_COMPUTING, /// Localization move in progress.
    LOCALIZATION_IDLE,      /// Localization idled.
    INVALID                 /// Invalid state.
  };
  /// Function returns what the current mean pose and its variance.
  virtual bool findLocalizationMeanVar(ArPose& mean, ArMatrix& var)
  {
    return false;
  }
  /// Used to switch off moveTo inside the child localization tasks.
  virtual void setCorrectRobotFlag(bool f) 
  {
    return;
  }
  /// Used to set the robot pose usually at the start of localization. 
  /// This one with a spread around the set pose.
  virtual void setRobotPose(ArPose pose, 
				     ArPose spread = ArPose(-1., -1., -1.), 
				     int nSam = -1)
  {
    return;
  }
  /// Finds if the robot is lost.
  virtual bool getRobotIsLostFlag() 
  {
    return false;
  }
  /// Gets the robot home pose
  /// This pose is the initial best localized position determined by 
  /// localizeRobotAtHomeBlocking() or localizeRobotAtHomeNonBlocking() methods
  virtual ArPose getRobotHome() 
  {
    return ArPose();
  }
  /// Get a functor that when called returns the robot's home pose
  /// This pose is the initial best localized position determined by 
  /// localizeRobotAtHomeBlocking() or localizeRobotAtHomeNonBlocking() methods
  ArRetFunctor<ArPose>* getRobotHomeCallback() 
  { 
    return &myGetRobotHomeCB; 
  }
  /// @deprecated
  ArPose getHomePose() 
  { 
    return getRobotHome(); 
  }
  /// Localize robot at start or while initialization.
  virtual bool localizeRobotAtHomeBlocking(double distSpread,
						    double angleSpread)
  {
    return false;
  }
  /// Try localizing at each home position, choosing the best to be the 
  /// current and the stored "home" position.
  virtual bool localizeRobotAtHomeBlocking() 
  {
    return false;
  }
  /// Gets the current sample poses if relevant.
  virtual std::list<ArPose> getCurrentSamplePoses() 
  {
    std::list<ArPose> l; return l;
  }
  /// Get the localization state.
  virtual LocalizationState getState(void)
  {
    return NOT_INITIALIZED;
  }
  /// Get the localization score.
  virtual double getLocalizationScore(void) 
  {
    return 0.0;
  }
  /// Get the localization threshold.
  virtual double getLocalizationThreshold(void) 
  {
    return 0.0;
  }
  /// Get the localization idle
  virtual bool checkLocalizationIdle(void) 
  {
    return (getState() == LOCALIZATION_IDLE);
  }
  /// Set the localization idle
  virtual void setLocalizationIdle(bool f)
  {
    return;
  }
  /// Set the localization threshold.
  virtual void setLocalizationThreshold(double t) 
  {
    return;
  }
  /// Get the robot encoder to the localization pose transform.
  virtual ArTransform getEncoderToLocalizationTransform(void) 
  {
    ArTransform t;
    return t;
  }
  /// Gets the name of this task
  const char *getName(void) 
  { 
    return myName.c_str(); 
  }
  /// Gets the position the robot was at at the given timestamp
  /** @see ArInterpolation::getPose 
   */
  virtual int getPoseInterpPosition(ArTime timeStamp, ArPose *position)
  { 
    return -4; 
  }

  /// Gets the callback that will call getPoseInterpPosition
  ArRetFunctor2<int, ArTime, ArPose *> *getPoseInterpPositionCallback(void)
  { 
    return &myGetPoseInterpPositionCB; 
  }
  /// Convert LLA to Robot
  virtual bool convertLLA2RobotCoords(double lat, double lon, double alt,
				      double& ea, double& no, double& up)
  {
    return false;
  }
  /// Convert Robot to LLA.
  virtual bool convertRobot2LLACoords(double ea, double no, double up,
				      double& lat, double& lon, double& alt)
  {
    return false;
  }

protected:
  std::string myName;
  ArRetFunctorC<ArPose, ArBaseLocalizationTask> myGetRobotHomeCB;
  ArRetFunctor2C<int, ArBaseLocalizationTask, ArTime, 
		 ArPose *> myGetPoseInterpPositionCB;
};

#endif // ARBASELOCALIZATIONTASK_H
