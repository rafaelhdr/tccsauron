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
/* ***************************************************************************
 * 
 * File: ArSystemError.h
 * 
 * Function: Header file for the systemerror.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 13 2002.
 *
 *****************************************************************************/
#ifndef ARSYSTEMERROR_H
#define ARSYSTEMERROR_H

#include <stdio.h>

#include "Aria.h"

/* 
  ArSystemError
  Class holds Localization error and robot motion error params.
*/
class ArSystemError
{
public:

  /// Base Constructor.
  AREXPORT ArSystemError(void);
  /// Base Destructor.
//  ~ArSystemError(void);

  /// Sets the 3x3 Covariance matrix.
  AREXPORT bool     setRobotPoseErrorParams(double cm[3][3]);
  /// Sets the standard deviations in the pose, x, y, theta.
  AREXPORT bool     setRobotPoseErrorParams(double stdx, double stdy, double stdt);
  /// Sets the motion error factors due to distance, angle and heading.
  AREXPORT bool     setRobotMotionErrorParams(double kr, double kt, double kd);
  /// Sets the sensor belief factor (0-1).
  AREXPORT bool     setSensorBelief(double b);
  /// Returns the standard deviation in X.
  AREXPORT double   getRobotPoseErrorXParam(void){return myStdX;}
  /// Returns the standard deviation in Y.
  AREXPORT double   getRobotPoseErrorYParam(void){return myStdY;}
  /// Returns the standard deviation in Theta.
  AREXPORT double   getRobotPoseErrorTParam(void){return myStdT;}
  /// Returns the motion error factor for distance per distance.
  AREXPORT double   getRobotMotionErrorKrParam(void){return myKr;}
  /// Returns the motion error factor for angle per angle
  AREXPORT double   getRobotMotionErrorKtParam(void){return myKt;}
  /// Returns the motion error factor for angle per mm.
  AREXPORT double   getRobotMotionErrorKdParam(void){return myKd;}
  /// Returns the sensor belief factor.
  AREXPORT double   getSensorBelief(void) {return mySensorBelief;}

private:

  double myCovarPose[3][3];
  double myStdX, myStdY, myStdT;
  double myKr, myKt, myKd;
  double mySensorBelief;

};

#endif // ARSYSTEMERROR.H
