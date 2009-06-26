/*
MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.7.0

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
/*****************************************************************************
 * 
 * File: ArRobotAndSonar.h
 * 
 * Function: Header file for the robotandsonar.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. March 3 2005.
 *
 *****************************************************************************/
#ifndef ARROBOTANDSONAR_H
#define ARROBOTANDSONAR_H

#include <stdio.h>
#include <math.h>
#include "Aria.h"
#include "ArOccGrid.h"
#include "ArSonarModel.h"

/* 
  @class ArRobotAndSonar.
  @internal This class is used internally by SonArnl.
  @brief Class holds details about the robot and laser pertaining to
         localization. 
*/
class ArRobotAndSonar
{

public:

  /// Base Constructor.
  ArRobotAndSonar(ArRobot* robot, ArSonarDevice* sonar,
		  double maxRange, double aperture);
  /// Base Destructor.
  ~ArRobotAndSonar(void);

  /// Pass the thing to ArSonarModel as is.
  bool     makeProbabilityTable(double rangeRes, double angleRes, 
				double incidenceLimit);
  /// Set up the configuration of the sonar data.
  void     setSonarParams(double maxRange, double aperture);
  /// Get the sonar count.
  int      getSonarCount(void) {return mySonarCount;}
  /// Get the sonar origin
  ArPose   getLocalSonarOrigin(int i) 
  {
    int n = myLocalSonarOrigin.size();
    if(i < n)
      return myLocalSonarOrigin[i];
    else
      return ArPose();
  }
  /// Get the sonar ray
  ArPose   getLocalSonarRay(int i) 
  {
    int n = myLocalSonarRay.size();
    if(i < n)
      return myLocalSonarRay[i];
    else
      return ArPose();
  }
  /// Get the list of sonars taken at the same instant.
  std::vector<int> getSonarSame(void) {return mySonarSame;}
  /// Return the pointer to the range data from the current scan.
  int      sonarData(int sr) 
  {return (sr < (int)mySonarData.size()) ? mySonarData[sr] : 0;}
  /// Return the pointer to the X data from the current sonar.
  ArPose   sonarXY(int sr)
  {return (sr < (int)mySonarData.size()) ? mySonarXY[sr] : ArPose();}
  /// Store the robot's Pose at which the current scan was taken.
  void     setPoseTaken(ArPose a) {myPoseTaken = a;}
  /// Store the robot's encoder pose at which the current scan was taken.
  void     setEncoderPoseTaken(ArPose a) {myEncoderPoseTaken = a;}
  /// Get the robot's Pose at which the current scan was taken.  
  ArPose   getPoseTaken(void) {return myPoseTaken;}
  /// Get the robot's encoder pose at which the current scan was taken.  
  ArPose   getEncoderPoseTaken(void) {return myEncoderPoseTaken;}
  /// Converts sonar ranges to global points.
  bool     scanToGlobalCoords(ArPose rp, std::vector<ArPose>& xyLrf);
  /// Scans the data from the sensor into our data structure.
  bool     scanSonarIntoArray(ArRangeDevice* aSonar);
  /// Finds the intersection point to the env for the sonarN
  bool     findIntersectionDetails(int sonarN,
				   ArPose robotPose,
				   double maxRange,
				   std::vector<ArLineSegment>& lines,
				   double& intDistance,
				   double& intIncidentAngle,
				   ArPose& intPoint,
				   ArPose& intOrigin);
  /// Computes the probability of sonarN reading at the given pose.
  double   findProbOfReadingAtPose(int sonarN,
				   ArPose robotPose,
				   double range,
				   std::vector<ArLineSegment>& lines,
				   double& closeness);
  /// Finds the closest few lines.
  std::vector<ArLineSegment> findClosestLines(ArPose robotPose,
				      std::vector<ArLineSegment>& allLines,
					      double maxRange);

  /// Gets the pose taken for sonar no n.
  ArPose   getSonarPoseTaken(int n)
  { 
    int m = mySonarPoseTaken.size(); 
    if(m > n) 
      return mySonarPoseTaken[n];
    else 
      return ArPose(); 
  }
  /// Gets the encoder pose taken for sonar no n.
  ArPose   getSonarEncoderPoseTaken(int n)
  { 
    int m = mySonarEncoderPoseTaken.size(); 
    if(m > n) 
      return mySonarEncoderPoseTaken[n];
    else 
      return ArPose(); 
  }

  /// Get the time at which the sonar no n was taken.
  ArTime   getSonarTimeTaken(int n)
  { 
    int m = mySonarTimeTaken.size(); 
    if(m > n) 
      return mySonarTimeTaken[n];
    else
    return ArTime();
  }

  /// Get the sonar model pointer.
  ArSonarModel* getSonarModel(void) {return mySonarModel;}

private:

  ArSonarDevice*      mySonarDev;

  std::vector<int>    mySonarData;
  std::vector<ArPose> mySonarXY;

  int                 mySonarCount;
  double              mySonarLambdaF;
  double              mySonarLambdaR;
  double              mySonarSigma;
  double              mySonarMaxRange;
  double              mySonarBetaMin;
  double              mySonarBetaMax;
  double              mySonarAperture;

  ArPose              myPoseTaken;
  ArPose              myEncoderPoseTaken;
  std::vector<ArPose> mySonarPoseTaken;
  std::vector<ArPose> mySonarEncoderPoseTaken;
  std::vector<ArTime> mySonarTimeTaken;
  std::vector<ArPose> mySonarPose;
  std::vector<ArPose> myLocalSonarRay;
  std::vector<ArPose> myLocalSonarOrigin;
  ArSonarModel*       mySonarModel;
  std::vector<int>    mySonarSame;
  ArMutex             myMutex;
};

#endif // ARROBOTANDSONAR.H
