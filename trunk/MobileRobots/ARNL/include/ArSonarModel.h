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
/* ***************************************************************************
 * 
 * File: ArRobotAndLaser.h
 * 
 * Function: Header file for the robotandlaser.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 3 2002.
 *
 *****************************************************************************/
#ifndef ARSONARMODEL_H
#define ARSONARMODEL_H

#include <iostream>
#include <vector>
#include "Aria.h"

using namespace std;
class ArSonarModel
{

public:

  /// Base Constructor.
  ArSonarModel(double lambdaF, double lambdaR, double sigma, double maxRange,
	       double betaMin, double betaMax) : 
    myLambdaF(lambdaF), myLambdaR(lambdaR), mySigma(sigma), 
    myMaxRange(maxRange), myBetaMin(betaMin), myBetaMax(betaMax),
    myTable(NULL){}
  ArSonarModel() : 
    myLambdaF(0.00002), myLambdaR(0.00005), mySigma(30), 
    myMaxRange(10000), myBetaMin(0.8), myBetaMax(0.6), myTable(NULL){}
  /// Base Destructor.
  ~ArSonarModel(void);

  /// Set lambdaF
  void     setLambdaF(double lambdaF) {myLambdaF = lambdaF;}
  /// Set lambdaR
  void     setLambdaR(double lambdaR) {myLambdaR = lambdaR;}
  /// Set sigma
  void     setSigma(double sigma) {mySigma = sigma;}
  /// Set maxRange
  void     setMaxRange(double maxRange) {myMaxRange = maxRange;}
  /// Set beta min
  void     setBetaMin(double betaMin) {myBetaMin = betaMin;}
  /// Set beta max
  void     setBetaMax(double betaMax) {myBetaMax = betaMax;}
  /// Set all.
  void     setParams(double lambdaF, double lambdaR, 
		     double sigma, double maxRange,
		     double betaMin, double betaMax)
  { myLambdaF = lambdaF; myLambdaR = lambdaR; mySigma = sigma;
  myMaxRange = maxRange; myBetaMin = betaMin; myBetaMax = betaMax;}
  /// Get lambdaF
  double   getLambdaF(void) {return myLambdaF;}
  /// Get lambdaR
  double   getLambdaR(void) {return myLambdaR;}
  /// Get sigma
  double   getSigma(void) {return mySigma;}
  /// Get maxRange
  double   getMaxRange(void) {return myMaxRange;}
  /// Get beta min
  double   getBetaMin(void) {return myBetaMin;}
  /// Get beta max
  double   getBetaMax(void) {return myBetaMax;}
  /// Get table dimension 0 angle.
  int      getTableDim0(void) {return myTableDim0;}
  /// Get table dimension 1 sonar reading.
  int      getTableDim1(void) {return myTableDim1;}
  /// Get table dimension 2 true range.
  int      getTableDim2(void) {return myTableDim2;}
  /// Get table resolution 0 angle.
  double   getTableRes0(void) {return myTableRes0;}
  /// Get table resolution 1 sonar reading.
  double   getTableRes1(void) {return myTableRes1;}
  /// Get table resolution 2 true range.
  double   getTableRes2(void) {return myTableRes2;}
  /// Get table 
  double*** getTable(void) {return myTable;}
  /// Compute beta.
  double   beta(double x) {return (myBetaMin + (x) / (myMaxRange) * 
				   (myBetaMax - myBetaMin));}
  /// Make a s, d, phi table of probs.
  bool     makeProbabilityTable(double rangeRes, double angleRes, 
				double incidentLimit);
  /// Get probability for given reading given target at distance and incidence
  double   getProb(double phi, double distance, double reading);
  /// Return on axis prob model for given sensor reading.
  std::vector<double> onAxisSonarProb(double res, double reading);
  /// Return off prob model for given sensor reading.
  std::vector<double> offAxisSonarProb(double res, double reading);

private:

  double   myLambdaF;
  double   myLambdaR;
  double   mySigma;
  double   myMaxRange;
  double   myBetaMin;
  double   myBetaMax;
  double*** myTable;
  int      myTableDim0;
  int      myTableDim1;
  int      myTableDim2;
  double   myTableRes0;
  double   myTableRes1;
  double   myTableRes2;

};

#endif // ARSONARMODEL.H
