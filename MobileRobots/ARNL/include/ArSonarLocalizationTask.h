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
/* ****************************************************************************
 * 
 * File: ArSonarLocalizationTask.h
 * 
 * Function: Header file for the localizationtask.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. January 10 2005.
 *
 *****************************************************************************/
#ifndef ARSONARLOCALIZATIONTASK_H
#define ARSONARLOCALIZATIONTASK_H

#include "Aria.h"
#include "ArNetworking.h"
#include "ArRobotPoseProb.h"
#include "ArRobotPoseSamples.h"
#include "ArRobotAndSonar.h"
#include "ArOccGrid.h"
#include "ArBaseLocalizationTask.h"

#include <set>
#include <vector>
#include <algorithm>

/*!
  @class ArSonarLocalizationTask
  @brief Task that performs localization of the robot with sonar range sensors in a seperate asynchronous thread. 

  The sonar localization task can be used to localize a robot in a given line map using the ring of sonar rangefinders. 

  ARNL's sonar localization uses Monte Carlo Localization algorithm to accurately localize the robot in the given line map using its sonar data. The localization task is meant to be initialized and run in a separate thread in ARIA. It can be used with a real robot or with a simulator.

  In order to get the localization task thread going, the ArSonarLocalizationTask class needs an instantiated ArRobot, ArSonarDevice and a line map of the robot's environment in an ArMap object (or loaded from file). The output of the localization will be reflected directly in the pose of the ArRobot. (unless explicitly set to not do so)

  ArSonarLocalizationTask automatically creates and runs its
  background thread when constructed.

*/
class ArSonarLocalizationTask: public ArBaseLocalizationTask
{
  public:

  /// Base constructor with all the necessary inputs.
  AREXPORT ArSonarLocalizationTask(ArRobot* robot,
				   ArSonarDevice* sonar, char* mapName);
  /// Base constructor with all the necessary inputs.
  AREXPORT ArSonarLocalizationTask(ArRobot* robot,
				   ArSonarDevice* sonar, ArMapInterface* ariaMap);
  /// Base destructor.
  AREXPORT virtual ~ArSonarLocalizationTask(void);

  /// Localization function mainly for initialization of robot 
  /// at a given pose.
  AREXPORT bool   localizeRobotInMapInit(ArPose given, int numSamples,
					 double stdX, double stdY, double stdT,
					 double thresFactor, bool warn = true,
					 bool setInitializedToFalse = true);
  /// Function used to do the localization after motion (normally automatic).
  AREXPORT bool   localizeRobotInMapMoved(int numSamples,
					  double distFactor, 
					  double angFactor,
					  double thresFactor);
  /** @brief Try initial localization at each home point in the map, set the robot pose to the point with best score, using given parameter values instead of values previously configured
   *  @sa localizeRobotAtHomeBlocking()
   */
  AREXPORT bool   localizeRobotAtHomeBlocking(double distSpread,
					      double angleSpread,
					      double probThreshold)
  {
    return localizeRobotAtHomeBlocking(distSpread, distSpread, 
				       angleSpread, probThreshold);
  }

  /// Try initial localization at each home point in the map, set the robot pose to the point with best score, using given parameter values instead of values previously configured
  AREXPORT bool   localizeRobotAtHomeBlocking(double spreadX, double spreadY,
					      double angleSpread, double probThreshold);

  /**
   *  @brief Try initial localization at each home point at the map, and 
   *   set the robot pose to the point with best score
   *
   *   Attempt to localize the robot in the most
   *   likely home position: first it checks the robot's current pose
   *   and then checks all of the home positions in the map.  The position
   *   with the highest score is then used.
   *   Call this function after the localization thread has
   *   been initialized but needs to be reset, or an initial localization
   *   (i.e. at program start up) is needed.  This function blocks
   *   while the localization takes place, and returns after it either succeeds    
   *   (Either localization at a home position succeeds, or all fail.)
   *
   *   @note Localization will fail if no sensor data has yet been obtained   
   *         when this function is called (e.g. if called before or immediately    
   *         after connecting to the robot and/or laser sensor).
   */

  AREXPORT bool localizeRobotAtHomeBlocking() {
    return localizeRobotAtHomeBlocking(getStdX(), getStdY(), getStdTh(),
				       getUsingPassThreshold());
  }

  /// Request that the task later localize the robot at a home position,
  /// and return immediately
  AREXPORT bool   localizeRobotAtHomeNonBlocking(void);
  /// Gets the pose that robot was localized to
  AREXPORT virtual ArPose getRobotHome(void)
  {
    return myHomePose;
  }
  /// Sets the force update parameters.
  AREXPORT void   setForceUpdateParams(int numSamples, 
				       double xStd, double yStd, double tStd);
  /// Force an update in thread instead of waiting for distance-angle trigger.
  AREXPORT void   forceUpdatePose(ArPose forcePose);
  /// Adds a callback which will be called when loca fails.
  AREXPORT void   addFailedLocalizationCB(ArFunctor1<int>* functor);
  /// Removes a callback.
  AREXPORT void   remFailedLocalizationCB(ArFunctor1<int>* functor);
  /// Sets tracking failed Callback. 
  AREXPORT void   setFailedCallBack(ArFunctor1<int>* fcb) {myFailedCB=fcb;}

  /** @name Modifiers for configuration values.
   *  Normally thes evalues are set via ArConfig
   *  (e.g. from a loaded file or other source) in the "Localization"
   *  section, and these methods would only be used internally by ARNL.
   *  However, they can be used if you are not using ArConfig
   *  or wish to override a parameter.
   */
  //@{
  /// Sets the motion trigger value for distance.
  AREXPORT void   setTriggerDelR(double tr)
  {
    myMutex.lock();
    myTriggerDelR = tr;
    myMutex.unlock();
  }
  /// Sets the motion trigger value for angle.
  AREXPORT void   setTriggerDelT(double tt)
  {
    myMutex.lock();
    myTriggerDelT = tt;
    myMutex.unlock();
  }
  /// Sets the flag to trigger on idle.
  AREXPORT void   setTriggerTimeFlag(bool tt)
  {
    myMutex.lock();
    myTriggerTimeFlag = tt;
    myMutex.unlock();
  }
  /// Sets the idle trigger time in millisecs.
  AREXPORT void   setTriggerTime(double tt)
  {
    myMutex.lock();
    myTriggerDelT = tt;
    myMutex.unlock();
  }
  /// Sets the X range when localization triggers due to idle time.
  AREXPORT void   setTriggerTimeX(double tt)
  {
    myMutex.lock();
    myTriggerTimeX= tt;
    myMutex.unlock();
  }
  /// Sets the Y range when localization triggers due to idle time.
  AREXPORT void   setTriggerTimeY(double tt)
  {
    myMutex.lock();
    myTriggerTimeY= tt;
    myMutex.unlock();
  }
  /// Sets the Theta range when localization triggers due to idle time.
  AREXPORT void   setTriggerTimeTh(double tt)
  {
    myMutex.lock();
    myTriggerTimeTh= tt;
    myMutex.unlock();
  }
  /// Set the number of samples to use in the MCL.
  AREXPORT void   setNumSamples(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myNumSamples = n;
    myMutex.unlock();
  }
  /// Set the number of samples to use during initialization.
  AREXPORT void   setNumSamplesAtInit(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myNumSamplesAtInit = n;
    myMutex.unlock();
  }
  /// Set the variable number of samples (usually adjusted automatically).
  AREXPORT void   setCurrentNumSamples(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myCurrentNumSamples = n;
    myMutex.unlock();
  }
  ///  Sets the rotation factor for adjusting no of samples with score.
  AREXPORT void   setNumSamplesAngleFactor(double f)
  {
    myMutex.lock();
    myNumSamplesAngleFactor = f;
    myMutex.unlock();
  }

  /// Sets the pass threshold for fraction of sonar points matched to env.
  AREXPORT void   setPassThreshold(double f)
  {
    myMutex.lock();
    myPassThreshold = f;
    myMutex.unlock();
  }
  /// Sets the sensor belief.
  AREXPORT void   setSensorBelief(double sensorBelief)
  {
    if(!mySystemErrorPtr)
      return;

    myMutex.lock();
    mySystemErrorPtr->setSensorBelief(sensorBelief);
    myMutex.unlock();
  }
  /// Sets the current pose.
  AREXPORT void   setCurrentLocaPose(double x, double y, double th)
  {
    myMutex.lock();
    myCurrentLocaPose.setPose(x, y, th);
    myMutex.unlock();
  }
  /// Sets the current pose.
  AREXPORT void   setCurrentLocaPose(ArPose p)
  {
    myMutex.lock();
    myCurrentLocaPose = p;
    myMutex.unlock();
  }
  /// Sets the verbose flag value. (for debugging)
  AREXPORT void   setVerboseFlag(bool f)
  {
    myMutex.lock();
    myVerboseFlag = f;
    myMutex.unlock();
  }
  /// Sets the recoverOnFailed flag.
  AREXPORT void   setRecoverOnFailedFlag(bool f)
  {
    myMutex.lock();
    myRecoverOnFailFlag = f;
    myMutex.unlock();
  }
  /// Sets the idle flag.
  AREXPORT void   setIdleFlag(bool f)
  {
    myMutex.lock();
    myIdleFlag = f;
    if(f)
      myState = LOCALIZATION_IDLE;
    myMutex.unlock();
    if(!f)
      forceUpdatePose(myRobot->getPose());    
  }
  /// Sets the map reloading flag.
  AREXPORT void   setReloadingMapFlag(bool f)
  {
    myMutex.lock();
    myReloadingMapFlag = f;
    myMutex.unlock();
  }
  /// Sets the sonar range.
  AREXPORT void   setSonarMaxRange(double f)
  { 
    myMutex.lock();
    mySonarMaxRange = f > 0 ? f : 1000; 
    myMutex.unlock();
  }
  //@}

  /** @name Accessors for confiuration values.
   * These values are normally set via ArConfig (the "Localization"
   * section) by loading a file or other means, or by manually
   * calling the modifier functions above.
   */
  //@{
  /// Get the params to do the localization from the class.
  AREXPORT void   getForceUpdateParams(ArPose& forcePose, int& numSamples, 
				       double& xStd, double& yStd, 
				       double& tStd);
  /// Gets the verbose flag. (for debugging only)
  AREXPORT bool   getVerboseFlag(void)
  {
    myMutex.lock();
    bool ret = myVerboseFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the initialized flag indicating if localization thread is on. 
  AREXPORT bool   getInitializedFlag(void)
  {
    myMutex.lock();
    bool ret = myInitializedFlag;
    myMutex.unlock();
    return ret;
  }
  /// Get the maximum number of samples used in the MCL.
  AREXPORT int    getNumSamples(void)
  {
    myMutex.lock();
    int p = myNumSamples;
    myMutex.unlock();
    return p;
  }
  /// Get the number of samples used in the MCL during initialization.
  AREXPORT int    getNumSamplesAtInit(void)
  {
    myMutex.lock();
    int p = myNumSamplesAtInit;
    myMutex.unlock();
    return p;
  }
  /// Get the variable number of samples if adjusted during move.
  AREXPORT int    getCurrentNumSamples(void)
  {
    myMutex.lock();
    int p = myCurrentNumSamples;
    if(!myAdjustNumSamplesFlag)
      p = myNumSamples;
    myMutex.unlock();
    return p;
  }
  /// Get the current computed best robot pose.
  AREXPORT ArPose getRobotMaxProbPose(void);
  /// Gets the min distance to localize.
  AREXPORT double getTriggerDelR(void)
  {
    myMutex.lock();
    double p = myTriggerDelR;
    myMutex.unlock();
    return p;
  }
  /// Gets the min angle to localize.
  AREXPORT double getTriggerDelT(void)
  {
    myMutex.lock();
    double p = myTriggerDelT;
    myMutex.unlock();
    return p;
  }
  /// Gets the flag indicating if localization should trigger on idle.
  AREXPORT bool   getTriggerTimeFlag(void)
  {
    myMutex.lock();
    bool p = myTriggerTimeFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the min time in millisecs to be idle.
  AREXPORT double getTriggerTime(void)
  {
    myMutex.lock();
    double p = myTriggerTime;
    myMutex.unlock();
    return p;
  }
  /// Gets the X range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeX(void)
  {
    myMutex.lock();
    double p = myTriggerTimeX;
    myMutex.unlock();
    return p;
  }
  /// Gets the Y range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeY(void)
  {
    myMutex.lock();
    double p = myTriggerTimeY;
    myMutex.unlock();
    return p;
  }
  /// Gets the Theta range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeTh(void)
  {
    myMutex.lock();
    double p = myTriggerTimeTh;
    myMutex.unlock();
    return p;
  }
  /// Gets the Pass threshold for localization success.
  AREXPORT double getPassThreshold(void)
  {
    myMutex.lock();
    double p = myPassThreshold;
    myMutex.unlock();
    return p;
  }
  /// Gets the Pass threshold for localization success being used right now
  AREXPORT double getUsingPassThreshold(void)
  {
    myMutex.lock();
    double p;
    if (myUseTempPassThreshold)
      p = myTempPassThreshold;
    else
      p = myPassThreshold;
    myMutex.unlock();
    return p;
  }
  /// Sets the Pass threshold to use until it is reset to config 
  AREXPORT void setTempPassThreshold(double passThreshold)
  {
    myMutex.lock();
    myUseTempPassThreshold = true;
    myTempPassThreshold = passThreshold;
    myMutex.unlock();
  }
  /// Gets the temporary pass threshold to use until it is reset to config 
  AREXPORT double getTempPassThreshold(void)
  {
    myMutex.lock();
    double p;
    if (myUseTempPassThreshold)
      p = myTempPassThreshold;
    else
      p = -1;
    myMutex.unlock();
    return p;
  }
  /// Resets the Pass threshold to config
  AREXPORT void clearTempPassThreshold(void)
  {
    myMutex.lock();
    myUseTempPassThreshold = false;
    myMutex.unlock();
  }

  /// Gets the score for the localization as fraction of matched sonar pts.
  AREXPORT virtual double getLocalizationScore(void);
  /// Gets the sensor belief.
  AREXPORT double getSensorBelief(void)
  {
    if(!mySystemErrorPtr)
      return -1;

    myMutex.lock();
    double ret = mySystemErrorPtr->getSensorBelief();
    myMutex.unlock();

    return ret;
  }
  /// Gets the last successful MCLocalized pose.
  AREXPORT ArPose getCurrentLocaPose(void)
  {

    myMutex.lock();
    ArPose p = myCurrentLocaPose;
    myMutex.unlock();
    return p;
  }
  // Gets the standard deviation of the initialization X coords.
  AREXPORT double getStdX(void) const { return myInitStdX; }
  // Gets the standard deviation of the initialization Y coords.
  AREXPORT double getStdY(void) const { return myInitStdY; }
  // Gets the standard deviation of the initialization Th coords.
  AREXPORT double getStdTh(void) const { return myInitStdTh; }
  // Gets the motion error mm per mm error coefficient.
  AREXPORT double getErrorMmPerMm(void) const { return myErrorMmPerMm; }
  // Gets the motion error deg per deg error coefficient.
  AREXPORT double getErrorDegPerDeg(void) const { return myErrorDegPerDeg; }
  // Gets the motion error deg per mm error coefficient.
  AREXPORT double getErrorDegPerMm(void) const { return myErrorDegPerMm; }
  // Gets the peak factor for multiple hypothesis.
  AREXPORT double getPeakFactor(void) const { return myPeakFactor; }
  // Gets the map name.
  AREXPORT char*  getMapName(void) { return myMapName; }
  // Gets the peturbation range in X coords in mm.
  AREXPORT double getPeturbRangeX(void) const { return myPeturbX; }
  // Gets the peturbation range in Y coords in mm.
  AREXPORT double getPeturbRangeY(void) const { return myPeturbY; }
  // Gets the peturbation range in Theta coords in degs.
  AREXPORT double getPeturbRangeTh(void) const { return myPeturbTh; }
  // Gets the failed range in X coords in mm.
  AREXPORT double getFailedRangeX(void) const { return myFailedX; }
  // Gets the failed range in Y coords in mm.
  AREXPORT double getFailedRangeY(void) const { return myFailedY; }
  // Gets the failed range in Theta coords in degs.
  AREXPORT double getFailedRangeTh(void) const { return myFailedTh; }
  // Gets the peak pose X standard deviation in mm.
  AREXPORT double getPeakStdX(void) const { return myPeakStdX; }
  // Gets the peak pose Y standard deviation in mm.
  AREXPORT double getPeakStdY(void) const { return myPeakStdY; }
  // Gets the peak pose Th standard deviation in degs
  AREXPORT double getPeakStdTh(void) const { return myPeakStdTh; }
  // Gets the Aria map.
  AREXPORT ArMapInterface* getAriaMap(void) { return myAriaMapPtr; }
  /// Scan Buffer size. (internal sonar scan buffer)
  AREXPORT int    getBufferSize(void) {return myXYBuffer.size();}
  /// Scan Buffer XY.
  AREXPORT std::vector<ArPose> getXYBuffer(void) {return myXYBuffer;}
  /// Scan Buffer pose taken. (internal sonar scan buffer)
  AREXPORT ArPose getBufferPose(void) {return myBufferPose;}
  /// Gets the pose samples for client.
  AREXPORT std::list<ArPose> getCurrentSamplePoses(void);
  /// Gets the recoverOnFailed flag.
  AREXPORT bool   getRecoverOnFailedFlag(void)
  {
    myMutex.lock();
    bool ret = myRecoverOnFailFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the angle factor when rotating which multiplies the no of samples.
  AREXPORT double getNumSamplesAngleFactor(void)
  {
    myMutex.lock();
    double ret = myNumSamplesAngleFactor;
    myMutex.unlock();
    return ret;
  }
  /// Gets the sonar range.
  AREXPORT double getSonarMaxRange(void)
  {
    myMutex.lock();
    double ret = mySonarMaxRange;
    myMutex.unlock();
    return ret;
  }
  /// Gets the min line size;
  AREXPORT double getSonarMinLineSize(void)
  {
    myMutex.lock();
    double ret = mySonarMinLineSize;
    myMutex.unlock();
    return ret;
  }
  /// Gets the flag which allows for changing numSamples with loca score.
  AREXPORT bool   getAdjustNumSamplesFlag(void)
  {
    myMutex.lock();
    bool ret = myAdjustNumSamplesFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the min no of samples if they can be adjusted.
  AREXPORT int    getMinNumSamples(void)
  {
    myMutex.lock();
    int ret = myMinNumSamples;
    myMutex.unlock();
    return ret;
  }
  //@}
  
  /// Gets the state of localization.
  AREXPORT virtual LocalizationState getState(void)
  {
    myMutex.lock();
    ArSonarLocalizationTask::LocalizationState p = myState;
    myMutex.unlock();
    return p;
  }
  /// Gets the idle flag.
  AREXPORT bool   getIdleFlag(void)
  {
    myMutex.lock();
    bool ret = myIdleFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the map reloading flag.
  AREXPORT bool   getReloadingMapFlag(void)
  {
    myMutex.lock();
    bool ret = myReloadingMapFlag;
    myMutex.unlock();
    return ret;
  }
  
  /// Read the Map data from a map file.
  AREXPORT ArMapInterface* readMapFromFile(char* mapName);
  /// Read the Map data from an ArMap object
  AREXPORT ArMapInterface* readAriaMap(ArMapInterface* ariaMap);

  /// load param file when Aria config sets it off.
  AREXPORT bool   loadParamFile(const char *file);
  /// Saves all default params to the param file.
  AREXPORT bool   saveParams(char* filename);

  /** @name Functions used internally by ARNL
   *  @internal
   */
  //@{
  /// Fill prob distribution histogram for debugging.
  AREXPORT bool   fillHistogram(double*& hist, double*& cumSum, 
				int& numSamples);
  /// Convert the sonar data to x and y coord array.
  AREXPORT bool   scanToGlobalCoords(ArPose robPose, 
				     std::vector<ArPose>& xyLrf);
  /// Set all the parameters for localization.
  AREXPORT bool   setLocaParams(double xStd, double yStd, double tStd,
				double kMmPerMm, double kDegPerDeg, 
				double kDegPerMm,
				double sensorBelief);
  /// Sets the flag which changes numSamples with the localization score.
  AREXPORT void   setAdjustNumSamplesFlag(bool f)
  {
    myMutex.lock();
    myAdjustNumSamplesFlag = f;
    myMutex.unlock();
  }
  /// Sets the min no of samples if they are adjustable with loca score.
  AREXPORT void   setMinNumSamples(int f)
  {
    myMutex.lock();
    myMinNumSamples = f;
    myMutex.unlock();
  }
  /// Sets the last loca time to now.
  AREXPORT void   setLastLocaTimeToNow(void)
  {
    myMutex.lock();
    myLastLocaTime.setToNow();
    myMutex.unlock();
  }
  /// Gets the last loca time to now.
  AREXPORT ArTime getLastLocaTime(void)
  {
    myMutex.lock();
    ArTime tim = myLastLocaTime;
    myMutex.unlock();
    return tim;
  }
  /// Finds the mean and var of MCL after moving to present pose.
  AREXPORT bool   findMCLMeanVar(ArPose& mean, ArMatrix& Var);
  /// The actual mean var calculator for the virtual in the base class.
  AREXPORT virtual bool findLocalizationMeanVar(ArPose& mean, ArMatrix& Var);
  /// Sets the flag deciding whether to reflect localized pose onto the robot.
  AREXPORT virtual void setCorrectRobotFlag(bool f)
  {
    myMutex.lock();
    myCorrectRobotFlag = f;
    myMutex.unlock();
  }
  /// Used to set the robot pose usually at the start of localization. 
  /// This one with a spread around the set pose.
  AREXPORT virtual void setRobotPose(ArPose pose,
				     ArPose spread = ArPose(0, 0, 0), 
				     int nSam = 0);
  /// Gets the lost flag.
  AREXPORT virtual bool getRobotIsLostFlag(void) 
  {return !getInitializedFlag();}
  /// Localize the robot at known homes.
  AREXPORT virtual bool localizeRobotAtHomeBlocking(double distSpread, 
						    double angleSpread)
  {return localizeRobotAtHomeBlocking(distSpread, distSpread, angleSpread, 
				      getUsingPassThreshold());}
  /// Set the localization idle
  AREXPORT virtual void setLocalizationIdle(bool f)
  {
    setIdleFlag(f);
    return;
  }
  //@}
 
  /** @name ArNetworking callback methods
   * (used by server classes in ArServerClasses.cpp)
   */
  //@{
  /// Draws range data used for localization.
  AREXPORT void    drawRangePoints(ArServerClient* client, 
				   ArNetPacket* packet);
  /// Draws the closest map lines to robot.
  AREXPORT void    drawClosestLines(ArServerClient* client, 
				    ArNetPacket* packet);
  /// Draws the sonar rays.
  AREXPORT void    drawBestRays(ArServerClient* client, 
				ArNetPacket* packet);
  /// Draws the sonar rays.
  AREXPORT void    drawSampleRays(ArServerClient* client, 
				  ArNetPacket* packet);
  /// Draws the sonar rays.
  AREXPORT void    drawSonarRays(ArServerClient* client, 
				 ArNetPacket* packet);
  /// Draws intersection to closest lines.
  AREXPORT void    drawIntersections(ArServerClient* client, 
				     ArNetPacket* packet);
  /// Draws the histogram.
  AREXPORT void    drawHistogram(ArServerClient* client, ArNetPacket* packet);
  /// Draws the histogram.
  AREXPORT void    drawBestPoses(ArServerClient* client, ArNetPacket* packet);
  /// Draws the samples.
  AREXPORT void    drawSamplePoses(ArServerClient* client,
				   ArNetPacket* packet);
  /// Draws the sample bounds.
  AREXPORT void   drawMCLVariance(ArServerClient* client, 
				  ArNetPacket* packet);
  //@}

protected:
  std::vector<ArPose>  makeMeanVarPoints(ArPose& mean, ArMatrix& Var);
  // Basic initializer (internal use only)
  void          basicInitialization(ArRobot* robot, ArSonarDevice* sonar);
  // Common segment to call after fail.
  void          setFlagsAndCallbacksOnFail(int ntimes);
  // Function used to run the task as a thread.
  virtual void* runThread(void* ptr);
  // Function which does the main eqn for the actual MCL localization.
  bool          correctPoseFromSensor(int sonarStart, int sonarEnd,
				      double maxRange,
				      std::vector<ArLineSegment> lines);
  // Gets the localized flag. (do not use)
  bool          getLocalizedFlag(void)
  {
    myMutex.lock();
    bool ret = myLocalizedFlag;
    myMutex.unlock();
    return ret;
  }
  // Gets the localizing flag value. To avoid multiple localizing threads.
  bool          getLocalizingFlag(void)
  {
    myMutex.lock();
    bool ret = myLocalizingFlag;
    myMutex.unlock();
    return ret;
  }
  // Gets the robot moved flag value. (Moved after last localization)
  bool          getRobotMovedFlag(void)
  {
    myMutex.lock();
    bool ret = myRobotMovedFlag;
    myMutex.unlock();
    return ret;
  }
  // Gets the flag indicating if localization result will be set on robot.
  bool          getCorrectRobotFlag(void)
  {
    myMutex.lock();
    bool ret = myCorrectRobotFlag;
    myMutex.unlock();
    return ret;
  }
  // Sets the localized flag value. (do not use)
  void          setLocalizedFlag(bool f)
  {
    myMutex.lock();
    myLocalizedFlag = f;
    myMutex.unlock();
  }
  // Sets the initialized flag value indicating robot was localized or not.
  void          setInitializedFlag(bool f)
  {
    myMutex.lock();
    myInitializedFlag = f;
    myMutex.unlock();
  }
  // Sets the robot moved flag to set off MCL.
  void          setRobotMovedFlag(bool f)
  {
    myMutex.lock();
    myRobotMovedFlag = f;
    myMutex.unlock();
  }
  // Initialize the MCL samples.
  bool          initializeSamples(int numSamples);
  // Legalize samples to eliminate illegal location such as on obstacles.
  void          killBadSamples(double obsThreshold);
  // Save samples for backup 
  void          saveSamples(bool saveFile = false);
  // Find the best poses by looking at prob distribution peaks.
  int           findBestPoses(ArRobotPoseSamples* mrsp, double factor);
  // Get the number of peaks.
  int           getNumBestPoses(ArRobotPoseSamples* mrsp)
  {
    return myBestPoses.size();
  }
  // Compute statistics of samples after localization.
  bool          findAllStatistics(double& xMean,double& yMean, double& thMean,
				  double& xStd,double& yStd, double& tStd);
  // Return time of last good localization.
  unsigned int  getLocaTime() {return myLocaTime;}
  // Set motion error params.
  void          setMotionErrorParam(int index, double value);
  // Get motion error params.
  double        getMotionErrorParam(int index);
  // The sensor interpretation callback. Called every 100msec.
  void          robotCallBack(void);
  // Needed if the sonar does not connect first.
  bool          configureSonar(void);
  // Needed if the params are changed or loaded again.
  bool          reconfigureLocalization(void);
  // Setup the path planing params with the aria config thing.
  void          setupLocalizationParams(void);
  // Actual thing that gets called when localization failed.
  void          failedLocalization(int times);
  // Change the sample size to reflect localization score.
  int           dynamicallyAdjustNumSamples(double dr, double dt);
  // Function for the things to do if map changes.
  void          mapChanged(void);
  // Find the closest lines to a robot pose.
  std::vector<ArLineSegment> findClosestLines(ArPose robotPose, double range);
  // Gets the closest lines.
  AREXPORT std::vector<ArLineSegment> getClosestLines(void);

  private:
  bool     myLocalizingFlag;          // Localizing going on.
  bool     myInitializedFlag;         // Robot pose is initialized.
  bool     myRobotMovedFlag;          // Robot Moved Flag.
  bool     myLocalizedFlag;           // Robot Localized Flag.
  bool     myVerboseFlag;             // Print details flag.
  bool     myCorrectRobotFlag;        // Modify pose on robot flag.
  bool     myOwnArMapFlag;            // Does the ArMap belong to us?
  int      myNumSamples;              // Number of samples to use in MCL.
  int      myNumSamplesAtInit;        // Number of samples at initialization.
  ArPose   myHomePose;                // where we started off localizing too
  bool     myAdjustNumSamplesFlag;    // Flag to change numSamples with score.

  LocalizationState myState;          // Current state

  double   myTriggerDelR;             // Min distance to trigger Loc.
  double   myTriggerDelT;             // Min angle to trigger Loc.
  bool     myTriggerTimeFlag;         // Trigger on idle time?
  double   myTriggerTime;             // Min idle time in msecs to trigger loc.
  double   myTriggerTimeX;            // Range for X at idle time localization.
  double   myTriggerTimeY;            // Range for Y at idle time localization.
  double   myTriggerTimeTh;           // Range for Theta ...
  double   myPassThreshold;           // Theshold to pass localization.
  double   myUseTempPassThreshold;    // Whether to use the temporary passThres
  double   myTempPassThreshold;       // The temporary passThres

  bool     myForceUpdateFlag;         // Flag to force an update in thread.
  int      myForceUpdateNumSamples;   // No of samples to force update with.
  ArPose   myForceUpdatePose;         // Pose to force update with.
  double   myForceUpdateXStd;         // X Std deviation.
  double   myForceUpdateYStd;         // Y Std deviation.
  double   myForceUpdateTStd;         // Theta Std deviation.

  std::vector<ArPose> myXYBuffer;     // Laser XY buffer.
  ArPose   myBufferPose;              // Pose at which buffer was filled.
  
  ArPose   myCurrentLocaPose;         // Current robot pose.
  ArPose   myTrueCurrentPose;         // True Current robot pose.
  ArPose   myTrueEncoderPose;         // True Current robot pose. Enc.
  ArPose   myLastLocaPose;            // Last localized pose.
  ArPose   myLastLocaEncoderPose;     // Last localized pose.Enc
  ArTime   myLastLocaTime;            // Last localization time.
  ArPose   myLastDelPose;             // Last delpose after localization.
  ArPose   myLastLocaMean;            // Kept for later.
  ArMatrix myLastLocaVar;             // Kept for later.
  std::vector<ArRobotPoseProb*> myBestPoses; // Heap of probable poses.
  unsigned int  myLocaTime;           // Time at which good localization done.
  int      myCurrentNumSamples;       // Variable sample size for the cycle.
  int      myMinNumSamples;           // Min no of samples when adjustable.
  double   myNumSamplesAngleFactor;   // Special factor when rotated.

  ArRobot* myRobot;
  ArSick*  mySick;
  ArSonarDevice* mySonar;

  ArMutex myMutex;

  ArRobotPoseSamples* myRobotSamplesPtr; // MCL samples data struct.
  ArRobotPoseSamples* myPreResamplesPtr; // Before resampling.
  ArRobotAndSonar*    myRobotAndSonarPtr;// Robot and Sonar related stuff.
  ArSystemError*      mySystemErrorPtr;  // Error parameter holder.
  ArMapInterface*     myAriaMapPtr;      // Map data.
  std::vector<ArLineSegment> myLines;

  ArFunctorC<ArSonarLocalizationTask>*     myRobotCB;  // Robot->getPose()
  ArFunctor1<int>*                         myFailedCB; // Failed callback.

  ArRetFunctorC<bool, ArSonarLocalizationTask>* mySonarConnectedCB;

  ArRetFunctorC<bool, ArSonarLocalizationTask>* myProcessFileCB; // Config.
  ArFunctorC<ArSonarLocalizationTask>*          myMapChangedCB;

  ArConfig*                                myParams; // Default params holder.
  std::list<ArFunctor1<int>*>              myFailedLocalizationCBList;
  std::vector<ArLineSegment>               myClosestLines;

  bool                                     myDisplayMCLRet;
  ArPose                                   myDisplayMCLMean;
  ArMatrix                                 myDisplayMCLVar;

  bool                                     myDisplayRefRet;
  ArPose                                   myDisplayRefMean;
  ArMatrix                                 myDisplayRefVar;

  protected:
  double myInitStdX;
  double myInitStdY;
  double myInitStdTh;
  double myErrorMmPerMm;
  double myErrorDegPerDeg;
  double myErrorDegPerMm;
  double myPeakFactor;
  char   myMapName[1024];
  double myPeturbX;
  double myPeturbY;
  double myPeturbTh;
  double myFailedX;
  double myFailedY;
  double myFailedTh;
  double myPeakStdX;
  double myPeakStdY;
  double myPeakStdTh;
  bool   myRecoverOnFailFlag;
  bool   myIdleFlag;
  bool   myReloadingMapFlag;
  double mySonarMaxRange;
  double mySonarAperture;
  double mySonarRangeRes;
  double mySonarAngleRes;
  double mySonarIncidenceLimit;
  double mySonarLambdaF;
  double mySonarLambdaR;
  double mySonarSigma;
  double mySonarBetaMin;
  double mySonarBetaMax;
  double mySonarMinLineSize;

};

#endif // ARSONARLOCALIZATIONTASK_H
