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
#ifdef _GPP_OMIT_
          /* combinedServer.cpp 
           *
           * Combined example, used as template to generate the other examples.
           *
           * This program can use one or more localization libraries.  It
           * selects the localizations to use using preprocessor definitions:
           * ARNL, SONARNL and MOGS.  The Makefile defines these based on which
           * localization libraries are available.
           *
           * Because the localization libraries have different hardware
           * requirements, and behave slightly differently, this file is
           * full of preprocessor conditionals.   For a more straightforward
           * example that uses just one of the localization libraries, see one
           * of: arnlServer.cpp, sonarnlServer.cpp, mogsServer.cpp.
           *
           * This program can be used as the input to the "gpp" preprocessor to
           * generate arnlServer.cpp, sonarnlServer.cpp and mogsServer.cpp,
           * but it is also by itself a usable program, which uses MCL if ARNL
           * is defined, sonar localization if SONARNL is defined but ARNL is not defined,
           * and GPS localization if either is defined and MOGS is enabled.
           *
           */
#endif
#if defined(ARNL)
/** @example arnlServer.cpp example of almost all ARNL features */
#elif defined(SONARNL)
/** @example sonarnlServer.cpp example of almost all SONARNL features */
#elif defined(MOGS)
/** @example mogsServer.cpp example of almost all MOGS features */
#else
#error Must have ARNL, SONARNL or MOGS defined.
#endif

#if defined(ARNL)
#ifndef WIN32
#warning Arnl selected. Enabling laser localization, laser, docking, multirobot, mapping.  Will not use SonArnl.
#endif
#define ARNL_LASERLOC
#define ARNL_DOCKING
#define ARNL_MULTIROBOT
#define ARNL_MAPPING
#define ARNL_LASER
#elif defined(SONARNL)
#ifndef WIN32
#warning Sonarnl selected.  Enabling sonar localization.
#endif
#define ARNL_SONARLOC
#endif

#ifdef MOGS
#ifndef WIN32
#warning Mogs selected. Enabling GPS localization, plus laser use and mapping.
#endif
#define ARNL_GPSLOC
#define ARNL_MAPPING
#define ARNL_LASER
#if defined(ARNL) || defined(SONARNL)
#define ARNL_MULTILOC
#endif
#endif

#if defined(MOGS) || defined(ARNL)
#define ARNL_MAPPING
#endif

#if defined(ARNL_MULTILOC)
#ifndef WIN32
#warning More than one localization method selected, will use localization manager.
#endif
#endif

#if !defined(MOGS)  && !defined(ARNL) && !defined(SONARNL) 
#error Must have at least one of MOGS, ARNL, SONARNL defined!
#endif
/**
  This is an example of using ARNL and ArNetworking to provide a
  server program that remote clients, such as MobileEyes, can connect
  to, and which includes most of the features available in ARNL.

  You can make a copy of this program's source file in the 'examples' directory
  and use it as the basis for your own program.

  MobileEyes sends requests to this server program to go to goals,
  get the current map, set configuration parameters, manually relocalize, 
  request the current state of the robot and sensors, etc. This example server
  includes almost all features provided by ARNL (or SONARNL or MOGS), including
  localization, path planning and navigation to a map goal or abritrary point,
  sharing information with other servers peer-to-peer or through 
  a "central server", using IR and bumper sensors, stopping the robot
  if localization fails, global replanning if a path is not followable,
  use of special SICK laser reflector beacons, as well as various networking
  services for MobileEyes and other clients, such as diagnostic visualizations,
  access to configuration parameters, access to camera control and 
  video images (if SAVServer is running), special debugging commands 
  ("custom commands"), file uploading/downloading and gathering raw laser
  scans using Mapper3, and safe and unsafe teleoperation.
*/

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"

#ifdef ARNL_LASERLOC
#include "ArLocalizationTask.h"
#endif
#ifdef ARNL_DOCKING
#include "ArDocking.h"
#endif
#ifdef ARNL_SONARLOC
#include "ArSonarLocalizationTask.h"
#endif
#ifdef ARNL_GPSLOC
#include "ArGPSLocalizationTask.h"
#endif

void logOptions(const char *progname)
{
  ArLog::log(ArLog::Normal, "Usage: %s [options]\n", progname);
  ArLog::log(ArLog::Normal, "[options] are any program options listed below, or any ARNL configuration");
  ArLog::log(ArLog::Normal, "parameters as -name <value>, see params/arnl.p for list.");
  ArLog::log(ArLog::Normal, "For example, -map <map file>.");
  Aria::logOptions();
}

bool gyroErrored = false;
const char* getGyroStatusString(ArRobot* robot)
{
  if(!robot || !robot->getOrigRobotConfig() || robot->getOrigRobotConfig()->getGyroType() < 2) return "N/A";
  if(robot->getFaultFlags() & ArUtil::BIT4)
  {
    gyroErrored = true;
    return "ERROR/OFF";
  }
  if(gyroErrored)
  {
    return "OK but error before";
  }
  return "OK";
}

int main(int argc, char **argv)
{
  // Initialize Aria and Arnl global information
  Aria::init();
  Arnl::init();


  // The robot object
  ArRobot robot;

  // Parse the command line arguments.
  ArArgumentParser parser(&argc, argv);

  // Set up our simpleConnector, to connect to the robot and laser
  //ArSimpleConnector simpleConnector(&parser);
  ArRobotConnector robotConnector(&parser, &robot);

  // Connect to the robot
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to robot... exiting");
    Aria::exit(3);
  }



  // Set up where we'll look for files. Arnl::init() set Aria's default
  // directory to Arnl's default directory; addDirectories() appends this
  // "examples" directory.
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  
  
  // To direct log messages to a file, or to change the log level, use these  calls:
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
  //ArLog::init(ArLog::File, ArLog::Verbose);
 
  // Add a section to the configuration to change ArLog parameters
  ArLog::addToConfig(Aria::getConfig());

  // set up a gyro (if the robot is older and its firmware does not
  // automatically incorporate gyro corrections, then this object will do it)
  ArAnalogGyro gyro(&robot);

  // Our networking server
  ArServerBase server;
  
#ifdef ARNL_GPSLOC
  // GPS connector.
  ArGPSConnector gpsConnector(&parser);
#endif

  // Set up our simpleOpener, used to set up the networking server
  ArServerSimpleOpener simpleOpener(&parser);

#ifdef ARNL_LASER
  // the laser connector
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  // Tell the laser connector to always connect the first laser since
  // this program always requires a laser.
  parser.addDefaultArgument("-connectLaser");
#endif
  
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // Parse arguments 
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(1);
  }
  

  // This causes Aria::exit(9) to be called if the robot unexpectedly
  // disconnects
  ArGlobalFunctor1<int> shutdownFunctor(&Aria::exit, 9);
  robot.addDisconnectOnErrorCB(&shutdownFunctor);


  // Create an ArSonarDevice object (ArRangeDevice subclass) and 
  // connect it to the robot.
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);



  // This object will allow robot's movement parameters to be changed through
  // a Robot Configuration section in the ArConfig global configuration facility.
  ArRobotConfig robotConfig(&robot);

  // Include gyro configuration options in the robot configuration section.
  robotConfig.addAnalogGyro(&gyro);

  // Start the robot thread.
  robot.runAsync(true);
  
#ifdef ARNL_LASER

  // connect the laser(s) if it was requested, this adds them to the
  // robot too, and starts them running in their own threads
  if (!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Could not connect to all lasers... exiting\n");
    Aria::exit(2);
  }
#endif

#if defined(ARNL_LASERLOC) || defined(ARNL_MAPPING)
  // find the laser we should use for localization and/or mapping,
  // which will be the first laser
  robot.lock();
  ArLaser *firstLaser = robot.findLaser(1);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ArLog::log(ArLog::Normal, "Did not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting");
    Aria::exit(2);
  }
  robot.unlock();
#endif  


    /* Create and set up map object */
  
  // Set up the map object, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the program's current directory
  // instead.
  // When a configuration file is loaded into ArConfig later, if it specifies a
  // map file, then that file will be loaded as the map.
  ArMap map(fileDir);
  // set it up to ignore empty file names (otherwise if a configuration omits
  // the map file, the whole configuration change will fail)
  map.setIgnoreEmptyFileName(true);
  // ignore the case, so that if someone is using MobileEyes or
  // MobilePlanner from Windows and changes the case on a map name,
  // it will still work.
  map.setIgnoreCase(true);

    
    /* Create localization and path planning threads */

#ifdef ARNL_MULTILOC
  ArLocalizationManager locManager(&robot, &map);
#define LOCTASK locManager
#endif

  ArPathPlanningTask pathTask(&robot, &sonarDev, &map);

#ifdef ARNL_LASERLOC
  ArLog::log(ArLog::Normal, "Creating laser localization task");
  // Laser Monte-Carlo Localization
  ArLocalizationTask locTask(&robot, firstLaser, &map);
#ifdef ARNL_MULTILOC
  locManager.addLocalizationTask(&locTask);
#else
#define LOCTASK locTask
#endif
#endif

#ifdef ARNL_SONARLOC
  ArLog::log(ArLog::Normal, "Creating sonar localization task");
  ArSonarLocalizationTask locTask(&robot, &sonarDev, &map);
#ifdef ARNL_MULTILOC
  locManager.addLocalizationTask(&locTask);
#else
#define LOCTASK locTask
#endif
#endif

#ifdef ARNL_GPSLOC
  ArLog::log(ArLog::Normal, "Connecting to GPS...");

  // On the Seekur, power to the GPS receiver is switched on by this command.
  // (A third argument of 0 would turn it off). On other robots this command is
  // ignored.
  robot.com2Bytes(116, 6, 1);

  // Connect to GPS
  ArGPS *gps = gpsConnector.createGPS(&robot);
  if(!gps || !gps->connect())
  {
    ArLog::log(ArLog::Terse, "Error connecting to GPS device."
      "Try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments."
      "Use -help for help. Exiting.");
    Aria::exit(5);
  }

  // set up GPS localization task
  ArLog::log(ArLog::Normal, "Creating GPS localization task");
  ArGPSLocalizationTask gpsLocTask(&robot, gps, &map);
#ifdef ARNL_MULTILOC
  locManager.addLocalizationTask(&gpsLocTask);
#else
#define LOCTASK gpsLocTask
#endif
#endif

#ifdef ARNL_LASER
  // Set some options on each laser that the laser connector 
  // connected to.
  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot.getLaserMap()->begin();
       laserIt != robot.getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    // Skip lasers that aren't connected
    if(!laser->isConnected())
      continue;

    // add the disconnectOnError CB to shut things down if the laser
    // connection is lost
    laser->addDisconnectOnErrorCB(&shutdownFunctor);
    // set the number of cumulative readings the laser will take
    laser->setCumulativeBufferSize(200);
    // add the lasers to the path planning task
    pathTask.addRangeDevice(laser, ArPathPlanningTask::BOTH);
    // set the cumulative clean offset (so that they don't all fire at once)
    laser->setCumulativeCleanOffset(laserNum * 100);
    // reset the cumulative clean time (to make the new offset take effect)
    laser->resetLastCumulativeCleanTime();

    // Add the packet count to the Aria info strings (It will be included in
    // MobileEyes custom details so you can monitor whether the laser data is
    // being received correctly)
    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10, 
	    new ArRetFunctorC<int, ArLaser>(laser, 
					 &ArLaser::getReadingCount));
  }
#endif


#ifdef ARNL_MULTIROBOT
  // Used for optional multirobot features (see below) (TODO move to multirobot
  // example?)
  ArClientSwitchManager clientSwitch(&server, &parser);
#endif




    /* Start the server */

  // Open the networking server
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    ArLog::log(ArLog::Normal, "Error: Could not open server.");
    exit(2);
  }



    /* Create various services that provide network access to clients (such as
     * MobileEyes), as well as add various additional features to ARNL */


#ifdef ARNL_MULTIROBOT
  // ARNL can optionally get information about the positions of other robots from a
  // "central server" (see central server example program), if command
  // line options specifying the address of the central server was given.
  // If there is no central server, then the address of each other robot
  // can instead be given in the configuration, and the multirobot systems
  // will connect to each robot (or "peer") individually.

        // TODO move this to multirobot example?
  ArServerHandlerMultiRobot *handlerMultiRobot = NULL;
  ArMultiRobotRangeDevice *multiRobotRangeDevice = NULL;
  ArServerHandlerMultiRobotPeer *multiRobotPeer = NULL;
  ArMultiRobotPeerRangeDevice *multiRobotPeerRangeDevice = NULL;


  bool usingCentralServer = false;
  if(clientSwitch.getCentralServerHostName() != NULL)
    usingCentralServer = true;

  // if we're using the central server then we want to create the
  // multiRobot central classes
  if (usingCentralServer)
  {
    // Make the handler for multi robot information (this sends the
    // information to the central server)
    handlerMultiRobot = new ArServerHandlerMultiRobot(&server, &robot, 
						      &pathTask,
						      &locTask, &map);
    
    // Normally each robot, and the central server, must all have
    // the same map name for the central server to share robot
    // information.  (i.e. they are operating in the same space).
    // This changes the map name that ArServerHandlerMutliRobot 
    // reports to the central server, in case you want this individual
    // robot to load a different map file name, but still report 
    // the common map file to the central server.
    //handlerMultiRobot->overrideMapName("central.map");

    // the range device that gets the multi robot information from
    // the central server and presents it as virtual range readings
    // to ARNL
    multiRobotRangeDevice = new ArMultiRobotRangeDevice(&server);
    
    robot.addRangeDevice(multiRobotRangeDevice);
    pathTask.addRangeDevice(multiRobotRangeDevice, 
			    ArPathPlanningTask::BOTH);
    
    // Set up options for drawing multirobot information in MobileEyes.
    multiRobotRangeDevice->setCurrentDrawingData(
	    new ArDrawingData("polyDots", ArColor(125, 125, 0),
			      100, 73, 1000), true);
    multiRobotRangeDevice->setCumulativeDrawingData(
	    new ArDrawingData("polyDots", ArColor(125, 0, 125),
			      100, 72, 1000), true);

    // This sets up the localization to use the known poses of other robots
    // for its localization in cases where numerous robots crowd out the map.
    locTask.setMultiRobotCallback(multiRobotRangeDevice->getOtherRobotsCB());
  }
  // if we're not using a central server then create the multirobot peer classes
  else
  {
    // set the path planning so it uses the explicit collision range for how far its planning
    pathTask.setUseCollisionRangeForPlanningFlag(true);
    // make our thing that gathers information from the other servers
    multiRobotPeerRangeDevice = new ArMultiRobotPeerRangeDevice(&map);
    // make our thing that sends information to the other servers
    multiRobotPeer = new ArServerHandlerMultiRobotPeer(&server, &robot, 
						     &pathTask, &locTask);
    // hook the two together so they both know what priority this robot is
    multiRobotPeer->setNewPrecedenceCallback(
	    multiRobotPeerRangeDevice->getSetPrecedenceCallback());
    // hook the two together so they both know what priority this
    // robot's fingerprint is
    multiRobotPeer->setNewFingerprintCallback(
	    multiRobotPeerRangeDevice->getSetFingerprintCallback());
    // hook the two together so that the range device can call on the
    // server handler to change its fingerprint
    multiRobotPeerRangeDevice->setChangeFingerprintCB(
	    multiRobotPeer->getChangeFingerprintCB());
    // then add the robot to the places it needs to be
    robot.addRangeDevice(multiRobotPeerRangeDevice);
    pathTask.addRangeDevice(multiRobotPeerRangeDevice, 
			    ArPathPlanningTask::BOTH);

    // Set the range device so that we can see the information its using
    // to avoid, you can comment these out in order to not see them
    multiRobotPeerRangeDevice->setCurrentDrawingData(
	    new ArDrawingData("polyDots", ArColor(125, 125, 0),
			      100, 72, 1000), true);
    multiRobotPeerRangeDevice->setCumulativeDrawingData(
	    new ArDrawingData("polyDots", ArColor(125, 0, 125),
			      100, 72, 1000), true);
    // This sets up the localization to use the known poses of other robots
    // for its localization in cases where numerous robots crowd out the map.
    locTask.setMultiRobotCallback(
	    multiRobotPeerRangeDevice->getOtherRobotsCB());
  }

#endif



  /* Add additional range devices to the robot and path planning task (so it
     avoids obstacles detected by these devices) */
  
  // Add IR range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  robot.lock();
  ArIRs irs;
  robot.addRangeDevice(&irs);
  pathTask.addRangeDevice(&irs, ArPathPlanningTask::CURRENT);

  // Add bumpers range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);
  pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Add range device which uses forbidden regions given in the map to give virtual
  // range device readings to ARNL.  (so it avoids obstacles
  // detected by this device)
  ArForbiddenRangeDevice forbidden(&map);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  robot.unlock();


  // Action to slow down robot when localization score drops but not lost.
  ArActionSlowDownWhenNotCertain actionSlowDown(&LOCTASK);
  pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

  // Action to stop the robot when localization is "lost" (score too low)
  ArActionLost actionLostPath(&LOCTASK, &pathTask);
  pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

#ifndef ARNL_SONARLOC
  // Arnl uses this object when it must replan its path because its
  // path is completely blocked.  It will use an older history of sensor
  // readings to replan this new path.  This should not be used with SONARNL
  // since sonar readings are not accurate enough and may prevent the robot
  // from planning through space that is actually clear.
  ArGlobalReplanningRangeDevice replanDev(&pathTask);
#endif 

  
  // Service to provide drawings of data in the map display :
  ArServerInfoDrawings drawings(&server);
  drawings.addRobotsRangeDevices(&robot);
#ifndef ARNL_SONARLOC
  drawings.addRangeDevice(&replanDev);
#endif 

  /* Draw a box around the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) */
  ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorP(&pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP); 

#ifdef ARNL_LASERLOC
  /* Show the sample points used by MCL */
  ArDrawingData drawingDataL("polyDots", ArColor(0,255,0), 100, 75);
  ArFunctor2C<ArLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorL(&locTask, &ArLocalizationTask::drawRangePoints);
  drawings.addDrawing(&drawingDataL, "Localization Points", &drawingFunctorL);
#endif

#ifdef ARNL_GPSLOC
  /* Show the positions calculated by GPS localization */

  ArDrawingData drawingDataG("polyDots", ArColor(100,100,255), 130, 61);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG(&gpsLocTask, &ArGPSLocalizationTask::drawGPSPoints);
  drawings.addDrawing(&drawingDataG, "GPS Points", &drawingFunctorG);

  ArDrawingData drawingDataG2("polyDots", ArColor(255,100,100), 100, 62);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG2(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanPoints);
  drawings.addDrawing(&drawingDataG2, "Kalman Points", &drawingFunctorG2);

  ArDrawingData drawingDataG3("polyDots", ArColor(100,255,100), 70, 63);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG3(&gpsLocTask, &ArGPSLocalizationTask::drawOdoPoints);
  drawings.addDrawing(&drawingDataG3, "Odom. Points", &drawingFunctorG3);

  ArDrawingData drawingDataG4("polyDots", ArColor(255,50,50), 100, 75);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG4(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanRangePoints);
  drawings.addDrawing(&drawingDataG4, "KalRange Points", &drawingFunctorG4);

  ArDrawingData drawingDataG5("polySegments", ArColor(100,0,255), 1, 78);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG5(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanVariance);
  drawings.addDrawing(&drawingDataG5, "VarGPS", &drawingFunctorG5);
#endif

  // "Custom" commands. You can add your own custom commands here, they will
  // be available in MobileEyes' custom commands (enable in the toolbar or
  // access through Robot Tools)
  ArServerHandlerCommands commands(&server);


  // These provide various kinds of information to the client:
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  serverInfoPath.addSearchRectangleDrawing(&drawings);
  serverInfoPath.addControlCommands(&commands);

  // Provides localization info and allows the client (MobileEyes) to relocalize at a given
  // pose:
  ArServerInfoLocalization serverInfoLocalization(&server, &robot, &LOCTASK);
  ArServerHandlerLocalization serverLocHandler(&server, &robot, &LOCTASK);

  // If you're using MobileSim, ArServerHandlerLocalization sends it a command
  // to move the robot's true pose if you manually do a localization through 
  // MobileEyes.  To disable that behavior, use this constructor call instead:
  // ArServerHandlerLocalization serverLocHandler(&server, &robot, true, false);
  // The fifth argument determines whether to send the command to MobileSim.

  // Provide the map to the client (and related controls):
  ArServerHandlerMap serverMap(&server, &map);

  // These objects add some simple (custom) commands to 'commands' for testing and debugging:
  ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
//  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


  // service that allows the client to monitor the communication link status
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor handlerCommMonitor(&server);



  // service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());



  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
#ifdef MOGS
  #warning Will use GPS localization's starting point in GOTO server mode, ignoring other localization tasks (FIXME?)
  ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map,
			    gpsLocTask.getRobotHome(),
			    gpsLocTask.getRobotHomeCallback());
#else
  ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map,
			    locTask.getRobotHome(),
			    locTask.getRobotHomeCallback());
#endif


  // Mode To stop and remain stopped:
  ArServerModeStop modeStop(&server, &robot);

#ifndef ARNL_SONARLOC
  // Cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, if using SONARNL to localize, then don't do this
  // since localization may get lost)
  ArSonarAutoDisabler sonarAutoDisabler(&robot);
#endif

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
//  ArServerModeDrive modeDrive(&server, &robot);            // Older mode for compatability



  // Prevent normal teleoperation driving if localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost actionLostRatioDrive(&LOCTASK, &pathTask, &modeRatioDrive);
  modeRatioDrive.getActionGroup()->addAction(&actionLostRatioDrive, 110);

  // Add drive mode section to the configuration, and also some custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive.addControlCommands(&commands);

  // Wander mode (also prevent wandering if lost):
  ArServerModeWander modeWander(&server, &robot);
  ArActionLost actionLostWander(&LOCTASK, &pathTask, &modeWander);
  modeWander.getActionGroup()->addAction(&actionLostWander, 110);


  // This provides a small table of interesting information for the client
  // to display to the operator. You can add your own callbacks to show any
  // data you want.
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  
  // Provide a set of informational data (turn on in MobileEyes with
  // View->Custom Details)

  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));

#ifdef ARNL_LASERLOC
  Aria::getInfoGroup()->addStringDouble(
	  "Laser Localization Score", 8, 
	  new ArRetFunctorC<double, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getLocalizationScore),
	  "%.03f");
  Aria::getInfoGroup()->addStringInt(
	  "Laser Loc Num Samples", 8, 
	  new ArRetFunctorC<int, ArLocalizationTask>(
		  &locTask, &ArLocalizationTask::getCurrentNumSamples),
	  "%4d");
#elif defined(ARNL_SONARLOC)
  Aria::getInfoGroup()->addStringDouble(
	  "Sonar Localization Score", 8, 
	  new ArRetFunctorC<double, ArSonarLocalizationTask>(
		  &locTask, 
      &ArSonarLocalizationTask::getLocalizationScore),
	  "%.03f");
  Aria::getInfoGroup()->addStringInt(
	  "Sonar Loc Num Samples", 8, 
	  new ArRetFunctorC<int, ArSonarLocalizationTask>(
		  &locTask, &ArSonarLocalizationTask::getCurrentNumSamples),
	  "%4d");
#endif

#ifdef ARNL_GPSLOC
  const char *dopfmt = "%2.6f";
  const char *posfmt = "%2.8f";
  const char *altfmt = "%3.6f m";
  const char *errfmt = "%2.6f m";
  Aria::getInfoGroup()->addStringString(
	    "GPS Fix Mode", 25,
	    new ArConstRetFunctorC<const char*, ArGPS>(gps, &ArGPS::getFixTypeName)
    );
  Aria::getInfoGroup()->addStringInt(
	    "GPS Num Sats", 4,
	    new ArConstRetFunctorC<int, ArGPS>(gps, &ArGPS::getNumSatellitesTracked)
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS HDOP", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getHDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS VDOP", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getVDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS PDOP", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getPDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Latitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Longitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Altitude", 8,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitude),
      altfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lat. Err.", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lon. Err.", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Alt. Err.", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitudeError),
      errfmt
    );



  // Add some "custom commands" for setting up initial GPS offset and heading.
  gpsLocTask.addLocalizationInitCommands(&commands);
  
#endif

  // Display gyro status if gyro is enabled and is being handled by the firmware (gyro types 2, 3, or 4).
  // (If the firmware detects an error communicating with the gyro or IMU it
  // returns a flag, and stops using it.)
  // (This gyro type parameter, and fault flag, are only in ARCOS, not Seekur firmware)
  if(robot.getOrigRobotConfig() && robot.getOrigRobotConfig()->getGyroType() > 1)
  {
    Aria::getInfoGroup()->addStringString(
          "Gyro/IMU Status", 10,
          new ArGlobalRetFunctor1<const char*, ArRobot*>(&getGyroStatusString, &robot)
      );
  }


#ifdef ARNL_DOCKING
  // Setup the dock if there is a docking system on board.
  ArServerModeDock *modeDock = NULL;
  modeDock = ArServerModeDock::createDock(&server, &robot, &locTask, 
					  &pathTask);
  if (modeDock != NULL)
  {
    modeDock->checkDock();
    modeDock->addAsDefaultMode();
    modeDock->addToConfig(Aria::getConfig());
    modeDock->addControlCommands(&commands);
  }
#endif



  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();
    // TODO move up near where stop mode is created?




#ifdef ARNL_MAPPING

  /* Services that allow the client to initiate scanning with the laser to
     create maps in Mapper3 (So not possible with SONARNL): */

  ArServerHandlerMapping handlerMapping(&server, &robot, firstLaser, 
					fileDir, "", true);

#ifdef ARNL_LASERLOC
  // make laser localization stop while mapping
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (&locTask, &ArLocalizationTask::setIdleFlag, true));

  // and then make it start again when we're doine
  handlerMapping.addMappingEndCallback(
	  new ArFunctor1C<ArLocalizationTask, bool>
	  (&locTask, &ArLocalizationTask::setIdleFlag, false));
#endif

#ifdef ARNL_GPSLOC
  // Save GPS positions in the .2d scan log when making a map
  handlerMapping.addLocationData("robotGPS", 
			    gpsLocTask.getPoseInterpPositionCallback());

  // add the starting latitude and longitude info to the .2d scan log
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArGPSLocalizationTask, ArServerHandlerMapping *>
	  (&gpsLocTask, &ArGPSLocalizationTask::addScanInfo, 
	   &handlerMapping));
#endif

  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping.addMappingStartCallback(actionLostPath.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostRatioDrive.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostWander.getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping.addMappingEndCallback(actionLostPath.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostRatioDrive.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostWander.getEnableCB());

  // don't let forbidden lines show up as obstacles while mapping
  // (they'll just interfere with driving while mapping, and localization is off anyway)
  handlerMapping.addMappingStartCallback(forbidden.getDisableCB());

  // let forbidden lines show up as obstacles again as usual after mapping
  handlerMapping.addMappingEndCallback(forbidden.getEnableCB());

#endif // ARNL_MAPPING


  /*
  // If we are on a simulator, move the robot back to its starting position,
  // and reset its odometry.
  // This will allow localizeRobotAtHomeBlocking() below will (probably) work (it
  // tries current odometry (which will be 0,0,0) and all the map
  // home points.
  // (Ignored by a real robot)
  //robot.com(ArCommands::SIM_RESET);
  */


  // create a pose storage class, this will let the program keep track
  // of where the robot is between runs...  after we try and restore
  // from this file it will start saving the robot's pose into the
  // file
  ArPoseStorage poseStorage(&robot);
  /// if we could restore the pose from then set the sim there (this
  /// won't do anything to the real robot)... if we couldn't restore
  /// the pose then just reset the position of the robot (which again
  /// won't do anything to the real robot)
  if (poseStorage.restorePose("robotPose"))
    serverLocHandler.setSimPose(robot.getPose());
  else
    robot.com(ArCommands::SIM_RESET);



  /* File transfer services: */
  
#pragma GPP off
#ifdef WIN32
  // Not implemented for Windows yet.
  ArLog::log(ArLog::Normal, "Note, file upload/download services are not implemented for Windows; not enabling them.");
#else
  // This block will allow you to set up where you get and put files
  // to/from, just comment them out if you don't want this to happen
  // /*
  ArServerFileLister fileLister(&server, fileDir);
  ArServerFileToClient fileToClient(&server, fileDir);
  ArServerFileFromClient fileFromClient(&server, fileDir, "/tmp");
  ArServerDeleteFileOnServer deleteFileOnServer(&server, fileDir);
  // */
#endif
#pragma GPP on

    /* Video image streaming, and camera controls (Requires SAVserver or ACTS) */

  // Forward any video if either ACTS or SAV server are running.
  // You can find out more about SAV and ACTS on our website
  // http://robots.activmedia.com. ACTS is for color tracking and is
  // a seperate product. SAV just does software A/V transmitting and is
  // free to all our customers. Just run ACTS or SAV server before you
  // start this program and this class here will forward video from the
  // server to the client.
  ArHybridForwarderVideo videoForwarder(&server, "localhost", 7070);
  
  // make a camera to use in case we have video. the camera collection collects
  // multiple ptz cameras 
  ArPTZ *camera = NULL;
  ArServerHandlerCamera *handlerCamera = NULL;
  ArCameraCollection *cameraCollection = NULL;

  // if we have video then set up a camera 
  if (videoForwarder.isForwardingVideo())
  {

    cameraCollection = new ArCameraCollection();
    cameraCollection->addCamera("Cam1", "VCC4", "Camera", "VCC4");

    videoForwarder.setCameraName("Cam1");
    videoForwarder.addToCameraCollection(*cameraCollection);

    bool invertedCamera = false;
    camera = new ArVCC4(&robot,	invertedCamera, 
			                  ArVCC4::COMM_UNKNOWN, true, true);
    camera->init();

    handlerCamera = new ArServerHandlerCamera("Cam1", 
		                                           &server, 
					                                     &robot,
					                                     camera, 
					                                     cameraCollection);

#if 0 // todo keep? 
    pathTask.addNewGoalCB(
	    new ArFunctor1C<ArServerHandlerCamera, ArPose>(
		    handlerCamera, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalSetGoal));
    pathTask.addGoalFinishedCB(
	    new ArFunctorC<ArServerHandlerCamera>(
		    handlerCamera, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalClearGoal));
#endif
  }

  // After all of the cameras / videos have been created and added to the collection,
  // then start the collection server.
  //
  if (cameraCollection != NULL) {
    new ArServerHandlerCameraCollection(&server, cameraCollection);
  }




    /* Load configuration values, map, and begin! */

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(&parser);

  // Read in parameter files.
  ArLog::log(ArLog::Normal, "Loading config file %s into ArConfig...", Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(6);
  }

  // Warn if there is no map
  if (map.getFileName() == NULL || strlen(map.getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### No map file is set up, you can make a map with the following procedure");
#ifdef ARNL
    ArLog::log(ArLog::Normal, "   0) You can find this information in README.txt or docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   1) Connect to this server with MobileEyes");
    ArLog::log(ArLog::Normal, "   2) Go to Tools->Map Creation->Start Scan");
    ArLog::log(ArLog::Normal, "   3) Give the map a name and hit okay");
    ArLog::log(ArLog::Normal, "   4) Drive the robot around your space (see docs/Mapping.txt");
    ArLog::log(ArLog::Normal, "   5) Go to Tools->Map Creation->Stop Scan");
    ArLog::log(ArLog::Normal, "   6) Start up Mapper3");
    ArLog::log(ArLog::Normal, "   7) Go to File->Open on Robot");
    ArLog::log(ArLog::Normal, "   8) Select the .2d you created");
    ArLog::log(ArLog::Normal, "   9) Create a .map");
    ArLog::log(ArLog::Normal, "  10) Go to File->Save on Robot");
    ArLog::log(ArLog::Normal, "  11) In MobileEyes, go to Tools->Robot Config");
    ArLog::log(ArLog::Normal, "  12) Choose the Files section");
    ArLog::log(ArLog::Normal, "  13) Enter the path and name of your new .map file for the value of the Map parameter.");
    ArLog::log(ArLog::Normal, "  14) Press OK and your new map should become the map used");
    ArLog::log(ArLog::Normal, "");    
#elif defined(SONARNL)
    ArLog::log(ArLog::Normal, "   0) You can find this information in README.txt or docs/SonarMapping.txt");
    ArLog::log(ArLog::Normal, "   1) Start up Mapper3Basic");
    ArLog::log(ArLog::Normal, "   2) Go to File->New");
    ArLog::log(ArLog::Normal, "   3) Draw a line map of your area (make sure it is to scale)");
    ArLog::log(ArLog::Normal, "   4) Go to File->Save on Robot");
    ArLog::log(ArLog::Normal, "   5) In MobileEyes, go to Tools->Robot Config");
    ArLog::log(ArLog::Normal, "   6) Choose the Files section");
    ArLog::log(ArLog::Normal, "   7) Enter the path and name of your new .map file for the value of the Map parameter.");
    ArLog::log(ArLog::Normal, "   8) Press OK and your new map should become the map used");
    ArLog::log(ArLog::Normal, "");    
#endif
#ifdef ARNL_GPSLOC
    ArLog::log(ArLog::Normal, "\n   See docs/GPSMapping.txt for instructions on creating a map for GPS localization");
#endif
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // Do an initial localization of the robot. It tries all the home points
  // in the map, as well as the robot's current odometric position, as possible
  // places the robot is likely to be at startup.   If successful, it will
  // also save the position it found to be the best localized position as the
  // "Home" position, which can be obtained from the localization task (and is
  // used by the "Go to home" network request).
#ifdef _GPP_OMIT_
    // TODO do this for all localizations someday:
  LOCTASK.localizeRobotAtHomeBlocking();
#endif
#if defined(ARNL_LASERLOC) || defined(ARNL_SONARLOC)
  locTask.localizeRobotAtHomeBlocking();
#endif
  
#ifdef ARNL_MULTIROBOT
  // Let the client switch manager (for multirobot) spin off into its own thread
  // TODO move to multirobot example?
  clientSwitch.runAsync();
#endif

  // Start the networking server's thread
  server.runAsync();


  // Add a key handler so that you can exit by pressing
  // escape. Note that this key handler, however, prevents this program from
  // running in the background (e.g. as a system daemon or run from 
  // the shell with "&") -- it will lock up trying to read the keys; 
  // remove this if you wish to be able to run this program in the background.
  ArKeyHandler *keyHandler;
  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    robot.lock();
    robot.attachKeyHandler(keyHandler);
    robot.unlock();
    puts("Server running. To exit, press escape.");
  }

  // Enable the motors and wait until the robot exits (disconnection, etc.) or this program is
  // canceled.
  robot.enableMotors();
  robot.waitForRunExit();
  Aria::exit(0);
}

