/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "stdafx.h"
#include "Aria.h"
#include "ArNetworking.h"

/** @example serverDemo.cpp Example ArNetworking server providing teleoperation,
 * sonar data, control the camera, etc.
 *
 * This is a basic ArNetworking server. It connects to a robot or simulator,
 * including, if available, IRs, gyro, and bumpers.  Give the option 
 * "-connectLaser" on the command line to enable the laser rangefinder,
 * if available.
 *
 * Run "./serverDemo -help" for a full list of command line options.
 *
 * Once running, connect to this server with a a client such as 
 * MobileEyes.
 *
 * This server provides the following services:
 *  - User login (optional)
 *  - Basic robot telemetry information 
 *  - Range sensor data values (not used by MobileEyes)
 *  - Graphics representing range sensor reading positions
 *  - Teleoperation modes (including safe/unsafe drive modes)
 *  - Wander mode
 *  - Various advanced "custom" commands to control logging, debugging, etc.
 *  - If an ACTS or SAV server is running, forward the video stream 
 *  - Camera control (pan/tilt/zoom) if cameras are available
 *
 * Note that this program requires a terminal to run -- i.e. you can't run
 * it in the background in Linux.  To modify it to allow that, remove the key
 * handler code in main().
 */

int main(int argc, char **argv)
{
  // mandatory init
  Aria::init();

  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // set up our parser
  ArArgumentParser parser(&argc, argv);

  // load the default arguments 
  parser.loadDefaultArguments();

  // robot
  ArRobot robot;
  // set up our simple connector
  ArRobotConnector robotConnector(&parser, &robot);


  // add a gyro, it'll see if it should attach to the robot or not
  ArAnalogGyro gyro(&robot);


  // set up the robot for connecting
  if (!robotConnector.connectRobot())
  {
    printf("Could not connect to robot... exiting\n");
    Aria::exit(1);
  }

  // our base server object
  ArServerBase server;

  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  ArServerSimpleOpener simpleOpener(&parser);


  ArClientSwitchManager clientSwitchManager(&server, &parser);

  // parse the command line... fail and print the help if the parsing fails
  // or if the help was requested
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    Aria::logOptions();
    Aria::exit(1);
  }

  // Set up where we'll look for files such as user/password 
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "ArNetworking/examples");

  // first open the server up
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    if (simpleOpener.wasUserFileBad())
      printf("Bad user/password/permissions file\n");
    else
      printf("Could not open server port\n");
    exit(1);
  }

  // Range devices:
  
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);

  ArIRs irs;
  robot.addRangeDevice(&irs);

  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);

  // attach services to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoDrawings drawings(&server);

  // modes for controlling robot movement
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
  ArServerModeWander modeWander(&server, &robot);
  modeStop.addAsDefaultMode();
  modeStop.activate();

  // set up the simple commands
  ArServerHandlerCommands commands(&server);
  ArServerSimpleComUC uCCommands(&commands, &robot);  // send commands directly to microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // control debug logging
  ArServerSimpleComGyro gyroCommands(&commands, &robot, &gyro); // configure gyro
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot); // control more debug logging
  ArServerSimpleServerCommands serverCommands(&commands, &server); // control ArNetworking debug logging
  ArServerSimpleLogRobotDebugPackets logRobotDebugPackets(&commands, &robot, ".");  // debugging tool

  // This is an older drive mode. ArServerModeDrive is newer and generally performs better,
  // but you can use this for old clients if neccesary.
  //ArServerModeDrive modeDrive(&server, &robot);
  //modeDrive.addControlCommands(&commands); // configure the drive modes (e.g. enable/disable safe drive)

  ArServerHandlerConfig serverHandlerConfig(&server, Aria::getConfig()); // make a config handler
  ArLog::addToConfig(Aria::getConfig()); // let people configure logging
  
  // start the robot running, true means that if we lose connection the run thread stops
  robot.enableMotors();
  robot.runAsync(true);
  
  ArMap arMap;
  arMap.readFile("pavsup.map");
  ArServerHandlerMap serverMap(&server, &arMap);

  drawings.addRobotsRangeDevices(&robot);

  // log whatever we wanted to before the runAsync
  simpleOpener.checkAndLog();
  // now let it spin off in its own thread
  server.runAsync();

  printf("Server is now running...\n");

  clientSwitchManager.runAsync();

  robot.waitForRunExit();
  Aria::exit(0);
}


