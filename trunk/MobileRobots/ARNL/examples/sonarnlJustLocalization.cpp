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


/** @example sonarnlJustLocalization.cpp Example which does localization only,
  but provides servers for remote operation with MobileEyes. */
 
/**
This example server only does localization and not path planning.  
To test this you can start up MobileEyes and connect to the robot 
then just drive the robot around with teleoperation commands.

Use this example as a starting point for adding your own control behavior,
but using ARNL for localization.

Note, to make your own custom planning/navigation information and controls
available in MobileEyes you can look at ArServerClasses.cpp and make your own class
that adds the same server commands as ArServerInfoPath and
ArServerModeGoto.

*/

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"



#include "ArSonarLocalizationTask.h"

int main(int argc, char *argv[])
{
  Aria::init();
  Arnl::init();

  ArRobot robot;
  ArServerBase server;
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArSimpleConnector simpleConnector(&parser);
  ArServerSimpleOpener simpleOpener (&parser);
  ArAnalogGyro gyro (&robot);

  parser.loadDefaultArguments();
  if (!Aria::parseArgs () || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions ();
    Aria::exit (1);
  }


  ArSonarDevice sonarDev;
  robot.addRangeDevice (&sonarDev);

  ArBumpers bumpers;
  robot.addRangeDevice (&bumpers);

  // Use the "examples" directory as a place to keep map files
  char fileDir[1024];
  ArUtil::addDirectories (fileDir, sizeof (fileDir), Aria::getDirectory (),
			"examples");

  // Set up the map, this will look for files in the examples
  // directory (unless the file name starts with a / \ or .
  // You can take out the argument to look in the current directory
  ArMap arMap (fileDir);
  // set it up to ignore empty file names (otherwise the parseFile
  // on the config will fail)
  arMap.setIgnoreEmptyFileName (true);


  ArLocalizationManager locManager(&robot, &arMap);

  ArSonarLocalizationTask locTask (&robot, &sonarDev, &arMap);
  locManager.addLocalizationTask(&locTask);


  // Add log controls to configuration
  ArLog::addToConfig (Aria::getConfig ());

  // Open the server port
  if (!simpleOpener.open (&server, fileDir, 240))
  {
    ArLog::log (ArLog::Normal, "Could not open server port");
    exit (2);
  }

  // Connect to the robot
  if (!simpleConnector.connectRobot (&robot))
  {
    ArLog::log (ArLog::Normal, "Could not connect to robot... exiting");
    Aria::exit (3);
  }

  robot.enableMotors ();
  robot.clearDirectMotion ();

  // reset the simulator to its start position
  robot.com(ArCommands::SIM_RESET);


  // run robot task cycle
  robot.runAsync (true);


  ArUtil::sleep (300);

  // Forbidden regions from the map.
  ArForbiddenRangeDevice forbidden (&arMap);
  robot.addRangeDevice (&forbidden);

  // Objects that provide network services:
  ArServerInfoRobot serverInfoRobot (&server, &robot);
  ArServerInfoSensor serverInfoSensor (&server, &robot);
  ArServerInfoLocalization serverInfoLocalization (&server, &robot, &locManager);
  ArServerHandlerLocalization serverLocHandler (&server, &robot, &locManager);
  ArServerHandlerMap serverMap (&server, &arMap);

  // Drawing services in the map display:
  ArServerInfoDrawings drawings (&server);
  drawings.addRobotsRangeDevices (&robot);

  // Misc. "custom " commands:
  ArServerHandlerCommands commands (&server);
  ArServerSimpleComUC uCCommands (&commands, &robot);
  ArServerSimpleComMovementLogging loggingCommands (&commands, &robot);
  ArServerSimpleComGyro gyroCommands (&commands, &robot, &gyro);
  ArServerSimpleComLogRobotConfig configCommands (&commands, &robot);

  // Set up the possible modes for remote control from a client such as
  // MobileEyes:

  // To stop and remain stopped:
  ArServerModeStop modeStop (&server, &robot);
  modeStop.addAsDefaultMode ();


  // Teleoperate by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive (&server, &robot);

  // Teloperation mode's configuration and special commands:
  modeRatioDrive.addControlCommands (&commands);
  modeRatioDrive.addToConfig (Aria::getConfig (), "Teleop settings");

  // Wander mode:
  ArServerModeWander
  modeWander (&server, &robot);

  // Prevent driving if localization is lost:
  ArActionLost actionLostRatioDrive (&locManager, NULL, &modeRatioDrive);
  modeRatioDrive.getActionGroup ()->addAction (&actionLostRatioDrive, 110);

  // Prevent wandering if lost:
  ArActionLost
  actionLostWander (&locManager, NULL, &modeWander);
  modeWander.getActionGroup ()->addAction (&actionLostWander, 110);

  // This provides a small table of interesting information for the client
  // to display to the operator:
     ArServerInfoStrings
     stringInfo (&server);
  Aria::getInfoGroup ()->addAddStringCallback (stringInfo.
					     getAddStringFunctor ());

  // Display localization score and more
  Aria::getInfoGroup ()->addStringDouble ("Localization Score", 8,
	new ArRetFunctorC < double, ArSonarLocalizationTask > (&locTask,
							   &ArSonarLocalizationTask::
							   getLocalizationScore),
					  		"%.03f");


Aria::getInfoGroup ()->addStringInt ("Motor Packet Count", 10,
				     new ArConstRetFunctorC < int,
				     ArRobot > (&robot,
						&ArRobot::getMotorPacCount));


  // Create service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig (&server, Aria::getConfig (),
				     Arnl::getTypicalDefaultParamFileName (),
				     Aria::getDirectory ());



  // Read in parameter files.
  Aria::getConfig ()->useArgumentParser (&parser);
  if (!Aria::getConfig ()->parseFile (Arnl::getTypicalParamFileName ()))
  {
    ArLog::log (ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit (5);
  }

  // Error if there is no map
  if (arMap.getFileName () == NULL || strlen (arMap.getFileName ()) <= 0)
  {
    ArLog::log(ArLog::Terse, "Warning, no map given. Use the -map command-line argument or modify the config using MobileEyes or by editing the parameter file.");
    ArLog::log(ArLog::Terse, "See the ARNL documentation, including MAPPING.txt or SONAR_MAPPING.txt.");
  }

  ArLog::log (ArLog::Normal, "Directory for maps and file serving: %s", fileDir);

  ArLog::log (ArLog::Normal, "See the ARNL README.txt for more information");


  robot.unlock();


  // Localize robot at home.
  locTask.localizeRobotAtHomeBlocking();
  
  server.runAsync();

  ArLog::log(ArLog::Normal, "Server now running on port %d. Press Control-C to exit.", server.getTcpPort());


  robot.waitForRunExit();
  Aria::exit(0);

}

