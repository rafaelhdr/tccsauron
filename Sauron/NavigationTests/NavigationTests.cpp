#pragma once
#include "stdafx.h"
#include "Localization/LocalizationManager.h"
#include "Navigation/RouteExecuter.h"
#include "Navigation/RobotController.h"

using namespace sauron;

sauron::SauronArRobot* pRobot;

void testTurnAngle(double fromX, double fromY, double degrees, double toX, double toY) {
	Pose from(fromX, fromY, trigonometry::degrees2rads(degrees));
		Pose to(toX, toY, 0);

		pRobot->moveTo(ArPose(fromX, fromY, degrees));
		std::cout << trigonometry::rads2degrees(RouteExecuter().getTurnAngle(from, to)) << std::endl;
		std::cout << trigonometry::rads2degrees(RouteExecuter().getTurnAngle(pRobot, to)) << std::endl;
}

void testTurn() {
	while(true) {
		sauron::robotController::setRobot(pRobot);
		std::cout << "Turning 180 degrees..." << std::endl;
		sauron::robotController::turn(3.14159);
		std::cout << "Finished!" << std::endl;
	}
}

void testGoTo(sauron::LocalizationManager* locManager) {
	std::cout << "Digite o ponto de destino (x, y) : ";
	double x, y;
	std::cin >> x;
	std::cin >> y;
	sauron::robotController::setRobot(pRobot);
	RouteExecuter route(pRobot);
	RouteExecuter::MoveResult result = route.goTo(locManager, Point2DDouble(x, y));
	switch(result)
	{
	case RouteExecuter::SUCCESS:
		std::cout << "sucesso" << std::endl;
		break;
	default:
		std::cout << "erro: " << result << std::endl;
		break;
	}
}

int principal(int argc, char** argv) {
#pragma region
  Aria::init();

  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // set up our parser
  ArArgumentParser parser(&argc, argv);

  sauron::SauronArRobot robot;
  pRobot = &robot;

  // load the default arguments 
  parser.loadDefaultArguments();
  // set up our simple connector
  ArRobotConnector robotConnector(&parser, &robot);

  // set up the robot for connecting
  if (!robotConnector.connectRobot())
  {
    printf("Could not connect to robot... exiting\n");
    Aria::exit(1);
  }

  // our base server object
  ArServerBase server;
  
  ArServerSimpleOpener simpleOpener(&parser);


  ArClientSwitchManager clientSwitchManager(&server, &parser);

  // parse the command line... fail and print the help if the parsing fails
  // or if the help was requested
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    Aria::logOptions();
    Aria::exit(1);
  }

  // Read in parameter files.
  Aria::getConfig()->useArgumentParser(&parser);

  // first open the server up
  if (!simpleOpener.open(&server, "", 240))
  {
    printf("Could not open server port\n");
    exit(1);
  }

  // Range devices:
  
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);


  // attach services to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoDrawings drawings(&server);

  ArServerModeStop modeStop(&server, &robot);
  modeStop.addAsDefaultMode();
  modeStop.activate();

  // set up the simple commands
  ArServerHandlerCommands commands(&server);
  ArServerSimpleComUC uCCommands(&commands, &robot);  // send commands directly to microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // control debug logging
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot); // control more debug logging
  ArServerSimpleServerCommands serverCommands(&commands, &server); // control ArNetworking debug logging
  ArServerSimpleLogRobotDebugPackets logRobotDebugPackets(&commands, &robot, ".");  // debugging tool

  // This is an older drive mode. ArServerModeDrive is newer and generally performs better,
  // but you can use this for old clients if neccesary.
  //ArServerModeDrive modeDrive(&server, &robot);
  //modeDrive.addControlCommands(&commands); // configure the drive modes (e.g. enable/disable safe drive)

  ArServerHandlerConfig serverHandlerConfig(&server, Aria::getConfig()); // make a config handler
  ArLog::addToConfig(Aria::getConfig()); // let people configure logging

  // You can use this class to send a set of arbitrary strings 
  // for MobileEyes to display, this is just a small example
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));
  /*
  Aria::getInfoGroup()->addStringInt(
	  "Laser Packet Count", 10, 
	  new ArRetFunctorC<int, ArSick>(&sick, 
					 &ArSick::getSickPacCount));
  */
  
  // start the robot running, true means that if we lose connection the run thread stops
  robot.enableMotors();
  robot.runAsync(true);

  drawings.addRobotsRangeDevices(&robot);

  // log whatever we wanted to before the runAsync
  simpleOpener.checkAndLog();

  server.runAsync();
  printf("Server is now running...\n");
  clientSwitchManager.runAsync();
#pragma endregion

  sauron::LocalizationManager locManager(&robot, ArMap(), std::string(""));
  while(true) {
	testGoTo(&locManager);
  }
  robot.waitForRunExit();
  robot.disconnect();
  Aria::exit(0);

  return 0;
}


int main(int argc, char** argv)
{
	try
	{
		Output2FILE::Stream() = fopen("sonar_log.log", "w");
		FILELog::ReportingLevel() = logDEBUG2;

		return principal(argc, argv);
	} catch(std::exception& e)
	{
		std::cerr << "std::exception: " << e.what() << std::endl;
		return -1;
	}
}