#include "stdafx.h"
#include "ConsoleLogger.h"
#include <stdexcept>
#include <boost/thread.hpp>
#include "SauronArRobot.h"

#if 0
/** Main function */
int 
main(int argc, char *argv[])
{
  // Initialize location of Aria, Arnl and their args.
  Aria::init();
  Arnl::init();
  
  // To initialize, but log to a file, do this instead of the above:
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
  //ArLog::init(ArLog::File, ArLog::Verbose);
 
  // The robot object
  ArRobot robot;

  // Our server
  ArServerBase server;
  
  // Parse the command line arguments.
  ArArgumentParser parser(&argc, argv);

  // Set up our simpleConnector
  ArSimpleConnector simpleConnector(&parser);

  // Set up our simpleOpener
  ArServerSimpleOpener simpleOpener(&parser);

  // Set up our client for the central server
  ArClientSwitchManager clientSwitch(&server, &parser);
    
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // set up a gyro
  ArAnalogGyro gyro(&robot);
  
  // Parse arguments for the simple connector.
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    ArLog::log(ArLog::Normal, "\nUsage: %s -map mapfilename\n", argv[0]);
    Aria::logOptions();
    Aria::exit(1);
  }


  // The laser object, will be used if we have one
  ArSick sick;

  // Add the laser to the robot
  robot.addRangeDevice(&sick);

  // Sonar, must be added to the robot, used by teleoperation and wander to
  // detect obstacles, and for localization if SONARNL
  ArSonarDevice sonarDev;

  // Add the sonar to the robot
  robot.addRangeDevice(&sonarDev);
  
  // Set up where we'll look for files
  char fileDir[1024];
  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
			 "examples");
  ArLog::log(ArLog::Normal, "Installation directory is: %s\nMaps directory is: %s\n", Aria::getDirectory(), fileDir);
  
  // Set up the map, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the current directory
  // instead
  ArMap arMap(fileDir);
  arMap.readFile("pavsup.map");
  // set it up to ignore empty file names (otherwise the parseFile
  // on the config will fail)
  arMap.setIgnoreEmptyFileName(true);

  // Make the path task planning task
  ArPathPlanningTask pathTask(&robot, &sick, &sonarDev, &arMap);

  // Set up things so data can be logged (only do it with the laser
  // since it can overrun a 9600 serial connection which the sonar is
  // more likely to have)
  ArDataLogger dataLogger(&robot);
  dataLogger.addToConfig(Aria::getConfig());

  // add our logging to the config
  ArLog::addToConfig(Aria::getConfig());

  // First open the server 
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    if (simpleOpener.wasUserFileBad())
      ArLog::log(ArLog::Normal, "Bad user file");
    else
      ArLog::log(ArLog::Normal, "Could not open server port");
    exit(2);
  }

  // Connect the robot
  if (!simpleConnector.connectRobot(&robot))
  {
    ArLog::log(ArLog::Normal, "Could not connect to robot... exiting");
    Aria::exit(3);
  }

  // Set up a class that'll put the movement and gyro parameters into ArConfig
  ArRobotConfig robotConfig(&robot);
  robotConfig.addAnalogGyro(&gyro);

  robot.enableMotors();
  robot.clearDirectMotion();

  // if we are connected to a simulator, reset it to its start position
  robot.comInt(ArCommands::RESETSIMTOORIGIN, 1);
  robot.moveTo(ArPose(0,0,0));


  // Set up laser using connector (command line arguments, etc.)
  simpleConnector.setupLaser(&sick);

  // Start the robot thread.
  robot.runAsync(true);
  
  // Start the laser thread.
  sick.runAsync();

  // Try to connect the laser
  if (!sick.blockingConnect())
    ArLog::log(ArLog::Normal, "Warning: Couldn't connect to SICK laser, it won't be used");
  else
    ArLog::log(ArLog::Normal, "Connected to laser.");


  // Add additional range devices to the robot and path planning task.
  // IRs if the robot has them.
  robot.lock();
  ArIRs irs;
  robot.addRangeDevice(&irs);
  pathTask.addRangeDevice(&irs, ArPathPlanningTask::CURRENT);

  // Bumpers.
  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);
  pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Forbidden regions from the map
  ArForbiddenRangeDevice forbidden(&arMap);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  // This is the place to add a range device which will hold sensor data
  // and delete it appropriately to replan around blocked paths.
  ArGlobalReplanningRangeDevice replanDev(&pathTask);

  // Create objects that add network services:
  
  // Drawing in the map display:
  ArServerInfoDrawings drawings(&server);
  drawings.addRobotsRangeDevices(&robot);
  drawings.addRangeDevice(&replanDev);

  /* If you want to draw the destination put this code back in:
  ArServerDrawingDestination destination(
	  &drawings, &pathTask, "destination",
	  500, 500,
	  new ArDrawingData("polyDots",
			    ArColor(0xff, 0xff, 0x0),
			    800, // size
			    49), // just below the robot
  */

  /* If you want to see the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) 
  ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
  drawingFunctorP(pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP); 
  */

  /* If you want to see the points making up the local path in addition to the
   * main path use this. 
  ArDrawingData drawingDataP2("polyDots", ArColor(0,128,0), 100, 70);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
  drawingFunctorP2(pathTask, &ArPathPlanningTask::drawPathPoints);
  drawings.addDrawing(&drawingDataP2, "Path Points", &drawingFunctorP2);
  */

  // Misc. simple commands:
  ArServerHandlerCommands commands(&server);


  // These provide various kinds of information to the client:
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  serverInfoPath.addSearchRectangleDrawing(&drawings);
  serverInfoPath.addControlCommands(&commands);

  // Provide the map to the client (and related controls):
  // This uses both lines and points now, since everything except
  // sonar localization uses both (path planning with sonar still uses both)
  ArServerHandlerMap serverMap(&server, &arMap);

  

  // Add some simple (custom) commands for testing and debugging:
  ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
  ArServerSimpleComGyro gyroCommands(&commands, &robot, &gyro);        // monitor the gyro
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
  ArServerModeGoto modeGoto(&server, &robot, &pathTask, &arMap, ArPose(0,0,0));

  // Add a simple (custom) command that allows you to give a list of 
  // goals to tour, instead of all. Useful for testing and debugging.
  modeGoto.addTourGoalsInListSimpleCommand(&commands);

  // Mode To stop and remain stopped:
  ArServerModeStop modeStop(&server, &robot);

  // cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, this should not be done if you need the sonar
  // data to localize, or for other purposes while stopped)
  ArSonarAutoDisabler sonarAutoDisabler(&robot);

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  // New, improved mode
  ArServerModeDrive modeDrive(&server, &robot);            // Older mode for compatability

  // Drive mode's configuration and custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeDrive.addControlCommands(&commands);
  modeRatioDrive.addControlCommands(&commands);

  // Wander mode 
  ArServerModeWander modeWander(&server, &robot);


  // This provides a small table of interesting information for the client
  // to display to the operator:
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  
  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));


  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();




  /* File transfer services: */
  
#ifdef WIN32
  // these server file things don't work under windows yet
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

  // Create the service that allows the client to monitor the communication 
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor handlerCommMonitor(&server);

  // Create service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());


  
  // Read in parameter files.
  Aria::getConfig()->useArgumentParser(&parser);
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
  {
    ArLog::log(ArLog::Normal, "\nUsage: %s -map mapfilename\n", argv[0]);
    simpleConnector.logOptions();
    simpleOpener.logOptions();
    Aria::exit(6);
  }

  // Warn if there is no map
  if (arMap.getFileName() == NULL || strlen(arMap.getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### Warning, No map file is set up, you can make a map with sickLogger or arnlServer, and Mapper3; More info in docs/Mapping.txt and README.txt. Set the map with the -map command line option, or by changing the config with MobileEyes or by editing the config file.");
    ArLog::log(ArLog::Normal, "");    
  }

  // find out where we'll want to put files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // If you want MobileSim to try and load up the same map as you are
  // using in guiServer then uncomment out the next line and this object
  // will send a command to MobileSim to do so, but make sure you start 
  // MobileSim from the Arnl/examples directory or use the --cwd option, 
  // so that the map names used by MobileSim match  the map names used 
  // by guiServer
  //ArSimMapSwitcher mapSwitcher(&robot, &arMap);



  /* Finally, get ready to run the robot: */


  robot.unlock();

  server.runAsync();

  // Add a key handler (mostly so that on windows you can exit by pressing
  // escape.) This key handler, however, prevents this program from
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
    printf("To exit, press escape.\n");
  }

  robot.waitForRunExit();
  Aria::exit(0);
}

#endif
#if 1

#define TEST_LOG(level) FILE_LOG(level) << "LocalizationTests: "
#include <windows.h>
#include <fstream>


ULONGLONG epoch = -1;

ULONGLONG getTimeMs()
{
	SYSTEMTIME systemTime;
	GetSystemTime( &systemTime );

	FILETIME fileTime;
	SystemTimeToFileTime( &systemTime, &fileTime );

	ULARGE_INTEGER uli;
	uli.LowPart = fileTime.dwLowDateTime; // could use memcpy here!
	uli.HighPart = fileTime.dwHighDateTime;

	ULONGLONG systemTimeIn_ms( uli.QuadPart/10000 );
	if(epoch == -1) {
		epoch = systemTimeIn_ms; return 0;
	} else {
		return systemTimeIn_ms - epoch;
	}
}

std::ofstream grafico("grafico.csv", std::ios::trunc);

CConsoleLogger console;
boost::mutex consoleMutex;
sauron::SauronArRobot* pRobot;

sauron::Pose lastEstimatedPose;
sauron::Pose lastTruePose;

void cls(int n)
{
	while(n-- > 0)
		console.print("\n");
}


void printEstimatedPose(const sauron::Pose& currentPose)
{
	boost::unique_lock<boost::mutex> lock(consoleMutex);

	//pRobot->lock();
	
	//pRobot->unlock();

	console.printf("%.3f\t%.3f\t%.3f\n", currentPose.X(), currentPose.Y(), currentPose.Theta());
	ArPose arPose = pRobot->getTruePose();
	sauron::Pose truePose(arPose.getX(), arPose.getY(), sauron::trigonometry::degrees2rads(arPose.getTh()));
	console.printf("%.3f\t%.3f\t%.3f\n", truePose.X(), truePose.Y(), truePose.Theta());
    double erroX = currentPose.X() - truePose.X();
    double erroY = currentPose.Y() - truePose.Y();
    double erroTheta = currentPose.Theta() - truePose.Theta();
	console.printf("Erro em X = %.3f cm\n", erroX );
	console.printf("Erro em Y = %.3f cm\n", erroY );
	console.printf("Erro em Theta = %.3f radianos\n", erroTheta );

	grafico << getTimeMs() / 1000.0 << ";" << erroX << ";" << erroY << ";" << erroTheta << ";" << currentPose.X() << ";" << currentPose.Y() << ";" << currentPose.Theta() << ";" <<
		truePose.X() << ";" << truePose.Y() << ";" << truePose.Theta() << ";" << truePose.getDistance(currentPose) << std::endl;
	grafico.flush();

	cls(19);

	if(!(currentPose.X() == lastEstimatedPose.X() && currentPose.Y() == lastEstimatedPose.Y() &&
		currentPose.Theta() == lastEstimatedPose.Theta())) {
		TEST_LOG(logDEBUG1) << "Posição estimada: " << currentPose;
		TEST_LOG(logDEBUG1) << "Erro: ( " << erroX << " , " << erroY << " , " << erroTheta << " )";
		lastEstimatedPose = currentPose;
	}
	if(!(truePose.X() == lastTruePose.X() && truePose.Y() == lastTruePose.Y() &&
		truePose.Theta() == lastTruePose.Theta())) {
		TEST_LOG(logDEBUG1) << "Posição real: " << truePose;
		lastTruePose = truePose;
	}

	pRobot->moveTo(ArPose(currentPose.X() * 10, currentPose.Y() * 10,
		sauron::trigonometry::rads2degrees(currentPose.Theta())));
}

void startEstimatedPoseConsole(sauron::LocalizationManager& locManager)
{
	console.Create("Posicao atual");
	printEstimatedPose(locManager.getPose());
	locManager.addPoseChangedCallback(printEstimatedPose);
}


int principal(int argc, char** argv)
{
	
#pragma region boiler1
  // mandatory init
  Aria::init();
  Arnl::init();

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
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit(5);
  }

  // first open the server up
  if (!simpleOpener.open(&server, "", 240))
  {
    printf("Could not open server port\n");
    exit(1);
  }

  // Range devices:
  
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);

  
  ArMap map;
  char mapName[] = "pavsup.map";
  if(!map.readFile(mapName)) {
	 robot.disconnect(); // sem isso dá pau (pure virtual call)
	 throw std::invalid_argument(std::string("Mapa nao foi encontrado"));
  }

  // Make the path task planning task
  ArPathPlanningTask pathTask(&robot, &sonarDev, &map);


  // Forbidden regions from the map
  ArForbiddenRangeDevice forbidden(&map);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  // attach services to the server
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoDrawings drawings(&server);

  // modes for controlling robot movement
  // Mode To go to a goal or other specific point:
  ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map, ArPose(0,0,0));
  ArServerModeStop modeStop(&server, &robot);
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
  ArServerModeWander modeWander(&server, &robot);
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




  // Provide the map to the client (and related controls):
  ArServerHandlerMap serverMap(&server, &map);

  server.runAsync();
  printf("Server is now running...\n");
  clientSwitchManager.runAsync();

  sauron::LocalizationManager locManager(&robot, map, std::string(""));
	
  startEstimatedPoseConsole(locManager);

  std::cout	<< "Bem-vindo ao programa de testes mais bonito do Brasil" << std::endl 
			<< "Você está usando o mapa: " << mapName << std::endl
			<< "Digite a letra referente a opcao desejada:" << std::endl;

  char c = 'A';

  while(c != 'S' && c != 's')
  {
	  
	  std::cout << "Escolha a opção:"  << std::endl 
		  << "1. (M) Mostrar Posicao;" << std::endl
		  << "2. (P) Setar a Posição Inicial;" << std::endl
		  << "2. (S) Pegar a Posicao do Simulador;" << std::endl
		  << "3. (V) Setar Velocidades;" << std::endl
		  << "4. (T) Modo teleoperação;" << std::endl;

	  std::cin >> c;
	  ArPose arPose = robot.getTruePose();
	  switch(c){
		case 'm':
		case 'M':
			{
			sauron::Pose pose = locManager.getPose();
			std::cout << "Predicao: (" << pose.X() << ", " <<
				pose.Y() << ", " << pose.Theta() << ")" << std::endl;
			}
			break;
		case 'p':
		case 'P':
			double x, y, th;
			std::cin >> x;
			std::cin >> y;
			std::cin >> th;
			locManager.setInitialPose(sauron::Pose(x, y, th));
			break;
		case 's':
		case 'S':
			locManager.setInitialPose(sauron::Pose(arPose.getX(), arPose.getY(),
				sauron::trigonometry::degrees2rads(arPose.getTh())));
			break;
		case 'v':
		case 'V':
			std::cout << "Digite na ordem: velocidade velocidaRotacional e aperte ENTER:"  << std::endl;
			double vel;
			double rotVel;

			std::cin >> vel;
			std::cin >> rotVel;

			robot.setVel(vel);
			robot.setRotVel(rotVel);
			break;
		case 't':
		case 'T':
			robot.waitForRunExit();
			break;
		default:
			std::cout << "Ops... nao encontrei a opcao. Voce sabe ler instrucoes, seu macaco?" << std::endl;

	  } 
  }

  std::cout << "Ciao!";
  
  // Block execution of the main thread here and wait for the robot's task loop
  // thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
  // signal)
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
#endif