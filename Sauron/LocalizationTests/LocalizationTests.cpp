#include "stdafx.h"
#include "ConsoleLogger.h"
#include <stdexcept>
#include <boost/thread.hpp>
#include "SauronArRobot.h"
#include "SonarMatchPainter.h"

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

sauron::SonarMatchPainter* psonarPainter;
sauron::LocalizationManager* plocManager;

void printEstimatedPose(const sauron::Pose& currentPose)
{
	boost::unique_lock<boost::mutex> lock(consoleMutex);

	//pRobot->lock();
	
	//pRobot->unlock();

	psonarPainter->update(plocManager->getSonarMatches());

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

	/*pRobot->moveTo(ArPose(currentPose.X() * 10, currentPose.Y() * 10,
		sauron::trigonometry::rads2degrees(currentPose.Theta())));*/
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

  sauron::SonarMatchPainter sonarPainter(&drawings);
  psonarPainter = &sonarPainter;

  sauron::LocalizationManager locManager(&robot, map, std::string(""));
  plocManager = &locManager;

  startEstimatedPoseConsole(locManager);

  std::cout	<< "Bem-vindo ao programa de testes mais bonito do Brasil" << std::endl 
			<< "Você está usando o mapa: " << mapName << std::endl
			<< "Digite a letra referente a opcao desejada:" << std::endl;

  char c = 'A';

  while(c != 'Q' && c != 'q')
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