#include "stdafx.h"
#include "ConsoleLogger.h"
#include <stdexcept>
#include <boost/thread.hpp>
#include "SauronArRobot.h"

#define TEST_LOG(level) FILE_LOG(level) << "LocalizationTests: "
#include <windows.h>
#include <fstream>

AREXPORT const char *ArMapInterface::MAP_CATEGORY_2D = "2D-Map";
AREXPORT const char *ArMapInterface::MAP_CATEGORY_2D_MULTI_SOURCES = "2D-Map-Ex";
AREXPORT const char *ArMapInterface::MAP_CATEGORY_2D_EXTENDED = "2D-Map-Ex2";
/*
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
//sauron::SauronArRobot robot;

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

	robot.lock();
	robot.moveTo(ArPose(currentPose.X(), currentPose.Y(),
		sauron::trigonometry::rads2degrees(currentPose.Theta())));
	robot.unlock();

	console.printf("%.3f\t%.3f\t%.3f\n", currentPose.X(), currentPose.Y(), currentPose.Theta());
	ArPose arPose = robot.getTruePose();
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
		TEST_LOG(logDEBUG1) << "Posi��o estimada: " << currentPose;
		TEST_LOG(logDEBUG1) << "Erro: ( " << erroX << " , " << erroY << " , " << erroTheta << " )";
		lastEstimatedPose = currentPose;
	}
	if(!(truePose.X() == lastTruePose.X() && truePose.Y() == lastTruePose.Y() &&
		truePose.Theta() == lastTruePose.Theta())) {
		TEST_LOG(logDEBUG1) << "Posi��o real: " << truePose;
		lastTruePose = truePose;
	}
}

void startEstimatedPoseConsole(sauron::LocalizationManager& locManager)
{
	console.Create("Posicao atual");
	printEstimatedPose(locManager.getPose());
	locManager.addPoseChangedCallback(printEstimatedPose);
}
*/
int principal(int argc, char** argv)
{

#pragma region boiler1
  // Initialize some global data
  ArRobot robot;
  Aria::init();
  ArLog::init(ArLog::StdErr, ArLog::Terse);

  // If you want ArLog to print "Verbose" level messages uncomment this:
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // This object parses program options from the command line
  ArArgumentParser parser(&argc, argv);

  // Our networking server
  
  ArServerBase server;
  

  // Set up our simpleOpener, used to set up the networking server
  ArServerSimpleOpener simpleOpener(&parser);

  // Load some default values for command line arguments from /etc/Aria.args
  // (Linux) or the ARIAARGS environment variable.
  parser.loadDefaultArguments();

  // Central object that is an interface to the robot and its integrated
  // devices, and which manages control of the robot by the rest of the program.
  //ArRobot robot;

  // Object that connects to the robot or simulator using program options
  ArRobotConnector robotConnector(&parser, &robot);

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  if (!robotConnector.connectRobot())
  {
    // Error connecting:
    // if the user gave the -help argumentp, then just print out what happened,
    // and continue so options can be displayed later.
    if (!parser.checkHelpAndWarnUnparsed())
    {
      ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
    }
    // otherwise abort
    else
    {
      ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  // Parse the command line options. Fail and print the help message if the parsing fails
  // or if the help was requested with the -help option
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    Aria::logOptions();
    exit(1);
  }

  // Used to access and process sonar range data
  ArSonarDevice sonarDev;

  // Attach sonarDev to the robot so it gets data from it.
  robot.addRangeDevice(&sonarDev);

     /* Start the server */

  // Open the networking server
   if (!simpleOpener.open(&server))
   {
     ArLog::log(ArLog::Normal, "Error: Could not open server.");
     exit(2);
   }

   // Service to provide drawings of data in the map display :
  //ArServerInfoDrawings drawings(&server);
  //drawings.addRobotsRangeDevices(&robot);

  // service that allows the client to monitor the communication link status
  // between the robot and the client.
  //
  //ArServerHandlerCommMonitor handlerCommMonitor(&server);

  // These provide various kinds of information to the client:
  //ArServerInfoRobot serverInfoRobot(&server, &robot);
 // ArServerInfoSensor serverInfoSensor(&server, &robot);


  // Start the robot task loop running in a new background thread. The 'true' argument means if it loses
  // connection the task loop stops and the thread exits.
  robot.runAsync(true);

 
  // Sleep for a second so some messages from the initial responses
  // from robots and cameras and such can catch up
  ArUtil::sleep(1000);

  // We need to lock the robot since we'll be setting up these modes
  // while the robot task loop thread is already running, and they 
  // need to access some shared data in ArRobot.
  robot.lock();

  // now add all the modes for this demo
  // these classes are defined in ArModes.cpp in ARIA's source code.
  ArModeSilentTeleop teleop(&robot, "teleop", 't', 'T');


  // activate the default mode
  teleop.activate();

  // turn on the motors
  robot.comInt(ArCommands::ENABLE, 1);
#pragma endregion
  robot.unlock();

  ArMap map;
  char mapName[] = "pavsup.map";
  if(!map.readFile(mapName)) {
	  robot.disconnect(); // sem isso d� pau (pure virtual call)
	  throw std::invalid_argument(std::string("Mapa nao foi encontrado"));
  }

  // Provide the map to the client (and related controls):
   ArServerHandlerMap serverMap(&server, &map);

  server.runAsync();

  /*sauron::LocalizationManager locManager(&robot, map, std::string(""));
	
  startEstimatedPoseConsole(locManager);

std::cout	<< "Bem-vindo ao programa de testes mais bonito do Brasil" << std::endl 
			<< "Voc� est� usando o mapa: " << mapName << std::endl
			<< "Digite a letra referente a opcao desejada:" << std::endl;

  char c = 'A';

  while(c != 'S' && c != 's')
  {
	  
	  std::cout << "Escolha a op��o:"  << std::endl 
		  << "1. (M) Mostrar Posicao;" << std::endl
		  << "2. (P) Setar a Posi��o Inicial;" << std::endl
		  << "3. (V) Setar Velocidades;" << std::endl
		  << "4. (T) Modo teleopera��o;" << std::endl
		  << "5. (S) Sair;"            << std::endl;

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
		case 's':
		case 'S':
			break;
		default:
			std::cout << "Ops... nao encontrei a opcao. Voce sabe ler instrucoes, seu macaco?" << std::endl;

	  }

	 

  }

  std::cout << "Ciao!";
*/
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


