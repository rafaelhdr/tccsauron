#include "stdafx.h"
#include "ConsoleLogger.h"
#include <stdexcept>

void printEstimatedPoseLoop(sauron::LocalizationManager* plocManager, CConsoleLogger& console)
{
	int i = 0;
	while(true)
	{
		sauron::Pose currentPose = plocManager->getPose();
		console.printf("%.3f %.3f %.3f", currentPose.X(), currentPose.Y(), currentPose.Theta());
		console.print("\r");
		::Sleep(300);
	}
}

void startEstimatedPoseConsole(sauron::LocalizationManager& locManager)
{
	CConsoleLogger console;
	console.Create("Posicao atual");
	boost::thread printThread(&printEstimatedPoseLoop, &locManager, console);
}

int principal(int argc, char** argv)
{

#pragma region boiler1
  // Initialize some global data
  Aria::init();
  ArLog::init(ArLog::StdErr, ArLog::Terse);

  // If you want ArLog to print "Verbose" level messages uncomment this:
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // This object parses program options from the command line
  ArArgumentParser parser(&argc, argv);

  // Load some default values for command line arguments from /etc/Aria.args
  // (Linux) or the ARIAARGS environment variable.
  parser.loadDefaultArguments();

  // Central object that is an interface to the robot and its integrated
  // devices, and which manages control of the robot by the rest of the program.
  ArRobot robot;

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
  if(!map.readFile("pavsup.map")) {
	  robot.disconnect(); // sem isso dá pau (pure virtual call)
	  throw std::invalid_argument(std::string("Mapa nao foi encontrado"));
  }


  FILE_LOG(logINFO) << "ois";
  sauron::LocalizationManager locManager(&robot, map, std::string(""));
  locManager.startAsync();

  startEstimatedPoseConsole(locManager);

std::cout << "Bem-vindo ao programa de testes mais bonito do Brasil" << std::endl 
          << "Digite a letra referente a opcao desejada:" << std::endl;

  char c = 'A';

  while(c != 'S' && c != 's')
  {
	  
	  std::cout << "Escolha a opção:"  << std::endl 
		  << "1. (M) Mostrar Posicao;" << std::endl
		  << "2. (P) Setar a Posição Inicial;" << std::endl
		  << "3. (V) Setar Velocidades;" << std::endl
		  << "4. (T) Modo teleoperação;" << std::endl
		  << "5. (S) Sair;"            << std::endl;

	  std::cin >> c;
		
	  
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
			std::cout << "Digite na ordem: x y theta e aperte ENTER:"  << std::endl; 
			double x;
			double y;
			double theta;

			std::cin >> x;
			std::cin >> y;
			std::cin >> theta;
			locManager.setInitialPose(sauron::Pose(x, y, theta));
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

  // Block execution of the main thread here and wait for the robot's task loop
  // thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
  // signal)
  //robot.waitForRunExit();

  Aria::exit(0);
  return 0;
}

int main(int argc, char** argv)
{
	try
	{
		Output2FILE::Stream() = fopen("sonar_log.log", "w");
		FILELog::ReportingLevel() = logDEBUG4;

		return principal(argc, argv);
	} catch(std::exception& e)
	{
		std::cerr << "std::exception: " << e.what() << std::endl;
		return -1;
	}
}



