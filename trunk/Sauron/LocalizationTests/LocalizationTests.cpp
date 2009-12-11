#include "stdafx.h"
#include <stdexcept>
#include "TestInfrastructure/SauronArRobot.h"
#include "TestInfrastructure/SonarMatchServer.h"
#include "TestInfrastructure/LocalizationMonitorConsole.h"
#include "TestInfrastructure/NavigationMonitorConsole.h"
#include "Localization/MapManager.h"

#include "Navigation/PathPlanner.h"
#include "Navigation/MapPlanner.h"


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
sauron::SauronArRobot* pRobot;

sauron::Pose lastEstimatedPose;
sauron::Pose lastTruePose;

sauron::LocalizationManager* plocManager;

void printEstimatedPose(const sauron::Pose& currentPose)
{
	using namespace sauron;

	//pRobot->lock();
	
	//pRobot->unlock();

    ArPose arPose = pRobot->getTruePose();
	Pose truePose(arPose.getX(), arPose.getY(), sauron::trigonometry::degrees2rads(arPose.getTh()));
	
	double erroX = currentPose.X() - truePose.X();
	double erroY = currentPose.Y() - truePose.Y();
	double erroTheta = currentPose.Theta() - truePose.Theta();

	grafico << getTimeMs() / 1000.0 << ";" << erroX << ";" << erroY << ";" << erroTheta << ";" << currentPose.X() << ";" << currentPose.Y() << ";" << currentPose.Theta() << ";" <<
		truePose.X() << ";" << truePose.Y() << ";" << truePose.Theta() << ";" << truePose.getDistance(currentPose) << std::endl;
	grafico.flush();

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

ArServerHandlerMap* p_mapHandler;
void changeMap(sauron::MapPlannerStatus status, const sauron::Map* map)
{
	if(status == sauron::MAP_CHANGED)
	{
		p_mapHandler->loadMap(map->getOriginalMapFilename().c_str());
	}
}


void testNavigation(ArMap& arMap, sauron::LocalizationManager& locManager, sauron::MapPlanner& planner) {
	
	
	
	sauron::tests::NavigationMonitorConsole monitor(&planner.getPathPlanner(), &planner);
	while(true)
	{
		std::string goalName;
		std::cout << "Digite o nome de seu destino: ";
		std::cin >> goalName;
		if(planner.goTo(goalName)) {
			std::cout << "Chegamos em " << goalName << "!" << std::endl;
			return;
		} else {
			std::cout << "Hoje nao vai dar errado, hoje nao, hoje nao, hoje nao... Hoje sim.. hoje sim?" << std::endl;
			return;
		}
		std::cout << std::endl;
	}
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
  std::string mapName = "pavsup_mod.map";
  //std::string mapName = "corredorfake.map";
  if(!map.readFile(mapName.c_str())) {
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
  p_mapHandler = &serverMap;
  server.runAsync();
  printf("Server is now running...\n");
  clientSwitchManager.runAsync();
#pragma endregion
  std::vector<std::string> names;
  //names.push_back("corredorfake.map");
  //sauron::MapManager mapManager("corredorfake.map", names);
  names.push_back("mezanino.map");
  names.push_back("pavsup_mod.map");
  names.push_back("pavinf.map");

  sauron::MapManager mapManager("pavsup_mod.map", names);
  /*names.push_back("horizontal.map");
  names.push_back("vertical.map");
  sauron::MapManager mapManager("horizontal.map", names);*/
  sauron::MapPlanner planner(&robot, &mapManager); 
  sauron::LocalizationManager locManager(&robot, mapManager, std::string(""));
  planner.setLocalizationManager(&locManager);
  planner.addMapPlannerCallback(changeMap);

  plocManager = &locManager;

  sauron::tests::LocalizationMonitorConsole locConsole(&locManager, &robot);
  sauron::tests::SonarMatchServer sonarMatches(&drawings, &locManager);
  

  std::cout	<< "Bem-vindo ao programa de testes mais bonito do Brasil" << std::endl 
			<< "Você está usando o mapa: " << mapName << std::endl
			<< "Digite a letra referente a opcao desejada:" << std::endl;

  char c = 'A';

  while(c != 'Q' && c != 'q')
  {
	  
	  std::cout << "Escolha a opção:"  << std::endl 
		  << "1. (M) Mostrar Posicao;" << std::endl
		  << "2. (R) Setar a Posição Inicial por Marco;" << std::endl
  		  << "3. (P) Setar a Posição Inicial por Coordenadas;" << std::endl
	  	  << "4. (I) Setar Mapa Inicial" << std::endl
		  << "5. (S) Pegar a Posicao do Simulador;" << std::endl
		  << "6. (V) Setar Velocidades;" << std::endl
		  << "7. (T) Modo teleoperação;" << std::endl
		  << "8. (N) Navegacao automatica" << std::endl;


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
		case 'r':
		case 'R':
			{
			std::cout << "Nome do marco e ângulo: ";
			double theta;
			std::string markName;
			std::cin >> markName;
			std::cin >> theta;
			
			sauron::Graph& graph = planner.getGraph(mapManager.getCurrentMap());
			
			sauron::Graph::iterator it;

			for(it = graph.begin(); it != graph.end(); it++)
			{
				if((*it).getName() == markName)
				{
					locManager.setInitialPose(sauron::Pose((*it).getPosition().X(), (*it).getPosition().Y(), theta));
				}
			}
			
			break;
			}
		case 'i':
		case 'I':
			{
				std::string mapName;
				std::cin >> mapName;
				std::vector<sauron::Map*> maps = planner.getMaps();
				std::vector<sauron::Map*>::iterator mapIt;
				bool alterou = false;
				for(mapIt = maps.begin(); mapIt != maps.end(); mapIt++)
				{
					if((*mapIt)->getOriginalMapFilename() == mapName)
					{
						alterou = true;
						mapManager.setCurrentMap(*mapIt);
						p_mapHandler->loadMap((*mapIt)->getOriginalMapFilename().c_str());
						std::cout << "Mapa alterado com sucesso!"  << std::endl;
						break;
					}
				}

				if(!alterou)
				{
					std::cout << "Mapa nao encontrado!"  << std::endl;
				}
				break;
			}
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
		case 'n':
		case 'N':
			testNavigation(map, locManager, planner);
			break;
		case 't':
		case 'T':
			robot.waitForRunExit();
			break;
		default:
			std::cout << "Ops... nao encontrei a opcao. Voce sabe ler instrucoes, seu macaco?" << std::endl;
			break;
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