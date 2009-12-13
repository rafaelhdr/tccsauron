#include "stdafx.h"
#include <stdexcept>
#include "TestInfrastructure/SauronArRobot.h"
#include "TestInfrastructure/SonarMatchServer.h"
#include "TestInfrastructure/LocalizationMonitorConsole.h"
#include "TestInfrastructure/NavigationMonitorConsole.h"
#include "Localization/MapManager.h"


#include "Navigation/PathPlanner.h"
#include "Navigation/MapPlanner.h"

#include "Sauron.h"
#include "CommandServer.h"
#include "StatusServer.h"

#define TEST_LOG(level) FILE_LOG(level) << "LocalizationTests: "
#include <windows.h>
#include <fstream>




ArServerHandlerMap* p_mapHandler;
void changeMap(sauron::MapPlannerStatus status, const sauron::Map* map)
{
	if(status == sauron::MAP_CHANGED)
	{
		p_mapHandler->loadMap(map->getOriginalMapFilename().c_str());
	}
}


void testNavigation(sauron::Sauron& sauronzito) {
	
	sauron::tests::NavigationMonitorConsole monitor(
		&(sauronzito.getMapPlanner()->getPathPlanner()),
		sauronzito.getMapPlanner());
	while(true)
	{
		std::string goalName;
		std::cout << "Digite o nome de seu destino: ";
		std::cin >> goalName;
		if(sauronzito.goTo(goalName)) {
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




//  Provide the map to the client (and related controls):
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

  sauron::Sauron sauronzito(&robot, names, "pavsup_mod.map");

  sauron::StatusServer  statusServer( sauronzito );
  sauron::CommandServer commandServer( sauronzito );  

  statusServer.start();
  commandServer.start();

  /*names.push_back("horizontal.map");
  names.push_back("vertical.map");
  sauron::MapManager mapManager("horizontal.map", names);*/

  sauronzito.getMapPlanner()->addMapPlannerCallback(changeMap);

  sauron::tests::LocalizationMonitorConsole locConsole(sauronzito.getLocalizationManager(), &robot);
  sauron::tests::SonarMatchServer sonarMatches(&drawings, sauronzito.getLocalizationManager());
  

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
			sauron::Pose pose = sauronzito.getPose();
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
			sauronzito.setPose(sauron::Pose(x, y, th));
			break;
		case 'r':
		case 'R':
			{
			std::cout << "Nome do marco e ângulo: ";
			double theta;
			std::string markName;
			std::cin >> markName;
			std::cin >> theta;
			sauronzito.setPose(markName, theta);
			break;
			}
		case 'i':
		case 'I':
			{
				std::string mapName;
				std::cin >> mapName;
				serverMap.loadMap(mapName.c_str());
				if(sauronzito.setInitialMap(mapName))
					std::cout << "Mapa alterado com sucesso!"  << std::endl;
				else
					std::cout << "Mapa nao encontrado!"  << std::endl;
				break;
			}
		case 's':
		case 'S':
			sauronzito.setPose(sauron::Pose(arPose.getX(), arPose.getY(),
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
			testNavigation(sauronzito);
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
		FILELog::ReportingLevel() = logDEBUG1;

		return principal(argc, argv);
	} catch(std::exception& e)
	{
		std::cerr << "std::exception: " << e.what() << std::endl;
		return -1;
	}
}