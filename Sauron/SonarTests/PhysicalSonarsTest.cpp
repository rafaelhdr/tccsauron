#include "stdafx.h"
#include "SonarTests.h"
#ifdef PHYSICAL_SONARS_TEST
void sonar1Callback(sauron::SonarReading reading) {
	std::cout << "SONAR1: " << reading.getReading() << std::endl;
}

void sonar6Callback(sauron::SonarReading reading) {
	std::cout << "SONAR6: " << reading.getReading() << std::endl;
}

int main(int argc, char** argv)
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
#pragma endregion
  
  sauron::PhysicalSonars sonars(&robot);
  ArGlobalFunctor1<sauron::SonarReading> functorSonar1(&sonar1Callback);
  ArGlobalFunctor1<sauron::SonarReading> functorSonar6(&sonar6Callback);
  sonars.setAddReadingCallback(1, &functorSonar1);
  sonars.setAddReadingCallback(6, &functorSonar6);

#pragma region boiler2
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
  
  // Block execution of the main thread here and wait for the robot's task loop
  // thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
  // signal)
  robot.waitForRunExit();

  Aria::exit(0);


}
#endif