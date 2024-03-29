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
#include "Aria.h"


/** @example moduleExample.cpp Example demonstrating how to load an ArModule
 *
  This is a simple program that loads the module moduleExample_Mod, which is
  defined in moduleExample_Mod.cpp.   The
  program simply calls ArModuleLoader::load() with a short message
  argument and ArModuleLoader::reload with no argument and finally
  calls ArModuleLoader::close(). The return status of the load(),
  reload(), and close() are checked and printed out.
  ArModuleLoader uses the name of the module file without the platform-specific
  suffix (i.e. ".dll" on Windows and ".so" on Linux) to load the module.

  @sa moduleExample_Mod.cpp.
  @sa ArModuleLoader in the reference manual.  
*/


void printStatus(ArModuleLoader::Status status)
{
  if (status == ArModuleLoader::STATUS_ALREADY_LOADED)
    ArLog::log(ArLog::Terse, "moduleExample: Module already loaded.");
  else if (status == ArModuleLoader::STATUS_FAILED_OPEN)
    ArLog::log(ArLog::Terse, "moduleExample: Failed to find or open the simpleMod module.");
  else if (status == ArModuleLoader::STATUS_INVALID)
    ArLog::log(ArLog::Terse, "moduleExample: Invalid file.");
  else if (status == ArModuleLoader::STATUS_INIT_FAILED)
    ArLog::log(ArLog::Terse, "moduleExample: Module Init failed.");
  else if (status == ArModuleLoader::STATUS_SUCCESS)
    ArLog::log(ArLog::Terse, "moduleExample: Module succedded.");
  else if (status == ArModuleLoader::STATUS_EXIT_FAILED)
    ArLog::log(ArLog::Terse, "moduleExample: Module exit sequence failed.");
  else if (status == ArModuleLoader::STATUS_NOT_FOUND)
    ArLog::log(ArLog::Terse, "moduleExample: Module not found.");
  else
    ArLog::log(ArLog::Terse, "moduleExample: Module returned unknown status!");
  ArLog::log(ArLog::Terse, "");
}

int main(int argc, char **argv)
{

  Aria::init();

  ArArgumentParser parser(&argc, argv);
  // set up our simple connector
  ArSimpleConnector simpleConnector(&parser);  
  ArRobot robot;

  // set up the robot for connecting
  if (!simpleConnector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::exit(1);
  }

  robot.runAsync(true);

  ArModuleLoader::Status status;
  std::string str;

  ArLog::log(ArLog::Terse, "moduleExample: Loading the module \"moduleExample_Mod\" with a string argument...");
  status=ArModuleLoader::load("./moduleExample_Mod", &robot, (char *)"You've loaded a module!");
  printStatus(status);
  //ArLog::log(ArLog::Terse, "moduleExample: Reloading the module with no argument...");
  //status=ArModuleLoader::reload("./moduleExample_Mod", &robot);
  //printStatus(status);

  //ArLog::log(ArLog::Terse, "moduleExample: Closing the module...");
  //status=ArModuleLoader::close("./moduleExample_Mod");
  //printStatus(status);

  ArUtil::sleep(3000);

  Aria::shutdown();
  return(0);
}
