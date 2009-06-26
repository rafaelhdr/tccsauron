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

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArGPSConnector.h"
#include "ArGPS.h"
#include "ArDeviceConnection.h"
#include "ArSerialConnection.h"
#include "ArTcpConnection.h"
#include "ArRobot.h"
#include "ariaInternal.h"

#include "ArNovatelGPS.h"
#include "ArTrimbleGPS.h"

#include <iostream>


const int ARGPS_DEFAULT_SERIAL_BAUD = 9600;
const char* const ARGPS_DEFAULT_SERIAL_PORT = ArUtil::COM2;
const int ARGPS_DEFAULT_REMOTE_TCP_PORT = 8103;

AREXPORT ArGPSConnector::ArGPSConnector(ArArgumentParser *argParser) :
  myDeviceCon(NULL),
  myArgParser(argParser),
  myParseArgsCallback(this, &ArGPSConnector::parseArgs),
  myLogArgsCallback(this, &ArGPSConnector::logOptions),
  myBaud(-1), 
  myPort(NULL),
  myTCPHost(NULL),
  myTCPPort(8103),
  myDeviceType(Invalid)
{
  myParseArgsCallback.setName("ArGPSConnector");
  myLogArgsCallback.setName("ArGPSConnector");
  Aria::addParseArgsCB(&myParseArgsCallback);
  Aria::addLogOptionsCB(&myLogArgsCallback);
}


AREXPORT ArGPSConnector::~ArGPSConnector()
{
  if(myDeviceCon) delete myDeviceCon;
}


AREXPORT bool ArGPSConnector::parseArgs() 
{
  if (!myArgParser) return false;
  if (!myArgParser->checkParameterArgumentString("-gpsPort", &myPort)) return false;
  if (!myArgParser->checkParameterArgumentInteger("-gpsBaud", &myBaud)) return false;
  if (!myArgParser->checkParameterArgumentString("-remoteGpsTcpHost", &myTCPHost)) return false;
  if (!myArgParser->checkParameterArgumentInteger("-remoteGpsTcpPort", &myTCPPort)) return false;
  char *deviceType = myArgParser->checkParameterArgument("-gpsType");
  if (deviceType) // if -gpsType was given
    myDeviceType = deviceTypeFromString(deviceType);
  return true;
}

ArGPSConnector::GPSType ArGPSConnector::deviceTypeFromString(const char *str)
{
  if (strcasecmp(str, "novatel") == 0)
  {
    return Novatel;
  }
  else if (strcasecmp(str, "trimble") == 0)
  {
    return Trimble;
  }
  else if (strcasecmp(str, "standard") == 0)
  {
    return Standard;
  }
  else
  {
    ArLog::log(ArLog::Terse, "GPSConnector: Error: unrecognized GPS type.");
    return Invalid;
  }
}

void ArGPSConnector::logOptions()
{
  ArLog::log(ArLog::Terse, "GPS options:"); 
  ArLog::log(ArLog::Terse, "-gpsType <standard|novatel|trimble>\tSelect GPS device type (default: standard)");
  ArLog::log(ArLog::Terse, "-gpsPort <gpsSerialPort>\tUse the given serial port (default: %s)", ARGPS_DEFAULT_SERIAL_PORT);
  ArLog::log(ArLog::Terse, "-gpsBaud <gpsSerialBaudRate>\tUse the given serial Baud rate (default: %d)", ARGPS_DEFAULT_SERIAL_BAUD);
  ArLog::log(ArLog::Terse, "-remoteGpsTcpHost <host>\tUse a TCP connection instead of serial, and connect to remote host <host>");
  ArLog::log(ArLog::Terse, "-remoteGpsTcpPort <host>\tUse the given port number for TCP connection, if using TCP. (default %d)", ARGPS_DEFAULT_REMOTE_TCP_PORT);
}



AREXPORT ArGPS* ArGPSConnector::createGPS(ArRobot *robot)
{
  // If we have a robot with params (have connected), use param values if no
  // command-line argument was given, else use defaults:
  if(robot && robot->getRobotParams())
  {
    if(myPort == NULL) {
      myPort = robot->getRobotParams()->getGPSPort();
      if(strcmp(myPort, "COM1") == 0)
        myPort = ArUtil::COM1;
      if(strcmp(myPort, "COM2") == 0)
        myPort = ArUtil::COM2;
      if(strcmp(myPort, "COM3") == 0)
        myPort = ArUtil::COM3;
      if(strcmp(myPort, "COM4") == 0)
        myPort = ArUtil::COM4;
    }
    if(myBaud == -1) {
      myBaud = robot->getRobotParams()->getGPSBaud();
    }
    if(myDeviceType == Invalid) {
      myDeviceType = deviceTypeFromString(robot->getRobotParams()->getGPSType());
    }
  }
  else
  {
    if(myPort == NULL) myPort = ARGPS_DEFAULT_SERIAL_PORT;
    if(myBaud == -1) myBaud = ARGPS_DEFAULT_SERIAL_BAUD;
    if(myDeviceType == Invalid) myDeviceType = Standard;
  }

  // Create gps:
  ArGPS* newGPS = NULL;
  switch (myDeviceType)
  {
    case Novatel:
      ArLog::log(ArLog::Normal, "ArGPSConnector: Using Novatel GPS");
      newGPS = new ArNovatelGPS;
      break;
    case Trimble:
      ArLog::log(ArLog::Normal, "ArGPSConnector: Using Trimble GPS");
      newGPS = new ArTrimbleGPS;
      break;
    default:
      ArLog::log(ArLog::Normal, "ArGPSConnector: Using standard NMEA GPS");
      newGPS = new ArGPS;
      break;
  }

  if (myTCPHost == NULL)
  {
    // Setup serial connection
    ArSerialConnection *serialCon = new ArSerialConnection;
    ArLog::log(ArLog::Normal, "ArGPSConnector: Connecting to GPS on port %s at %d baud...", myPort, myBaud);
    if (!serialCon->setBaud(myBaud)) { delete serialCon; return false; }
    if (serialCon->open(myPort) != 0) {
      ArLog::log(ArLog::Terse, "ArGPSConnector: Error: could not open GPS serial port %s.", myPort);
      delete serialCon;
      return NULL;
    }
    newGPS->setDeviceConnection(serialCon);
    myDeviceCon = serialCon;
  }
  else
  {
    // Setup TCP connection
    ArTcpConnection *tcpCon = new ArTcpConnection;
    ArLog::log(ArLog::Normal, "ArGPSConnector: Opening TCP connection to %s:%d...", myTCPHost, myTCPPort);
    int openState = tcpCon->open(myTCPHost, myTCPPort);
    if (openState != 0) {
      ArLog::log(ArLog::Terse, "ArGPSConnector: Error: could not open TCP connection to %s port %d: %s", tcpCon->getOpenMessage(openState));
      delete tcpCon;
      return NULL;
    }
    newGPS->setDeviceConnection(tcpCon);
    myDeviceCon = tcpCon;
  }

  return newGPS;
}

#if 0  
//doesn't really do anything :
AREXPORT bool ArGPSConnector::connectGPS(ArGPS *gps)
{
  // TODO try different BAUD rates if using a serial connection.
  return gps->blockingConnect();
}
#endif

