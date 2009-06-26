/*
MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.7.1

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

All Rights Reserved.

MobileRobots Inc does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
MobileRobots
10 Columbia Drive
Amherst, NH 03031
800-639-9481

*/

#ifndef ARNL_H
#define ARNL_H

#include "Aria.h"
#include "ArNetworking.h"
#include "arnlInternal.h"
#include "ArPathPlanningTask.h"
#include "ArServerClasses.h"
#include "ArLocalizationManager.h"
#include "ArMultiRobot.h"
#include "ArMultiRobotPeer.h"
#include "ArCentralMultiRobot.h"

#ifdef ARNL
#include "ArLocalizationTask.h"
#include "ArDocking.h"
#endif

#ifdef SONARNL
#include "ArSonarLocalizationTask.h"
#endif

#ifdef MOGS
#include "ArGPSLocalizationTask.h"
#endif

#endif

