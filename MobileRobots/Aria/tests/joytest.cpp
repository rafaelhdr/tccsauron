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

/*
  This program just outputs the values from the joystick
*/
int main(void)
{
  Aria::init();
  ArLog::init(ArLog::StdOut, ArLog::Verbose);
  ArJoyHandler joy;
  joy.init();
  unsigned int i;
  double x, y, z;
  if (!joy.haveJoystick())
  {
    printf("Did not definitely detect a joystick, it may not work.\n");
  }
  printf("Num. Buttons: %d\n", joy.getNumButtons());
  printf("Num. Axes: %d\n", joy.getNumAxes());
  if (joy.haveZAxis())
    printf("The Z axis (Axis 3, the throttle) works\n");
  else
    printf("The Z axis (Axis 3, the throttle) may or may not work (it'll show up below if it does)\n");
	 
  while (Aria::getRunning())
  {
    printf("\rButton ");
    for (i = 1; i <= joy.getNumButtons(); i++)
      printf(" %d:%d", i, joy.getButton(i));

    joy.getDoubles(&x, &y, &z);
    printf(" Axis x:%6.3f y:%6.3f", x, y);
    if (joy.haveZAxis())
      printf(" z:%6.3f", z);
    for (i = 4; i < joy.getNumAxes(); i++)
      printf(" %d:%6.3f", i, joy.getAxis(i));
    fflush(stdout);
    ArUtil::sleep(1);
  }

}
