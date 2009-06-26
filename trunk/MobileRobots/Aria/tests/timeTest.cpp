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

int main(void)
{
  printf("\nTesting platform localtime (broken-down) struct:\n");
  //struct tm t;
  //ArUtil::localtime(&t);

  struct tm t;
  ArUtil::localtime(&t);
  printf("ArUtil::localtime() returned: year=%d mon=%d mday=%d hour=%d min=%d sec=%d\n",
    t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  time_t yesterday = time(NULL) - (24*60*60) ;
  ArUtil::localtime(&yesterday, &t);
  printf("ArUtil::localtime(time(NULL) - 24hours, struct tm*) returned: year=%d mon=%d mday=%d hour=%d min=%d sec=%d\n",
    t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);

  printf("\nCurrent time as strings:\n");
  char year[5], month[3], day[3], hour[3], min[3], sec[3];
  ArUtil::putCurrentYearInString(year, 5);
  ArUtil::putCurrentMonthInString(month, 3);
  ArUtil::putCurrentDayInString(day, 3);
  ArUtil::putCurrentHourInString(hour, 3);
  ArUtil::putCurrentMinuteInString(min, 3);
  ArUtil::putCurrentSecondInString(sec, 3);
  printf("  Year:%s, Month:%s, Day:%s, Hour:%s, Min:%s, Sec:%s\n",
      year, month, day, hour, min, sec);

  printf("\nTesting ArTime:\n");
  ArTime start, test;
  
  printf("Setting an ArTime object \"start\" to now...\n");
  start.setToNow();
  start.log();
  printf("Sleeping 4 secs\n");
  ArUtil::sleep(4000);
  printf("Setting an ArTime object \"test\" to now...\n");
  test.setToNow();
  test.log();

  printf("ms of \"test\" since start %ld\n", test.mSecSince(start));
  printf("seconds \"test\" since start %ld\n", test.secSince(start));
  printf("ms of start since \"test\" %ld\n", start.mSecSince(test));
  printf("seconds \"start\" test %ld\n", start.secSince(test));
  printf("\"start\" is before \"test\"? %d\n", test.isBefore(start));
  printf("\"start\" is after \"test\"? %d\n", test.isAfter(start));
  printf("\"test\" is before \"start\"? %d\n", start.isBefore(test));
  printf("\"test\" is after \"start\"? %d\n", start.isAfter(test));
  printf("ms from \"start\" to now %ld\n", start.mSecTo());
  printf("s from \"start\" to now %ld\n", start.secTo());
  printf("ms since \"start\" %ld\n", start.mSecSince());
  printf("s since \"start\" %ld\n", start.secSince());
  printf("ms from \"test\" stamp to now %ld\n", test.mSecTo());
  printf("s from \"test\" stamp to now %ld\n", test.secTo());

  printf("Testing addMSec, adding 200 mSec\n");
  test.addMSec(200);
  printf("ms from \"test\" stamp to now %ld\n", test.mSecTo());
  printf("Testing addMSec, subtracting 300 mSec\n");
  test.addMSec(-300);
  printf("ms from \"test\" stamp to now %ld\n", test.mSecTo());
  printf("Testing addMSec, adding 20.999 seconds\n");
  test.addMSec(20999);
  printf("ms from \"test\" stamp to now %ld\n", test.mSecTo());
  printf("Testing addMSec, subtracting 23.5 seconds\n");
  test.addMSec(-23500);
  printf("ms from \"test\" stamp to now %ld\n", test.mSecTo());
  
  ArTime timeDone;
  printf("Setting ArTime object \"done\" to now.\n");
  timeDone.setToNow();
  timeDone.addMSec(1000);
  printf("Making sure the add works in the right direction, adding a second to a timestamp set now\n");
  printf("Reading: %ld\n", timeDone.mSecTo());
  printf("Sleeping 20 ms\n");
  ArUtil::sleep(20);
  printf("Reading: %ld\n", timeDone.mSecTo());
  printf("Sleeping 2 seconds\n");
  ArUtil::sleep(2000);
  printf("Reading: %ld\n", timeDone.mSecTo());

  puts("\nslamming ArUtil::localtime() from a bunch of threads with the same input time...");
  time_t now = time(NULL);
  class LocaltimeTestThread : public virtual ArASyncTask 
  {
  private:
    time_t time;
  public:
    LocaltimeTestThread(time_t t) : time(t) {}
    virtual void *runThread(void *) {
      struct tm t;
      ArUtil::localtime(&time, &t);
      printf("ArUtil::localtime() returned: year=%d mon=%d mday=%d hour=%d min=%d sec=%d\n", t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      return 0;
    }
  };
  for(int i = 0; i < 200; ++i)
    (new LocaltimeTestThread(now))->runAsync();

  ArUtil::sleep(5000);

  printf("test is done.\n");

}
