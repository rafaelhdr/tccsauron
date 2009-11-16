#pragma once
#include "ConsoleLogger.h"
namespace sauron
{
class Pose;
class LocalizationManager;
class SauronArRobot;

namespace tests
{
class LocalizationMonitorConsole
{
public:
	LocalizationMonitorConsole(LocalizationManager* locManager, SauronArRobot* robot);
	~LocalizationMonitorConsole(void);

private:
	CConsoleLogger m_console;
	LocalizationManager* mp_localization;
	SauronArRobot* mp_robot;
	int m_callbackId;

	void printFunction(const Pose& pose);
	void cls(int n);
};
}
}