#include "StdAfx.h"
#include "LocalizationMonitorConsole.h"

namespace sauron
{
namespace tests
{
LocalizationMonitorConsole::LocalizationMonitorConsole(LocalizationManager* locManager,
													   SauronArRobot* robot)
: mp_localization(locManager), mp_robot(robot)
{
	m_console.Create("LocalizationMonitor");
	m_callbackId = mp_localization->addPoseChangedCallback(
		boost::bind(&LocalizationMonitorConsole::printFunction, this, _1));
}

LocalizationMonitorConsole::~LocalizationMonitorConsole(void)
{
	mp_localization->removePoseChangedCallback(m_callbackId);
}

void LocalizationMonitorConsole::printFunction(const Pose& currentPose) {

	m_console.printf("%.3f\t%.3f\t%.3f\n", currentPose.X(), currentPose.Y(), currentPose.Theta());
	ArPose arPose = mp_robot->getTruePose();
	Pose truePose(arPose.getX(), arPose.getY(), sauron::trigonometry::degrees2rads(arPose.getTh()));
	m_console.printf("%.3f\t%.3f\t%.3f\n", truePose.X(), truePose.Y(), truePose.Theta());
	double erroX = currentPose.X() - truePose.X();
	double erroY = currentPose.Y() - truePose.Y();
	double erroTheta = currentPose.Theta() - truePose.Theta();
	m_console.printf("Erro em X = %.3f cm\n", erroX );
	m_console.printf("Erro em Y = %.3f cm\n", erroY );
	m_console.printf("Erro em Theta = %.3f radianos\n", erroTheta );
	m_console.printf("Covariancia do erro:");
	std::stringstream ss;
	ss << mp_localization->getPoseEstimateCovariance();
	m_console.print(ss.str().c_str());

	cls(16);
}
void LocalizationMonitorConsole::cls(int n)
{
	while(n-- > 0)
		m_console.print("\n");
}
}
}