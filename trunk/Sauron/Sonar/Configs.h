#pragma once
#include "CustomTypes.h"
#include "Pose.h"
#include "MathHelper.h"

namespace sauron
{
	namespace configs
	{
		const reading_t sonarReadingStandardDeviationMm =  25;
		const int kMin = 4;
		const double phoErrorFront4mm = 0.05;
		const double readingValidationAlpha = 0.5;

		namespace sonars
		{
			// distância mínima que deve ser percorrida pelo robô para que uma leitura
			// seja considerada interessante
			const double minimalRobotDistance = 5;
			inline Pose getSonarPose(int sonarNumber) {
				switch(sonarNumber) 
				{
				case 0:
					return sauron::Pose(115, 130, trigonometry::degrees2rads(90));
				case 1:
					return sauron::Pose(155, 115, trigonometry::degrees2rads(50));
				case 2:
					return sauron::Pose(190, 80, trigonometry::degrees2rads(30));
				case 3:
					return sauron::Pose(210, 25, trigonometry::degrees2rads(10));
				case 4:
					return sauron::Pose(210, -25, trigonometry::degrees2rads(-10));
				case 5:
					return sauron::Pose(190, -80, trigonometry::degrees2rads(-30));
				case 6:
					return sauron::Pose(155, -115, trigonometry::degrees2rads(-50));
				case 7:
					return sauron::Pose(115, -130, trigonometry::degrees2rads(-90));
				default:
					throw std::invalid_argument("Nao existe esse sonar");
				}
			}
		}
	}
}