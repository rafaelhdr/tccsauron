#pragma once
#include "CustomTypes.h"
#include "Pose.h"
#include "MathHelper.h"

namespace sauron
{
	namespace configs
	{
		namespace sonars
		{
			const int circularBufferLength = 10;//15;
			const reading_t sonarReadingStandardDeviationMm =  15;
			const int validationGateSigma2 = 400; //625;
			const int kMin = 4;
			const double phoErrorFront4mm = 0.05;
			const double readingValidationAlpha = 0.5;
			const double wallRejectionValue2 = 15;//2.7060;
			const double maximumSonarToLineDistance = 500;//350;
			// variação mínima em radianos para que o sistema considere que o robô fez
			// uma curva
			const double minimumRobotTurnAngle =  0.174;//0.05; //0.174; // 10 graus
			// valor de "não achei nada" do sonar
			const double invalidReading = 500; // (cm)
			// abertura do cone do sonar
			const double sonarApertureAngleRads = 0.48; // 27.5 graus
			// distância mínima que deve ser percorrida pelo robô para que uma leitura
			// seja considerada interessante
			const double minimumRobotDistance = 5;
			inline Pose getSonarPose(int sonarNumber) {
				switch(sonarNumber) 
				{
				case 0:
					return sauron::Pose(11.5, 13, trigonometry::degrees2rads(90));
				case 1:
					return sauron::Pose(15.5, 11.5, trigonometry::degrees2rads(50));
				case 2:
					return sauron::Pose(19, 8, trigonometry::degrees2rads(30));
				case 3:
					return sauron::Pose(21, 2.5, trigonometry::degrees2rads(10));
				case 4:
					return sauron::Pose(21, -2.5, trigonometry::degrees2rads(-10));
				case 5:
					return sauron::Pose(19, -8, trigonometry::degrees2rads(-30));
				case 6:
					return sauron::Pose(15.5, -11.5, trigonometry::degrees2rads(-50));
				case 7:
					return sauron::Pose(11.5, -13, trigonometry::degrees2rads(-90));
				default:
					throw std::invalid_argument("Nao existe esse sonar");
				}
			}
		}
	}
}