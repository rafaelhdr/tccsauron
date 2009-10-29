#include "SonarTestHelper.h"
using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarTestHelper {
/* alpha = ângulo de incidência da trajetória do robô */
void assertExpectedReading(ArMap& map, const SonarReadingsLogParser& parser,
						   int sonarNumber, const sauron::Line& seenLine, double alpha_rads) {
	sauron::Pose sonarRelativePose = sauron::configs::sonars::getSonarPose(sonarNumber);
	sauron::SonarModel sonar(sonarRelativePose);
	parser.addAllReadingsOfOneSonar(sonarNumber, sonar);

	double beta_rads = sonarRelativePose.getTheta() + alpha_rads;
	sauron::Pose lastRobotPose = sonar.getSonarGlobalPose(parser.m_readings.back().pose);

	double expectedReading = (seenLine.getRWall() -
		lastRobotPose.X() * ::cos(seenLine.getTheta()) -
		lastRobotPose.Y() * ::sin(seenLine.getTheta())) /
		::sin(beta_rads);

	if(expectedReading < 0) expectedReading = -expectedReading;

	sauron::LineSegment matchedLineSegment;
	sauron::SonarReading actualExpectedReading(-1);
	sauron::SonarReading actualReading(-1);
	if(sonar.tryGetMatchingMapLine(sauron::Map(map), actualExpectedReading.getStdDeviationMm() * actualExpectedReading.getStdDeviationMm(),
		&matchedLineSegment, &actualExpectedReading, &actualReading)) {
			Assert::AreNotEqual(-1, actualReading.getReading());
			sauron::Line matchedLine = matchedLineSegment.getSauronLine();
			Assert::AreEqual(seenLine.getRWall(), matchedLine.getRWall(), 0.0001);
			Assert::AreEqual(seenLine.getTheta(), matchedLine.getTheta(), 0.0001);
			Assert::AreEqual(expectedReading, actualExpectedReading.getReading(),10);
	} else {
		Assert::Fail("Nenhum segmento foi encontrado");
	}
}
}