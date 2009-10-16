#pragma once
#include "SonarReadingsLogParser.h"
#include "ArMap.h"
#include "Line.h"
namespace SonarTestHelper {
/* alpha = ângulo de incidência da trajetória do robô */
void assertExpectedReading(ArMap& map, const SonarReadingsLogParser& parser,
						   int sonarNumber, const sauron::Line& seenLine, double alpha_rads);
}