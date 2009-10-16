#pragma once
#include "SonarReadingsLogParser.h"
#include "ArMap.h"
#include "Line.h"
namespace SonarTestHelper {
/* alpha = �ngulo de incid�ncia da trajet�ria do rob� */
void assertExpectedReading(ArMap& map, const SonarReadingsLogParser& parser,
						   int sonarNumber, const sauron::Line& seenLine, double alpha_rads);
}