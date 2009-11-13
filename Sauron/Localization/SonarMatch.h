#pragma once
#include "Sonar/LineSegment.h"

namespace sauron{
class SonarMatch
{
public:
	SonarMatch()
		: m_sonar(-1), m_segment(0, 0, 0, 0){}

	SonarMatch(int sonarNumber, LineSegment matchedLine) :
	  m_sonar(sonarNumber), m_segment(matchedLine)
	{
	}
	  int getSonarNumber() const { return m_sonar; }
	  LineSegment getMatchedLine() const  { return m_segment; }
private:
	int m_sonar;
	LineSegment m_segment;
};
}
