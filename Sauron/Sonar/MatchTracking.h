#pragma once

#include "LineSegment.h"

namespace sauron {
class MatchTracking
{
public:
	MatchTracking(int sonarNumber) :
	  m_sonarNumber(sonarNumber) { }
	void setMatch(const LineSegment& matchedSegment, double expectedReading,
		double actualReading);
	void updateMatch(double expectedReading, double actualReading);
	bool isMatchValid();
	inline double getScore() { return m_score; }
	void reset();

private:
	void updateScore(double deltaImprovement);
	int m_sonarNumber;
	static const int K_improvement = 3;
	static const int K_worsening = 7;
	LineSegment m_segment;
	double m_score;
	double m_lastError;
};
}
