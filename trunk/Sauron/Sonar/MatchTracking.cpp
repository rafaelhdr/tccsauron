#include "MatchTracking.h"

namespace sauron
{

void MatchTracking::setMatch(const LineSegment &matchedSegment, double expectedReading,
							 double actualReading)
{
	m_segment = matchedSegment;
	m_lastError = actualReading - expectedReading;
	m_lastError *= m_lastError;
}


void MatchTracking::reset()
{
	m_segment = LineSegment();
	m_lastError = 0;
	m_score = 50;
}

void MatchTracking::updateMatch(double expectedReading, double actualReading)
{
	double error = actualReading - expectedReading;
	error = error * error;

	double deltaImprovement = m_lastError - error;
	updateScore(deltaImprovement);
}

void MatchTracking::updateScore(double deltaImprovement)
{
	if(deltaImprovement > 0) {
		m_score += K_improvement * deltaImprovement;
	} else {
		m_score += K_worsening * deltaImprovement;
	}
}

bool MatchTracking::isMatchValid()
{
	return m_score < 0;
}

void setMatch(const LineSegment& matchedSegment, double expectedReading,
		double actualReading);
	void updateMatch(double expectedReading, double actualReading);
	bool isMatchValid();
	double getScore();

}