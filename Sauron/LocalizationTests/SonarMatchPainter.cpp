#include "stdafx.h"
#include "SonarMatchPainter.h"

namespace sauron{

	SonarMatchPainter::SonarMatchPainter(ArServerInfoDrawings* serverDrawings) :
		mp_serverDrawings(serverDrawings)
		{
			for(int i = 0; i < 8; i++)
			{
				m_drawings[i] = new SonarMatchDrawing(mp_serverDrawings, i);
			}
		}

		SonarMatchPainter::~SonarMatchPainter()
		{
			for(std::map<int, SonarMatchDrawing*>::iterator it = m_drawings.begin(); it != m_drawings.end(); it++) {
				delete it->second;
			}
		}

	void SonarMatchPainter::update(const std::vector<SonarMatch>& sonarMatches) {
		std::map<int, int> hasBeenUpdated;

		for(std::vector<SonarMatch>::const_iterator it = sonarMatches.begin(); it != sonarMatches.end(); it++) {
			if(it->getSonarNumber() < m_drawings.size()) {
				m_drawings[it->getSonarNumber()]->setSegment(splitSegment(sonarMatches, it->getMatchedLine(),
					it->getSonarNumber()));
				hasBeenUpdated[it->getSonarNumber()] = 42;
			}
		}

		for(std::map<int, SonarMatchDrawing*>::iterator it = m_drawings.begin(); it != m_drawings.end(); it++) {
			if(hasBeenUpdated[it->first] != 42)
				it->second->setSegment(LineSegment(0,0,0,0));
		}
	}

	LineSegment SonarMatchPainter::splitSegment(const std::vector<SonarMatch>& matches, LineSegment& segment, int sonarNumber) {
		int nOthers = 0; // em quantas partições devermos partir o segmento
		int nOthersWhoHaveSmallerNumberThanMe = 0; // qual é a nossa partição
		for(std::vector<SonarMatch>::const_iterator it = matches.begin(); it != matches.end(); it++)
		{
			if(it->getSonarNumber() != sonarNumber) {
				LineSegment& currentSegment = it->getMatchedLine();
				if(floating_point::isEqual(currentSegment.getX1(), segment.getX1()) &&
					floating_point::isEqual(currentSegment.getX2(), segment.getX2()) &&
					floating_point::isEqual(currentSegment.getY1(), segment.getY1()) &&
					floating_point::isEqual(currentSegment.getY2(), segment.getY2())) {
						nOthers++;
						if(it->getSonarNumber() < sonarNumber)
							nOthersWhoHaveSmallerNumberThanMe++;
				}
			}
		}
		double deltaX, deltaY;
		deltaX = segment.getX2() - segment.getX1();
		deltaY = segment.getY2() - segment.getY1();
		double partitionSizeX = deltaX / ( nOthers + 1 );
		double partitionSizeY = deltaY / ( nOthers + 1 );
		double myDistanceX = nOthersWhoHaveSmallerNumberThanMe * partitionSizeX;
		double myDistanceY = nOthersWhoHaveSmallerNumberThanMe * partitionSizeY;

		double myX2 = segment.getX1() + myDistanceX + partitionSizeX;
		double myY2 = segment.getY1() + myDistanceY + partitionSizeY;
		return LineSegment(segment.getX1() + myDistanceX, segment.getY1() + myDistanceY, myX2, myY2);
	}

}
