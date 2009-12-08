#pragma once
#include "stdafx.h"
#include <vector>
#include <map>
#include "Localization/SonarMatch.h"
#include "LineSegment.h"
#include "ArDrawingData.h"
#include "SonarMatchDrawing.h"
namespace sauron
{
class SonarMatchPainter
{
public:
		SonarMatchPainter(ArServerInfoDrawings* serverDrawings);
	  SonarMatchPainter::~SonarMatchPainter();
	  void update(const std::vector<SonarMatch>& sonarMatches);

private:
	SonarMatchPainter(SonarMatchPainter& copy);
	LineSegment splitSegment(const std::vector<SonarMatch>&, LineSegment& original, int sonar);
	void callback(ArServerClient* client, ArNetPacket* requestPkt);
	ArFunctor2C<SonarMatchDrawing, ArServerClient*, ArNetPacket*> m_functor;

	ArServerInfoDrawings* mp_serverDrawings;
	std::map<int, SonarMatchDrawing*> m_drawings;
};
}
