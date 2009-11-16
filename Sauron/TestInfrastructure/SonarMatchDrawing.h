#pragma once
#include "ArDrawingData.h"
#include <boost/thread.hpp>

namespace sauron {
class SonarMatchDrawing
{
public:
	SonarMatchDrawing(ArServerInfoDrawings* server, int sonarNumber);
	~SonarMatchDrawing();
	void setSegment(LineSegment segment);

private:
	void callback(ArServerClient* client, ArNetPacket* requestPkt);
	ArFunctor2C<SonarMatchDrawing, ArServerClient*, ArNetPacket*> m_functor;

	ArDrawingData m_drawingData;
	ArServerInfoDrawings* mp_server;
	boost::mutex m_segmentMutex;
	LineSegment m_segment;
};
}

