#include "StdAfx.h"
#include "SonarMatchDrawing.h"

namespace sauron
{
	std::string sorteiaNome() {
		static int n = 0;
		std::stringstream ss;
		ss << "associacao" << n++;
		return ss.str();
	}

	ArColor getColor(int sonarNumber) {
		switch(sonarNumber) {
			case 0:
				return ArColor(255, 0, 0); // vermelho
			case 1:
				return ArColor(0, 255, 0); // verde
			case 2:
				return ArColor(0, 0, 255); // azul
			case 3:
				return ArColor(255, 255, 0); // amarelo
			case 4:
				return ArColor(255, 0, 255); // rosa
			case 5:
				return ArColor(255, 106, 0); // laranja
			case 6:
				return ArColor(128, 128, 128); // cinza
			case 7:
				return ArColor(0, 255, 255); // cyan
			default:
				return ArColor(0,0,0);

		}
	}

	SonarMatchDrawing::SonarMatchDrawing(ArServerInfoDrawings* server, int sonarNumber) :
	mp_server(server),  m_functor(this, &SonarMatchDrawing::callback),
		m_drawingData("polySegments", getColor(sonarNumber), 3, 47)
	{
		mp_server->addDrawing(&m_drawingData, sorteiaNome().c_str(), &m_functor);
	}

	SonarMatchDrawing::~SonarMatchDrawing()
	{
		//mp_server->
	}

	void SonarMatchDrawing::setSegment(LineSegment segment)
	{
		boost::unique_lock<boost::mutex> lock(m_segmentMutex);
		m_segment = segment;
	}

	void SonarMatchDrawing::callback(ArServerClient* client, ArNetPacket* requestPkt)
	{
			ArNetPacket reply;
			// 1 segmento, 2 vértices:
			reply.byte4ToBuf(2);
			boost::unique_lock<boost::mutex> lock(m_segmentMutex);
			reply.byte4ToBuf(m_segment.getX1() * 10); // X1
			reply.byte4ToBuf(m_segment.getY1()* 10);   // Y1
			reply.byte4ToBuf(m_segment.getX2() * 10); // X2
			reply.byte4ToBuf(m_segment.getY2() * 10);  // Y2

			client->sendPacketUdp(&reply);
	}

}

