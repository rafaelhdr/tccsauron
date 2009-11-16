#include "StdAfx.h"
#include "SonarMatchServer.h"

namespace sauron
{
namespace tests
{
SonarMatchServer::SonarMatchServer(ArServerInfoDrawings* drawingsServer,
								   LocalizationManager* locManager) :
mp_localization(locManager), mp_drawingsServer(drawingsServer),
m_painter(drawingsServer)
{
	m_callbackId = mp_localization->addPoseChangedCallback(
		boost::bind(&SonarMatchServer::paintCallback, this, _1));
}

SonarMatchServer::~SonarMatchServer(void)
{
	mp_localization->removePoseChangedCallback(m_callbackId);
}

void SonarMatchServer::paintCallback(const Pose& pose)
{
	m_painter.update(mp_localization->getSonarMatches());
}
}
}