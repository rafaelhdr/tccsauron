#pragma once
#include "SonarMatchPainter.h"

namespace sauron
{
namespace tests
{
class SonarMatchServer
{
public:
	SonarMatchServer(ArServerInfoDrawings* drawingsServer, LocalizationManager* locManager);
	~SonarMatchServer(void);

private:
	SonarMatchPainter m_painter;
	ArServerInfoDrawings* mp_drawingsServer;
	LocalizationManager* mp_localization;
	int m_callbackId;

	void paintCallback(const Pose& pose);
};
}
}