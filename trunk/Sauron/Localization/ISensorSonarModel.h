#pragma once
#include "SonarMatch.h"
#include "ISensorModel.h"
namespace sauron
{
class ISensorSonarModel :
	public ISensorModel
{
public:
	virtual bool hasMatch() const = 0;
	virtual SonarMatch getLatestMatch() const = 0;
};
}
