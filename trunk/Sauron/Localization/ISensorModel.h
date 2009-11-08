#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include "Matrix.h"

namespace sauron
{
class ILocalizationManager;

class ISensorModel
{
    public:
		virtual void setLocalizationManager(ILocalizationManager& locManager);
	private:
		virtual bool getEstimate(/*out*/Matrix &hValue,
							   /*out*/Measure &z,
							   /*out*/Model &H,
							   /*out*/Covariance &R ) = 0;

};

}   // namespace sauron

#endif  // __SENSOR_MODEL_H__