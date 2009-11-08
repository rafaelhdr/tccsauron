#ifndef __DYNAMIC_H__
#define __DYNAMIC_H__

#include "Matrix.h"

namespace sauron
{

class ILocalizationManager;

class IDynamicModel
{
	public:
		virtual void setLocalizationManager(ILocalizationManager& locManager) = 0;
    private:
        virtual void updateModel(Matrix &fValue, Model &dynModel, Covariance &dynNoise ) = 0;
};

}   // namespace sauron

#endif  // __DYNAMIC_H__