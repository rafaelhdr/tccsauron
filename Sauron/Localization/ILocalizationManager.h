#ifndef __LOCALIZATION_H__
#define __LOCALIZATION_H__

#include "Matrix.h"
#include <boost/thread.hpp>
namespace sauron
{
class Pose;
class Map;
class ILocalizationManager
{
    public:
        virtual void setInitialPose( const Pose &initial ) = 0;
        virtual Pose getPose() = 0;
		virtual Map getMap() = 0;
		virtual boost::recursive_mutex* getPoseMutex() = 0;

		virtual void update(const Matrix &hValue,
							const Measure &z,
							const Model &H,
							const Covariance &R) = 0;

		virtual void predict(const Matrix &fValue,
							 const Model &dynModel,
							 const Covariance &dynNoise) = 0;


};


}   // namespace sauron

#endif  // __LOCALIZATION_H__