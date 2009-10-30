#ifndef __LOCALIZATION_H__
#define __LOCALIZATION_H__

namespace sauron
{
class Pose;
class Map;
class ILocalizationManager
{
    public:
        virtual void setInitialPose( const Pose &initial ) = 0;
        virtual Pose getPose() = 0;
		virtual void start() = 0;
		virtual void stop() = 0;
		virtual Map getMap() = 0;
};


}   // namespace sauron

#endif  // __LOCALIZATION_H__