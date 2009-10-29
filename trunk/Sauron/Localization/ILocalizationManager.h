#ifndef __LOCALIZATION_H__
#define __LOCALIZATION_H__

namespace sauron
{

class ILocalizationManager
{
    public:
        virtual void setInitialPose( const Pose &initial ) = 0;
        virtual Pose getEstimate() = 0;
        virtual void update() = 0;
		virtual Map getMap() = 0;
};


}   // namespace sauron

#endif  // __LOCALIZATION_H__