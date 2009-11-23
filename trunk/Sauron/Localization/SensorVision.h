#ifndef __SENSOR_VISION_H__
#define __SENSOR_VISION_H__

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "ISensorModel.h"
#include "Vision/VisionModel.h"

namespace sauron
{

class ILocalizationManager;

class SensorVision : public ISensorModel
{
    public:
        SensorVision( const std::string &marksFile );
        ~SensorVision();

        void setLocalizationManager( ILocalizationManager& locManager );

        bool getEstimate( Matrix &hValue, 
                          Measure &z, 
                          Model &H, 
                          Covariance &R );

        void updateEstimate();

        // Thread control
        void start();
        void pause();
        void stop();
        void setUpdateFrequency( uint hz );
        
        void mainLoop();

    private:
        VisionModel           m_visionModel;
        ILocalizationManager *m_localization;

        // Configs
        uint   m_updateFreq;

        // Thread
        boost::thread   m_thread;
        boost::mutex    m_mutexVisionModel;
        boost::mutex    m_mutexUpdateFreq;

        bool m_threadRunning;
        bool m_threadPause;

};

}   // namespace sauron

#endif  // __SENSOR_SONAR_H__
