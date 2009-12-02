#include "SensorVision.h"
#include "ILocalizationManager.h"
#include "Vision/CameraParams.h"
#include <cmath>

namespace sauron
{

SensorVision::SensorVision( const std::string &marksFile )
    : m_updateFreq( 15 ),
      m_localization( NULL )
{
    m_visionModel.loadMarks( marksFile );

    m_thread = boost::thread( &SensorVision::mainLoop, this );
    start();
}


SensorVision::~SensorVision()
{
    stop();
    m_thread.join();
}


void SensorVision::setUpdateFrequency( uint hz )
{
    boost::mutex::scoped_lock lock( m_mutexUpdateFreq );
    m_updateFreq = hz;
}


void SensorVision::start()
{
    m_threadRunning = true;
    m_threadPause = false;
}


void SensorVision::stop()
{
    m_threadRunning = false;
}


void SensorVision::pause()
{
    m_threadPause = true;
}


bool SensorVision::getEstimate( Matrix &hValue, Measure &z, Model &H, Covariance &R )
{
    MarkVector       associatedMarks;
    ProjectionVector associatedProjections;

    Pose last = m_localization->getPose();

    m_mutexVisionModel.lock();
    m_visionModel.getAssociatedMarks( associatedMarks, associatedProjections );
    m_mutexVisionModel.unlock();

    if ( associatedMarks.size() > 0 )
    {
        uint numMarks = associatedMarks.size();

        z.resize( numMarks, 1 );
        H.resize( numMarks, 3 );
        R = boost::numeric::ublas::identity_matrix<double>( numMarks );
        hValue.resize( numMarks, 1 );

        z.clear();
        H.clear();
        hValue.clear();

        double aux_v;
        double aux_z;
        double aux_z2;

        double fu    = CameraParams::getHorizontalFocalDistance();
        double u0    = CameraParams::getProjectionPlaneHorizontalCenter();
        double sigma = m_visionModel.getSigma();

        double sinTheta;
        double cosTheta;
        double diffX;
        double diffY;

        int index = 0;

        MarkVector::iterator       itM = associatedMarks.begin();
        ProjectionVector::iterator itP = associatedProjections.begin(); 
        
        for ( ; itM != associatedMarks.end(); ++itM, ++itP, ++index )
        {
            const Point2DFloat markPos = itM->getPosition();

            sinTheta = sin( last.Theta() );
            cosTheta = cos( last.Theta() );

            diffX = markPos.X() - last.X();
            diffY = markPos.Y() - last.Y();

            //aux_v  = cosTheta * diffY - sinTheta * diffX;
            //aux_z  = cosTheta * diffX + sinTheta * diffY;
            aux_v = diffX * sinTheta + diffY * cosTheta;
            aux_z = diffX * cosTheta - diffY * sinTheta;
            aux_z2 = aux_z * aux_z;

            hValue( index, 0 ) = -fu * aux_v / aux_z + u0;

            //H( index, 0 ) =  -fu * (sinTheta * aux_z + cosTheta * aux_v) / aux_z2;
            //H( index, 1 ) =  fu * (cosTheta * aux_z - sinTheta * aux_v) / aux_z2;
            //H( index, 2 ) =  fu * ( (aux_v * aux_v) / aux_z2 + 1 );
            H( index, 0 ) = fu * (sinTheta * aux_z - cosTheta * aux_v ) / aux_z2;
            H( index, 1 ) = fu * (cosTheta * aux_z + sinTheta * aux_v ) / aux_z2;
            H( index, 2 ) = fu * ( -(aux_v * aux_v) / aux_z2 + 1.0 );

            R( index, index ) = sigma;

            z( index, 0 ) = itP->getDiscretizedLine().getMeanX();
            //z( index, 0 ) = hValue( index, 0 ) + rand() % 10 * (rand() % 2 > 0 ? 1 : -1);

        }
        
        return true;
    }

    return false;
}


void SensorVision::mainLoop() 
{
    boost::xtime time;
    boost::xtime lastTime;
    boost::xtime sleepTime;
    boost::xtime sleepDeltaTime;
    
    clock_t fpsStartTime = 0;
    unsigned int framesCount = 0;
    
    sleepDeltaTime.nsec = 0;

    bool toUpdate = false;

    cvNamedWindow( "MarksWindow" );
    Image frame( 320, 240, 8, Pixel::PF_RGB );

    while ( m_threadRunning )
    {
        if ( m_threadPause )
        {
            boost::xtime_get( &sleepTime, boost::TIME_UTC );
            sleepTime.nsec += 250000000;
            m_thread.sleep( sleepTime );
        }
        else
        {
            if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
            {
                //std::cout << "Thread: " << (double)framesCount * CLOCKS_PER_SEC / (double)(clock() - fpsStartTime) << " <=> " << framesCount <<  std::endl;
                framesCount = 1;

                boost::xtime_get( &lastTime, boost::TIME_UTC );

                m_mutexVisionModel.lock();
                toUpdate = m_visionModel.updateCaptureDetectTrackAssociate( m_localization->getPose() );
                m_mutexVisionModel.unlock();

                m_visionModel.getLastFrameWithMarks( frame, m_localization->getPose() );
                //m_visionModel.getLastFrameWithProjections( frame );
                cvShowImage( "MarksWindow", frame );
                cvWaitKey( 1 );

                if ( toUpdate )
                    this->updateEstimate();

                boost::xtime_get( &time, boost::TIME_UTC );
    
                m_mutexUpdateFreq.lock();
                sleepDeltaTime.nsec = ( boost::xtime::xtime_nsec_t( 1000000000 ) - m_updateFreq * (time.nsec - lastTime.nsec) ) / m_updateFreq;
                m_mutexUpdateFreq.unlock();

                fpsStartTime = clock();
            }
            else
            {
                m_mutexVisionModel.lock();
                bool toUpdate = m_visionModel.updateCaptureDetectTrackAssociate( m_localization->getPose() );
                m_mutexVisionModel.unlock();

                m_visionModel.getLastFrameWithMarks( frame, m_localization->getPose() );
                //m_visionModel.getLastFrameWithProjections( frame );
                cvShowImage( "MarksWindow", frame );
                cvWaitKey( 1 );
                
                if ( toUpdate )
                    this->updateEstimate();
                
                ++framesCount;
            }

            boost::xtime_get( &sleepTime, boost::TIME_UTC );
            sleepTime.nsec += sleepDeltaTime.nsec;

            m_thread.sleep( sleepTime );
        }
    }

    cvDestroyWindow( "MarksWindow" );
}


void SensorVision::updateEstimate()
{
	if( m_localization ) 
    {
		Matrix          hValue; 
		Measure         z; 
		Model           H; 
		Covariance      R;

		if( getEstimate( hValue, z, H, R ) ) 
			m_localization->update( hValue, z, H, R );
	}
}


void SensorVision::setLocalizationManager( ILocalizationManager &locManager )
{
    m_localization = &locManager;
}

}   // namespace sauron