#include "VisionModel.h"

#include <iostream>

namespace sauron
{

VisionModel::VisionModel()
    : m_lastFrame( 320, 240, 8, sauron::Pixel::PF_RGB ),
      m_lastMarksFrame( 320, 240, 8, sauron::Pixel::PF_RGB ),
      m_updateFreq( 60 )
{
    m_projectionPlaneHorizontalCenter = m_camera.getWidth() / 2.0;

    // Empirical values
    m_horizontalFocalDistance = 1.0f;
    m_sigmaVert = 1.0f;

    m_camera.setSize( 320, 240 );

    m_thread = boost::thread( boost::ref( *this ) );
    m_threadRunning = true;
    m_threadPause = false;

    
}

VisionModel::~VisionModel()
{
    stop();
}


void VisionModel::loadMarks( const std::string &filename )
{
    MarkVector marksLoaded;
    MarkPersistenceManager::loadFromFile( filename, marksLoaded );
    m_associator.loadMarks( marksLoaded );
}


void VisionModel::getLastFrame( sauron::Image &frame )
{
    m_mutexLastFrame.lock();
    frame = m_lastMarksFrame;
    m_mutexLastFrame.unlock();
}


void VisionModel::getLastFrameWithMarks( sauron::Image &frame )
{
    m_mutexLastFrame.lock();
    frame = m_lastMarksFrame;
    m_mutexLastFrame.unlock();

    for ( register uint i = 0; i < m_marksAssociatedProjections.size(); ++i )
        drawProjection( frame, m_marksAssociatedProjections[i], 255, 255, 0 );
}


void VisionModel::getLastFrameWithProjections( sauron::Image &frame )
{
    m_mutexLastFrame.lock();
    frame = m_lastFrame;
    m_mutexLastFrame.unlock();

    m_mutexProjectionsSeen.lock();
    for ( register uint i = 0; i < m_projectionsSeen.size(); ++i )
        drawProjection( frame, m_projectionsSeen[i], 255, 0, 0 );
    m_mutexProjectionsSeen.unlock();
}


void VisionModel::getLastFrameWithTrackedProjections( sauron::Image &frame )
{
    m_mutexLastFrame.lock();
    frame = m_lastFrame;
    m_mutexLastFrame.unlock();

    m_mutexProjectionsTracked.lock();
    for ( register uint i = 0; i < m_projectionsTracked.size(); ++i )
        drawProjection( frame, m_projectionsTracked[i], 0, 255, 0 );
    m_mutexProjectionsTracked.unlock();
}


void VisionModel::drawProjection( Image &im, const Projection &proj, byte r, byte g, byte b )
{
    sauron::DiscretizedLine line = proj.getDiscretizedLine();
    for ( register sauron::uint k = 0; k < line.getNumPoints(); ++k )
    {   
        sauron::Point2DInt point = line.getPoint( k );
        im( point.X(), point.Y() ).set( r, g, b );
    }
}


void VisionModel::updateCaptureDetectTrack()
{
    static Image frame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );
    static Image grayFrame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );

    clock_t allStart = clock();

    clock_t start = clock();
    m_camera.getFrame( frame );
    std::cout << " Get Frame:" << clock() - start;
    
    start = clock();
    grayFrame = frame;
    std::cout << "\tCopy Frame:" << clock() - start;

    
    start = clock();
    m_convolutor.convolve( grayFrame );
    std::cout << "\tConvolve:" << clock() - start;

    start = clock();
    grayFrame.convertToGray();
    std::cout << "\tTo Gray:" << clock() - start;

    start = clock();
    m_mutexProjectionsSeen.lock();
    m_detector.detect( frame, grayFrame, m_projectionsSeen );
    std::cout << "\nDetect:" << clock() - start;

    start = clock();
    m_mutexProjectionsTracked.lock();
    m_tracker.track( m_projectionsSeen, m_projectionsTracked );
    std::cout << "\tTrack:" << clock() - start;

    m_mutexProjectionsSeen.unlock();
    m_mutexProjectionsTracked.unlock();
    
    start = clock();
    m_mutexLastFrame.lock();
    m_lastFrame = frame;
    m_mutexLastFrame.unlock();
    std::cout << "\tCopy Final:" << clock() - start << "\tTOTAL: " << clock() - allStart << std::endl;
}


void VisionModel::operator() () 
{
    boost::xtime time;
    boost::xtime afterTime;
    boost::xtime sleepTime;

    clock_t fpsStartTime = clock();
    unsigned int framesCount = 0;

    while ( m_threadRunning )
    {
        if ( m_threadPause )
        {
            m_thread.yield();
        }
        else
        {
            boost::xtime_get( &time, boost::TIME_UTC );
            this->updateCaptureDetectTrack();
            boost::xtime_get( &sleepTime, boost::TIME_UTC );
            m_mutexUpdateFreq.lock();
            boost::xtime_get( &afterTime, boost::TIME_UTC );
            sleepTime.nsec += (boost::xtime::xtime_nsec_t(1000000000) - (afterTime.nsec - time.nsec)) / m_updateFreq;
            m_mutexUpdateFreq.unlock();

            // DEBUG
            if ( clock() - fpsStartTime > CLOCKS_PER_SEC )
            {
                std::cout << "Thread: " << (double)framesCount * CLOCKS_PER_SEC / (double)(clock() - fpsStartTime) << " <=> " << framesCount <<  std::endl;
                framesCount = 0;
                fpsStartTime = clock();
            }
            else
                ++framesCount;

            
            //m_thread.sleep( sleepTime );
        }
    }
}


void VisionModel::setUpdateFrequency( uint hz )
{
    boost::mutex::scoped_lock lock( m_mutexUpdateFreq );
    m_updateFreq = hz;
}


void VisionModel::start()
{
    m_threadRunning = true;
    m_threadPause = false;
}


void VisionModel::stop()
{
    m_threadRunning = false;
}


void VisionModel::pause()
{
    m_threadPause = true;
}


void VisionModel::getAssociatedMarks( const Pose &current, MarkVector &marks, ProjectionVector &projs )
{
    boost::mutex::scoped_lock lock( m_mutexProjectionsTracked );
    m_associator.associateMarks( m_projectionsTracked, m_marksAssociated, m_marksAssociatedProjections );
    marks = m_marksAssociated;
    projs = m_marksAssociatedProjections;
    m_lastMarksFrame = m_lastFrame;
}


double VisionModel::getHorizontalFocalDistance()
{
    return m_horizontalFocalDistance;
}


double VisionModel::getProjectionPlaneHorizontalCenter()
{
    return m_projectionPlaneHorizontalCenter;
}


double VisionModel::getSigma()
{
    return m_sigmaVert;
}

}   // namespace sauron