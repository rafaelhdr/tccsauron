#include "VisionModel.h"

#include <iostream>

namespace sauron
{

VisionModel::VisionModel()
    : m_lastFrame( 320, 240, 8, sauron::Pixel::PF_RGB ),
      m_lastMarksFrame( 320, 240, 8, sauron::Pixel::PF_RGB )
{
    // Empirical values
    m_sigmaVert = 1.0f;

    m_camera.setSize( 320, 240 );

    uint camWidth  = m_camera.getWidth();
    uint camHeigth = m_camera.getHeight();

    m_lastFrame      = Image( camWidth, camHeigth, 8, Pixel::PF_RGB );
    m_lastMarksFrame = Image( camWidth, camHeigth, 8, Pixel::PF_RGB );
}

VisionModel::~VisionModel()
{
}


void VisionModel::loadMarks( const std::string &filename )
{
    MarkVector marksLoaded;
    MarkPersistenceManager::loadFromFile( filename, marksLoaded );
    m_associator.loadMarks( marksLoaded );
}


void VisionModel::getLastFrame( sauron::Image &frame )
{
    frame = m_lastMarksFrame;
}


void VisionModel::getLastFrameWithMarks( sauron::Image &frame )
{
    frame = m_lastMarksFrame;

    for ( register uint i = 0; i < m_marksAssociatedProjections.size(); ++i )
        drawProjection( frame, m_marksAssociatedProjections[i], 255, 255, 0 );
}


void VisionModel::getLastFrameWithProjections( sauron::Image &frame )
{
    frame = m_lastFrame;

    for ( register uint i = 0; i < m_projectionsSeen.size(); ++i )
        drawProjection( frame, m_projectionsSeen[i], 255, 0, 0 );
}


//void VisionModel::getLastFrameWithTrackedProjections( sauron::Image &frame )
//{
//    frame = m_lastFrame;
//
//    for ( register uint i = 0; i < m_projectionsTracked.size(); ++i )
//        drawProjection( frame, m_projectionsTracked[i], 0, 255, 0 );
//}


void VisionModel::drawProjection( Image &im, const Projection &proj, byte r, byte g, byte b )
{
    sauron::DiscretizedLine line = proj.getDiscretizedLine();
    for ( register sauron::uint k = 0; k < line.getNumPoints(); ++k )
    {   
        sauron::Point2DInt point = line.getPoint( k );
        im( point.X(), point.Y() ).set( r, g, b );
    }
}


bool VisionModel::updateCaptureDetectTrackAssociate( const Pose &lastPose )
{
    static Image frame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );
    static Image grayFrame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );

    m_camera.getFrame( frame );   
    grayFrame = frame;
    m_lastFrame = frame;

    m_convolutor.convolve( grayFrame );
    grayFrame.convertToGray();

    m_detector.detect( frame, grayFrame, m_projectionsSeen );

//    m_tracker.track( m_projectionsSeen, m_projectionsTracked );
//    return m_associator.associateMarks( m_projectionsTracked, m_marksAssociated, m_marksAssociatedProjections );    

    m_associator.associateMarks( m_projectionsSeen, lastPose, m_marksAssociated, m_marksAssociatedProjections );
    if ( m_marksAssociated.size() )
        return true;

    return false;
}



void VisionModel::getAssociatedMarks( MarkVector &marks, ProjectionVector &projs )
{
    //m_associator.associateMarks( m_projectionsTracked, m_marksAssociated, m_marksAssociatedProjections );
    marks = m_marksAssociated;
    projs = m_marksAssociatedProjections;
    m_lastMarksFrame = m_lastFrame;
}


double VisionModel::getSigma()
{
    return m_sigmaVert;
}

}   // namespace sauron