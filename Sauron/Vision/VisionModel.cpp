#include "VisionModel.h"
#include "CoordinateConverter.h"

#include <iostream>
#include <sstream>

namespace sauron
{

VisionModel::VisionModel()
    : m_lastFrame( 320, 240, 8, sauron::Pixel::PF_RGB ),
      m_lastMarksFrame( 320, 240, 8, sauron::Pixel::PF_RGB )
{
    // Empirical values
    m_sigmaVert = 60.0f;

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


void VisionModel::getLastFrameWithMarks( Image &frame, const Pose &lastPose )
{
    frame = m_lastMarksFrame;

    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    for ( register uint i = 0; i < m_marksAssociatedProjections.size(); ++i )
        drawProjection( frame, m_marksAssociatedProjections[i], 255, 255, 0 );

    MarkVector marks; 
    m_associator.filterMarksByAngleOfView( lastPose, marks );
    for ( register uint i = 0; i < marks.size(); ++i )
    {
        double posU = m_associator.predictMarkPositionAtCamera( lastPose, marks[i] );

        if ( posU >= 0 && posU <= frame.getWidth() )
        {
            for ( uint h = 0; h < frame.getHeight(); ++h )
                frame( posU, h ).set( 0, 0, 255 );        

            std::stringstream ss;
            ss << marks[i].getDescription();       
            cvPutText( frame, ss.str().c_str(), cvPoint( (int)posU, 10 * (i+1) ), &font, CV_RGB( 0, 0, 255 ) );
        }
    }   
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


void VisionModel::drawProjection( Image &im, const Projection &proj, byte r, byte g, byte b, std::string &text )
{
    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );

    sauron::DiscretizedLine line = proj.getDiscretizedLine();
    for ( register sauron::uint k = 0; k < line.getNumPoints(); ++k )
    {   
        sauron::Point2DInt point = line.getPoint( k );
        im( point.X(), point.Y() ).set( r, g, b );
    }

    std::stringstream ss;
    if ( text.size() )
        ss << text;
    if ( ss.str().size() > 5 )
    {
        sauron::Point2DInt point = line.getPoint( line.getNumPoints() / 2 );
        cvPutText( im, ss.str().c_str(), cvPoint( point.X() - 4 * ss.str().size(), point.Y() ), &font, CV_RGB( r, g, b ) );
    }
    else
    {
        sauron::Point2DInt point = line.getPoint( 0 );
        cvPutText( im, ss.str().c_str(), cvPoint( point.X() + 3, point.Y() + 10 ), &font, CV_RGB( r, g, b ) );
    }
}


bool VisionModel::updateCaptureDetectTrackAssociate( const Pose &lastPose )
{
    static Image frame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );
    static Image grayFrame( m_camera.getWidth(), m_camera.getHeight(), 8, Pixel::PF_RGB );

    if ( !m_camera.getFrame( frame ) )
        return false;

    grayFrame = frame;
    m_lastFrame = frame;
    m_lastMarksFrame = frame;

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
    marks = m_marksAssociated;
    projs = m_marksAssociatedProjections;
    m_lastMarksFrame = m_lastFrame;
}


double VisionModel::getSigma()
{
    return m_sigmaVert;
}

}   // namespace sauron