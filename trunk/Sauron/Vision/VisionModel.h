#ifndef __VISION_MODEL_H__
#define __VISION_MODEL_H__

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "Image.h"
#include "Camera.h"
#include "VerticalLineConvolutionOperator.h"
#include "VerticalProjectionDetector.h"
#include "ProjectionTracker.h"
#include "Mark.h"
#include "MarkPersistenceManager.h"
#include "MarkAssociator.h"
#include "Pose.h"

namespace sauron
{

class VisionModel
{
    public:
        VisionModel();
        ~VisionModel();

        void loadMarks( const std::string &filename );
        void getAssociatedMarks( const Pose &last, MarkVector &marks, ProjectionVector &projs );

        // Parameters getters
        double getHorizontalFocalDistance();
        double getProjectionPlaneHorizontalCenter();
        double getSigma();

        // Frame getters - debug or visual info for final application
        void getLastFrame( Image &frame );
        void getLastFrameWithProjections( Image &frame );
        void getLastFrameWithTrackedProjections( Image &frame );
        void getLastFrameWithMarks( Image &frame );

        // Thread control
        void start();
        void pause();
        void stop();
        void setUpdateFrequency( uint hz );
        
        void operator() ();

    private:
        void updateCaptureDetectTrack();
        

        void drawProjection( Image &im, const Projection &p, byte r, byte g, byte b );

    private:
        Camera                          m_camera;
        MarkAssociator                  m_associator;
        ProjectionTracker               m_tracker;
        VerticalProjectionDetector      m_detector;
        VerticalLineConvolutionOperator m_convolutor;

        //  Last iteratation 
        Image               m_lastFrame;
        Image               m_lastMarksFrame;
        ProjectionVector    m_projectionsSeen;
        ProjectionVector    m_projectionsTracked;
        MarkVector          m_marksAssociated;
        ProjectionVector    m_marksAssociatedProjections;

        // Configs
        uint   m_updateFreq;

        // Thread
        boost::thread   m_thread;
        boost::mutex    m_mutexLastFrame;
        boost::mutex    m_mutexProjectionsSeen;
        boost::mutex    m_mutexProjectionsTracked;
        boost::mutex    m_mutexUpdateFreq;

        bool m_threadRunning;
        bool m_threadPause;

        // Model constants
        double m_horizontalFocalDistance;
        double m_projectionPlaneHorizontalCenter;
        double m_sigmaVert;
};

}   // namespace sauron

#endif  // __VISION_MODEL_H__