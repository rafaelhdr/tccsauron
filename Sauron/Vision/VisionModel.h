#ifndef __VISION_MODEL_H__
#define __VISION_MODEL_H__

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
        void getAssociatedMarks( MarkVector &marks, ProjectionVector &projs );

        // Parameters getters
        double getSigma();

        // Frame getters - debug or visual info for final application
        void getLastFrame( Image &frame );
        void getLastFrameWithProjections( Image &frame );
        //void getLastFrameWithTrackedProjections( Image &frame );
        void getLastFrameWithMarks( Image &frame );

        // Main method - triggers all vision processes
        bool updateCaptureDetectTrackAssociate( const Pose &lastPose );

    private:
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

        // Model constants
        double m_sigmaVert;
};

}   // namespace sauron

#endif  // __VISION_MODEL_H__