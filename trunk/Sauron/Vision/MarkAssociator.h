#ifndef __MARK_ASSOCIATOR_H__
#define __MARK_ASSOCIATOR_H__

#include <string>
#include "Mark.h"
#include "Projection.h"
#include "Pose.h"

#define _USE_NEW_MARK_ASSOCIATOR    1

namespace sauron
{

#if _USE_NEW_MARK_ASSOCIATOR

class MarkAssociator
{
    public:
        MarkAssociator();
        MarkAssociator( const MarkVector &marks );
        ~MarkAssociator();

        void loadMarks( const MarkVector &marks );
        void associateMarks( const ProjectionVector &projections, 
                             const Pose &lastPose,
                             MarkVector &associatedMarks,
                             ProjectionVector &associatedProjs ) const;

        const MarkVector &getMarks() const;

    /*private:*/
        void filterMarksByAngleOfView( const Pose &lastPose, MarkVector &possibleMarks ) const;

    private:
        MarkVector m_marks;
};

#else  //_USE_NEW_MARK_ASSOCIATOR

class MarkAssociator
{
    public:
        MarkAssociator();
        MarkAssociator( const MarkVector &marks );
        ~MarkAssociator();

        void loadMarks( const MarkVector &marks );
        uint associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks, ProjectionVector &associatedProjs ) const;

    private:
        MarkVector m_marks; 
};

#endif  // _USE_NEW_MARK_ASSOCIATOR

}   // namespace sauron

#endif  // __MARK_ASSOCIATOR_H__
