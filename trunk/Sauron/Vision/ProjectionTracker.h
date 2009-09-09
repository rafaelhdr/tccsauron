#ifndef __PROJECTION_TRACKER_H__
#define __PROJECTION_TRACKER_H__

#include <map>
#include <list>
#include "Projection.h"

namespace sauron
{

class ProjectionTracker
{
    public:
        ProjectionTracker();
        ~ProjectionTracker();

        void reset();
        void track( const ProjectionVector &projs, ProjectionVector &trackedProjs );

        std::vector< uint > debug_getTrackedIDs() const;

    private:
        typedef uint trackid_t;
        typedef uint time_t;

        void startTrackingNewProjection( const Projection &proj );
        void trackProjection( trackid_t id, const Projection &proj );
        void stopTrackingProjection( trackid_t id );

        void retrieveValidTrackedProjections( ProjectionVector &tracked ) const;

        void removeOldObservations();

    private:
        time_t    m_iterationCount;
        trackid_t m_uniqueIDReference;

        time_t  m_maxTimeBetweenObservations;
        uint    m_minObservations;

        std::map< trackid_t, time_t >            m_lastTimeSeen;
        std::map< trackid_t, Projection >        m_lastProjectionSeen;
        std::multimap< trackid_t, Projection >   m_beenTrackedProjections;
        std::list< trackid_t >                   m_validIDs;
};

}   // namespace sauron

#endif  // _PROJECTION_TRACKER_H__