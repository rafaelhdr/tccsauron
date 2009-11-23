#include "ProjectionTracker.h"

#include <algorithm>
#include <utility>

#define PROJECTION_TRACKER_CORRELATION_THRESHOLD    0.8f


namespace sauron
{

#if 0

ProjectionTracker::ProjectionTracker()
    : m_iterationCount( 0 ),
      m_uniqueIDReference( 0 ),
      m_maxTimeBetweenObservations( 8 ),
      m_minObservations( 10 )
{
}


ProjectionTracker::~ProjectionTracker()
{
}


void ProjectionTracker::reset()
{
    m_iterationCount = 0;
    m_uniqueIDReference = 0;
    m_lastTimeSeen.clear();
    m_beenTrackedProjections.clear();
}


void ProjectionTracker::startTrackingNewProjection( const Projection &proj )
{
    trackProjection( m_uniqueIDReference, proj );
    m_validIDs.push_back( m_uniqueIDReference );
    ++m_uniqueIDReference;
}


void ProjectionTracker::trackProjection( trackid_t id, const Projection &proj )
{
    m_beenTrackedProjections.insert( std::pair<trackid_t, Projection>( id, proj ) );

    // TODO Retrieve the last inserted pair at m_beenTrackedProjections to pass its pointer to
    //      m_lastProjectionSeen. This way, the class will use less memory, and may be faster
    m_lastProjectionSeen.erase( id );
    m_lastProjectionSeen.insert( std::pair<trackid_t, Projection>( id, proj ) );    

    m_lastTimeSeen.erase( id );
    m_lastTimeSeen.insert( std::pair<trackid_t, time_t>( id, m_iterationCount ) );
}


void ProjectionTracker::stopTrackingProjection( trackid_t id )
{
    m_beenTrackedProjections.erase( id );
    m_lastProjectionSeen.erase( id );
    m_lastTimeSeen.erase( id );
    m_validIDs.remove( id );
}


void ProjectionTracker::retrieveValidTrackedProjections( ProjectionVector &tracked ) const
{
    tracked.clear();

    std::map< trackid_t, Projection >::const_iterator it;
    for ( it = m_lastProjectionSeen.begin(); it != m_lastProjectionSeen.end(); ++it )
    {
        if ( m_beenTrackedProjections.count( it->first ) > m_minObservations )
           if ( m_lastTimeSeen.find( it->first )->second >= m_iterationCount )
                tracked.push_back( it->second );
    }
}


void ProjectionTracker::removeOldObservations()
{
    std::vector< trackid_t > toRemove;

    std::list< trackid_t >::iterator it;
    for ( it = m_validIDs.begin(); it != m_validIDs.end(); ++it )
    {
        if ( m_iterationCount - (*m_lastTimeSeen.find( *it )).second > m_maxTimeBetweenObservations )
            toRemove.push_back( *it );

        // TODO Remove observations older than a value to avoid excessive multimaps entries with the same key
    }

    std::vector< trackid_t >::iterator trIt;
    for ( trIt = toRemove.begin(); trIt != toRemove.end(); ++trIt )
        stopTrackingProjection( *trIt );    
}


void ProjectionTracker::track( const ProjectionVector &projs, ProjectionVector &trackedProjs ) 
{
    if ( m_beenTrackedProjections.empty() )
    {
        ProjectionVector::const_iterator it;
        for ( it = projs.begin(); it != projs.end(); ++it )
            startTrackingNewProjection( *it );
    }
    else
    {
        std::map< trackid_t, Projection > copyLastSeen = m_lastProjectionSeen;

        std::list< const Projection * > notMatchedList( projs.size() );
        for ( register uint i = 0; i < projs.size(); ++i )
            notMatchedList.push_back( &projs[i] );

        std::map< trackid_t, std::vector<AssociationPair> > associationsMap;

        std::map< trackid_t, Projection >::const_iterator lastSeenIt;
        ProjectionVector::const_iterator newProjsIt;

        for ( lastSeenIt = copyLastSeen.begin() ; lastSeenIt != copyLastSeen.end(); ++lastSeenIt )
        {
            for ( newProjsIt = projs.begin(); newProjsIt != projs.end(); ++newProjsIt )
            { 
                const float correlation = lastSeenIt->second.compare( *newProjsIt );
                if ( correlation > PROJECTION_TRACKER_CORRELATION_THRESHOLD )
                {
                    AssociationPair newPossibleAssociation;
                    newPossibleAssociation.projection  = &(*newProjsIt);
                    newPossibleAssociation.correlation = correlation;

                    associationsMap[ lastSeenIt->first ].push_back( newPossibleAssociation );
                }
            }
        }

        associateProjections( associationsMap, notMatchedList );

        std::list< const Projection * >::iterator notMatchedIt;
        for ( notMatchedIt = notMatchedList.begin(); notMatchedIt != notMatchedList.end(); ++notMatchedIt )
            if ( *notMatchedIt )
                startTrackingNewProjection( *(*notMatchedIt) );

        retrieveValidTrackedProjections( trackedProjs );
        removeOldObservations();
    }

    ++m_iterationCount;
}


void ProjectionTracker::associateProjections( std::map< trackid_t, std::vector<AssociationPair> > &associationsMap, 
                                              std::list< const Projection * > &notMatchedList )
{
    std::map< trackid_t, std::vector<AssociationPair> >::iterator mapIt;
    std::vector<AssociationPair>::iterator vecIt;

    std::pair< trackid_t, Projection * > selected;
    float maxSimilarity;

    while ( associationsMap.size() )
    {
        selected = std::make_pair( 0, (Projection *)NULL );
        maxSimilarity = -999999.9f;
        for ( mapIt = associationsMap.begin(); mapIt != associationsMap.end(); ++mapIt )
        {
            std::sort<std::vector<AssociationPair>::iterator>( mapIt->second.begin(), mapIt->second.end() );

            float similarity = -1.0f;
            if ( mapIt->second.size() > 1 )
                similarity = fabs( mapIt->second[0].correlation * ( mapIt->second[0].correlation - mapIt->second[1].correlation ) );
            else if ( mapIt->second.size() == 1 )
                similarity = fabs( mapIt->second[0].correlation * ( mapIt->second[0].correlation - PROJECTION_TRACKER_CORRELATION_THRESHOLD ) );
            else
                continue;

            if ( similarity > maxSimilarity )
            {
                selected = std::make_pair( mapIt->first, (Projection *)mapIt->second[0].projection );
                maxSimilarity = similarity;
            }
        }

        

        if ( selected.second )
        {
            trackProjection( selected.first, *selected.second );
            notMatchedList.remove( selected.second );

            associationsMap.erase( selected.first );

            for ( mapIt = associationsMap.begin(); mapIt != associationsMap.end(); ++mapIt )
            {
                if ( mapIt->second.size() )
                {
                    for ( vecIt = mapIt->second.begin(); vecIt != mapIt->second.end(); )
                    {
                        if ( vecIt->projection == selected.second )
                            vecIt = mapIt->second.erase( vecIt );
                        else
                            ++vecIt;
                    }
                }
            }
        }
        else
            break;
    }
}


std::vector< uint > ProjectionTracker::debug_getTrackedIDs() const
{ 
    std::vector< uint > ret;

    std::map< trackid_t, Projection >::const_iterator it;
    for ( it = m_lastProjectionSeen.begin(); it != m_lastProjectionSeen.end(); ++it )
    {
        if ( m_beenTrackedProjections.count( it->first ) > m_minObservations )
           if ( m_lastTimeSeen.find( it->first )->second >= m_iterationCount - 1)
                ret.push_back( it->first );
    }

    return ret;
}

#endif

}   // namespace sauron