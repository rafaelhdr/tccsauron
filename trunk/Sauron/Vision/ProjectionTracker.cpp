#include "ProjectionTracker.h"
#include <utility>

namespace sauron
{

ProjectionTracker::ProjectionTracker()
    : m_iterationCount( 0 ),
      m_uniqueIDReference( 0 ),
      m_maxTimeBetweenObservations( 8 ),
      m_minObservations( 5 )
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

        std::vector< bool > notMatchedVec( projs.size() );
        std::fill( notMatchedVec.begin(), notMatchedVec.end(), true );

        std::map< trackid_t, Projection >::const_iterator lastSeenIt;
        ProjectionVector::const_iterator newProjsIt;

        for ( lastSeenIt = copyLastSeen.begin() ; lastSeenIt != copyLastSeen.end(); ++lastSeenIt )
        {
            unsigned int index = 0;
            for ( newProjsIt = projs.begin(); newProjsIt != projs.end(); ++newProjsIt, ++index )
            {
                if ( lastSeenIt->second == *newProjsIt )
                {
                    trackProjection( lastSeenIt->first, *newProjsIt );
                    notMatchedVec[ index ] = false;
                    break;
                }
            }
        }

        const uint size = notMatchedVec.size();
        for ( register unsigned int i = 0; i < size; ++i )
            if ( notMatchedVec[i] )
                startTrackingNewProjection( projs[i] );


        retrieveValidTrackedProjections( trackedProjs );
        removeOldObservations();
    }

    ++m_iterationCount;

    /*ProjectionVector previousProjections;
    if ( m_buffer.filteredPop( previousProjections ) )
    {
        ProjectionVector::iterator currentProjsIt;
        ProjectionVector::iterator previousProjsIt;
        for ( currentProjsIt = projs.begin(); currentProjsIt != projs.end(); ++currentProjsIt )
        {
            for ( previousProjsIt = previousProjections.begin(); previousProjsIt != previousProjections.end(); ++previousProjsIt )
            {
                previousProjsIt->equals( *currentProjsIt );
            }
        }

        m_buffer.push( projs );
        m_buffer.filteredPop( trackedProjs );
    }
    else
    {
        m_buffer.push( projs );
        ProjectionVector allProjs;
        m_buffer.pop( allProjs );


        trackedProjs.clear();
    }*/
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



//
//LineTracker::LineTracker()
//{
//}
//
//LineTracker::~LineTracker()
//{
//}
//
//void LineTracker::track( std::vector<DiscretizedLine> &previous, std::vector<DiscretizedLine> &current, sauron::Image &im )
//{
//    uint previousNumLines = previous.size();
//    uint currentNumLines  = current.size();
//
//    if ( !previousNumLines )
//        return;
//
//    std::vector < ColorProfile > previousCP;
//    std::vector < ColorProfile > currentCP;
//
//    const uint size = 5;
//
//    for ( register uint i = 0; i < previousNumLines; ++i )
//    {
//        ColorProfile profile( im, previous[i], size );
//        previousCP.push_back( profile );
//        //previousCP.push_back( ColorProfile( im, previous[i], size ) );
//    }
//
//    for ( register uint i = 0; i < currentNumLines; ++i )
//    {
//        ColorProfile profile( im, current[i], size );
//        currentCP.push_back( profile );
//        //currentCP.push_back( ColorProfile( im, current[i], size ) );
//    }
//
//    bool *currentMatched  = new bool [ currentNumLines ];
//    bool *previousMatched = new bool [ previousNumLines ];
//    std::fill( currentMatched, currentMatched + currentNumLines, false );
//    std::fill( previousMatched, previousMatched + previousNumLines, false );
//
//    static char name = 'A';
//    CvFont font;
//    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0 );
//
//    for ( register uint i = 0; i < currentNumLines; ++i )
//    {
//        if ( currentMatched[i] )
//            continue;
//
//        for ( register uint j = 0; j < previousNumLines; ++j )
//        {
//            if ( previousMatched[i] )
//                continue;
//
//            if ( currentCP[i] == previousCP[i] )
//            {
//                currentMatched[i]  = true;
//                previousMatched[i] = true;
//
//                if ( previous[i].name == "" )
//                    current[i].name.push_back( name++ );
//                else
//                    current[i].name = previous[i].name;
//
//                if ( name > 'Z' )
//                    name = 'A';
//       
//
//                for ( register uint k = 0; k < current[i].getNumPoints(); ++k )
//                {   
//                    Point2DInt point = current[i].getPoint( k );
//                    im( point.X(), point.Y() ).set( 255, 255, 255 );
//                }
//                Point2DInt point = current[i].getPoint( 0 );
//                cvPutText( im, current[i].name.c_str(), cvPoint( point.X() + 3, point.Y() ), &font, CV_RGB( 255, 255, 255 ) );
//            }
//        }
//    }
//}

}   // namespace sauron