#include "MarkAssociator.h"
#include "MathHelper.h"
#include "CoordinateConverter.h"
#include <map>
#include <cmath>

namespace sauron
{

#if _USE_NEW_MARK_ASSOCIATOR

MarkAssociator::MarkAssociator()
{
}


MarkAssociator::MarkAssociator( const MarkVector &marks )
{
    loadMarks( marks );
}


MarkAssociator::~MarkAssociator()
{
}


void MarkAssociator::loadMarks( const MarkVector &marks )
{
    m_marks = marks;
}


void MarkAssociator::filterMarksByAngleOfView( const Pose &lastPose, MarkVector &possibleMarks ) const
{
    double deltaX;
    double deltaY;
    double length;

    double coneHalfAngle = CameraParams::getAngleOfView() / 2.0;
    double coneX = cos( lastPose.Theta() );
    double coneY = sin( lastPose.Theta() );

    double dotProduct;

    possibleMarks.clear();

    MarkVector::const_iterator it;
    for ( it = m_marks.begin(); it != m_marks.end(); ++it )
    {
        deltaX = it->getPosition().X() - lastPose.X();
        deltaY = it->getPosition().Y() - lastPose.Y();
        length = sqrt( deltaX * deltaX + deltaY * deltaY );
        deltaX /= length;
        deltaY /= length;

        dotProduct = coneX * deltaX + coneY * deltaY;
        double angle = acos( dotProduct );
        if ( angle <= coneHalfAngle )
            possibleMarks.push_back( *it );
    }
}


void MarkAssociator::associateMarks(const ProjectionVector &projections, 
                                    const Pose &lastPose, 
                                    MarkVector &associatedMarks, 
                                    ProjectionVector &associatedProjs) const
{
    MarkVector possibleMarks;
    MarkVector::const_iterator mIt;

    ProjectionVector::const_iterator projIt;

    associatedMarks.clear();
    associatedProjs.clear();

    if ( !projections.size() )
        return;

    filterMarksByAngleOfView( lastPose, possibleMarks );
    if ( !possibleMarks.size() )
        return;

    for ( mIt = possibleMarks.begin(); mIt != possibleMarks.end(); ++mIt )
    {
        //double posU = CoordinateConverter::Wordl2Cam_U( mIt->getPosition().X() * cos( lastPose.Theta() ) - lastPose.X(), 
        //                                                mIt->getPosition().Y() * sin( lastPose.Theta() ) - lastPose.Y() );

        double posU = predictMarkPositionAtCamera( lastPose, *mIt );
        std::map< double, const Projection* >  correlationMap;

        for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
        {
            if ( abs( projIt->getDiscretizedLine().getMeanX() - posU ) > 10 )
                continue;

            correlationMap[ mIt->compare( *projIt ) ] = &(*projIt);
        }

        if ( correlationMap.size() )
        {
            // TODO Move the mim value to a more appropriate place
            if ( correlationMap.rbegin()->first > 0.0 )
            {
                associatedMarks.push_back( *mIt );
                associatedProjs.push_back( *(correlationMap.rbegin()->second) );
            }
        }

        //associatedMarks.push_back( *mIt );
    }
}


const MarkVector &MarkAssociator::getMarks() const
{
    return m_marks;
}


pose_t MarkAssociator::predictMarkPositionAtCamera( const Pose &lastPose, const Mark &mark ) const
{
    double camX = mark.getPosition().X() - lastPose.X();
    double camY = mark.getPosition().Y() - lastPose.Y();
    double rotCamX = camX * cos( lastPose.Theta() ) - camY * sin( lastPose.Theta() );
    double rotCamY = camX * sin( lastPose.Theta() ) + camY * cos( lastPose.Theta() );
    return CoordinateConverter::Wordl2Cam_U( -rotCamX, rotCamY );
}

#else
MarkAssociator::MarkAssociator()
{
}


MarkAssociator::MarkAssociator( const MarkVector &marks )
{
    loadMarks( marks );
}


MarkAssociator::~MarkAssociator()
{
}


void MarkAssociator::loadMarks( const MarkVector &marks )
{
    m_marks = marks;
}


uint MarkAssociator::associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks, ProjectionVector &associatedProjs ) const
{
    MarkVector::const_iterator markIt;
    ProjectionVector::const_iterator projIt;

    std::vector< std::vector< float > > correlationsVec;

    associatedMarks.clear();
    associatedProjs.clear();

    if ( projections.empty() )
        return 0;

    for (  markIt = m_marks.begin(); markIt != m_marks.end(); ++markIt )
    {
        // TODO Insert some kind of mark filtering
        std::vector< float > corrs;
        for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
        {
            const float correlation = markIt->compare( *projIt );
            if ( correlation > 0.80f/*PROJECTION_TRACKER_CORRELATION_THRESHOLD*/ )
                corrs.push_back( correlation );
            else
                corrs.push_back( -1.0f );


        }

        correlationsVec.push_back( corrs );
    }

    while ( correlationsVec.size() )
    {
        float maxCorr = -1.0f;
        uint maxCorrMarkIndex = 0;
        uint maxCorrProjIndex = 0;

        bool matched  = false;
        bool allEmpty = true;

        std::vector< std::vector< float > >::iterator it = correlationsVec.begin();
        std::vector< std::vector< float > >::iterator toDelete = correlationsVec.end();
       
        for ( register uint i = 0; i < correlationsVec.size(); ++i, ++it )
        {
            std::vector< float > corrs = correlationsVec[i];
            for ( register uint j = 0; j < corrs.size(); ++j )
            {
                if ( corrs[j] > maxCorr )
                {
                    maxCorr = corrs[j];
                    maxCorrMarkIndex = i;
                    maxCorrProjIndex = j;
                    toDelete = it;
                    matched = true;
                }
            }
        }

        if ( matched )
        {
            associatedMarks.push_back( m_marks[maxCorrMarkIndex] );
            associatedProjs.push_back( projections[maxCorrProjIndex] );
            correlationsVec.erase( toDelete );
        }

        for ( register uint i = 0; i < correlationsVec.size(); ++i )
        {
            std::vector< float > corrs = correlationsVec[i];
            for ( register uint j = 0; j < corrs.size(); ++j )
            {
                if ( j == maxCorrProjIndex )
                {
                    if ( corrs[j] != -1.0f )
                    {
                        corrs[j] = -1.0f;
                        allEmpty = false;
                    }
                }
                else if ( corrs[j] > 0.0f )
                    allEmpty = false;
            }
        }

        if ( allEmpty )
            break;
    }

    return associatedMarks.size();
}

#endif // _USE_NEW_MARK_ASSOCIATOR

}   // namespace sauron