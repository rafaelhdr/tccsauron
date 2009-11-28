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

    double coneHalfAngle = CameraParams::getAngleOfView();    double coneX = cos( lastPose.Theta() );
    double coneY = sin( lastPose.Theta() );
    coneHalfAngle /= 2.0;

    double crossProduct;

    possibleMarks.clear();

    MarkVector::const_iterator it;
    for ( it = m_marks.begin(); it != m_marks.end(); ++it )
    {

        deltaX = it->getPosition().X() - lastPose.X();
        deltaY = it->getPosition().Y() - lastPose.Y();
        length = sqrt( deltaX * deltaX + deltaY * deltaY );
        deltaX /= length;
        deltaY /= length;

        crossProduct = coneX * deltaY - coneY * deltaX;//deltaX * coneY - deltaY * coneX;
        double angle = asin( crossProduct );
        if ( asin( crossProduct ) <= coneHalfAngle )
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

    //std::map< Mark, std::map< const Projection *, double > > correlationMap;
    //std::map< Mark, std::map< const Projection *, double > >::iterator cmIt;

    //for ( mIt = possibleMarks.begin(); mIt != possibleMarks.end(); ++mIt )
    //{
    //    double posU = CoordinateConverter::Wordl2Cam_U( mIt->getPosition().X() - lastPose.X(),
    //                                                    mIt->getPosition().Y() - lastPose.Y() );

    //    for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
    //    {
    //        if ( abs( projIt->getDiscretizedLine().getMeanX() - posU ) > 10 )
    //            continue;

    //        correlationMap[ mark ][ *projIt ] = mIt->compare( *projIt );
    //    }
    //}

    //if ( correlationMap.size() )
    //{
    //    std::map< Mark, const Projection * > selectedAssociationsMap;
    //    std::map< Mark, 

    //    for ( mIt = possibleMarks.begin(); mIt != possibleMarks.end(); ++mIt )
    //    {
    //        for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
    //        {

    //        }
    //    }
    //}

    for ( mIt = possibleMarks.begin(); mIt != possibleMarks.end(); ++mIt )
    {
        //double posU = CoordinateConverter::Wordl2Cam_U( mIt->getPosition().X() * cos( lastPose.Theta() ) - lastPose.X(), 
        //                                                mIt->getPosition().Y() * sin( lastPose.Theta() ) - lastPose.Y() );

        double camX = mIt->getPosition().X() - lastPose.X();
        double camY = mIt->getPosition().Y() - lastPose.Y();
        double rotCamX = camX * cos( lastPose.Theta() ) - camY * sin( lastPose.Theta() );
        double rotCamY = camX * sin( lastPose.Theta() ) + camY * cos( lastPose.Theta() );
        double posU = CoordinateConverter::Wordl2Cam_U( -rotCamX, rotCamY );

        std::map< double, const Projection* >  correlationMap;

        for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
        {
            if ( abs( projIt->getDiscretizedLine().getMeanX() - posU ) > 20 )
                continue;

            correlationMap[ mIt->compare( *projIt ) ] = &(*projIt);
        }

        if ( correlationMap.size() )
        {
            // TODO Move the mim value to a more apropriate place
            if ( correlationMap.rbegin()->first > 0.0 )
            {
                associatedMarks.push_back( *mIt );
                associatedProjs.push_back( *(correlationMap.rbegin()->second) );
            }
        }
    }
}


const MarkVector &MarkAssociator::getMarks() const
{
    return m_marks;
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