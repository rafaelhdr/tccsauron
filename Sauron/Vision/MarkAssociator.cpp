#include "MarkAssociator.h"

namespace sauron
{

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


void MarkAssociator::associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks, ProjectionVector &associatedProjs ) const
{
    MarkVector::const_iterator markIt;
    ProjectionVector::const_iterator projIt;

    std::vector< std::vector< float > > correlationsVec;

    associatedMarks.clear();
    associatedProjs.clear();

    if ( projections.empty() )
        return;

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

    uint loopCount = 0;

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
}

}   // namespace sauron