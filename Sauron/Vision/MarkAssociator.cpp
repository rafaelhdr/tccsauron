#include "MarkAssociator.h"

namespace sauron
{

MarkAssociator::MarkAssociator()
{
}

MarkAssociator::~MarkAssociator()
{
}


void MarkAssociator::loadMarks( const std::string &filename )
{
}


void MarkAssociator::associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks ) const
{
    MarkVector::const_iterator markIt;
    ProjectionVector::const_iterator projIt;

    std::vector< std::vector< float > > correlationsVec;

    associatedMarks.clear();

    for (  markIt = m_marks.begin(); markIt != m_marks.end(); ++markIt )
    {
        // TODO Insert some kind of mark filtering
        std::vector< float > corrs;
        for ( projIt = projections.begin(); projIt != projections.end(); ++projIt )
        {
            const float correlation = markIt->compare( *projIt );
            if ( correlation > 0.8f/*PROJECTION_TRACKER_CORRELATION_THRESHOLD*/ )
                corrs.push_back( correlation );
        }

        correlationsVec.push_back( corrs );
    }

    while ( correlationsVec.size() )
    {
        float maxCorr = -1.0f;
        uint maxCorrIndexI = 0;
        uint maxCorrIndexJ = 0;

        std::vector< std::vector< float > >::iterator it = correlationsVec.begin();
        std::vector< std::vector< float > >::iterator toDelete;
       
        for ( register uint i = 0; i < correlationsVec.size(); ++i, ++it )
        {
            std::vector< float > corrs = correlationsVec[i];
            for ( register uint j = 0; j < corrs.size(); ++j )
            {
                if ( corrs[j] > maxCorr )
                {
                    maxCorr = corrs[j];
                    maxCorrIndexI = i;
                    maxCorrIndexJ = j;
                    toDelete = it;
                }
            }
        }

        associatedMarks.push_back( m_marks[maxCorrIndexI] );

        correlationsVec.erase( toDelete );

        for ( register uint i = 0; i < correlationsVec.size(); ++i, ++it )
        {
            std::vector< float > corrs = correlationsVec[i];
            for ( register uint j = 0; j < corrs.size(); ++j )
            {
                if ( j == maxCorrIndexJ )
                    corrs[j] = -1.0f;
            }
        }
    }
}

}   // namespace sauron