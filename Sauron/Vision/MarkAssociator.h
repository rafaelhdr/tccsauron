#ifndef __MARK_ASSOCIATOR_H__
#define __MARK_ASSOCIATOR_H__

#include <string>
#include "Mark.h"
#include "Projection.h"

namespace sauron
{

class MarkAssociator
{
    public:
        MarkAssociator();
        MarkAssociator( const MarkVector &marks );
        ~MarkAssociator();

        void loadMarks( const MarkVector &marks );
        void associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks, ProjectionVector &associatedProjs ) const;

    private:
        MarkVector m_marks; 
};

}   // namespace sauron

#endif  // __MARK_ASSOCIATOR_H__
