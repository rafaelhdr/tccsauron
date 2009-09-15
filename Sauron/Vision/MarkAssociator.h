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
        ~MarkAssociator();

        void loadMarks( const std::string &filename );

        void associateMarks( const ProjectionVector &projections, MarkVector &associatedMarks ) const;

    private:
        MarkVector m_marks; 
};

}   // namespace sauron

#endif  // __MARK_ASSOCIATOR_H__
