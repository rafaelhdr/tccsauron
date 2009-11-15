#ifndef __WAYPOINT_LINKER_H__
#define __WAYPOINT_LINKER_H__

#include "Node.h"

namespace sauron
{

namespace util
{

class WaypointLinker
{
    public:
        static void link( Graph &graph );
        static void linkNodeToNearest( Graph &graph, Node &toLink, bool bidirectional = false );
};

}   // namespace util

}   // namespace sauron

#endif  // __WAYPOINT_LINKER_H__