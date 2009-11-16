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
		static void linkTemporaryNode( Graph &graph, Node &tempNode, const Node &goal );
};

}   // namespace util

}   // namespace sauron

#endif  // __WAYPOINT_LINKER_H__