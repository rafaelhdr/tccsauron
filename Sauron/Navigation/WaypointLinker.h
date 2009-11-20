#ifndef __WAYPOINT_LINKER_H__
#define __WAYPOINT_LINKER_H__

#include "CustomTypes.h"
#include "Node.h"


namespace sauron
{
	class Map;

namespace util
{

class WaypointLinker
{
    public:
        static void link( Graph &graph, Map& map );
		static void linkTemporaryNode( Graph &graph, Node &tempNode, const Node &goal, Map& map );

private:
		static void linkNodeToNearest( Graph &graph, Node &toLink, Map& map );

		static bool linkClosestPossibleRight( Graph& graph, Node& node, Map& map );
		static bool linkClosestPossibleLeft( Graph& graph, Node& node, Map& map );
		static bool linkClosestPossibleUp( Graph& graph, Node& node, Map& map );
		static bool linkClosestPossibleDown( Graph& graph, Node& node, Map& map );
		static bool isLinkPossible( Node& node1, Node& node2, Map& map);

		static pose_t getPathLength( const Node& start, const Path& path );

};

}   // namespace util

}   // namespace sauron

#endif  // __WAYPOINT_LINKER_H__