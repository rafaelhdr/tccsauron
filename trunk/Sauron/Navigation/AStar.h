#ifndef __NAVIGATION_ASTAR_H__
#define __NAVIGATION_ASTAR_H__

#include <map>
#include "Node.h"

namespace sauron
{

typedef std::vector<Node> Path;

class AStar
{
    public:
        static Path searchPath( const Node &initial, const Node &final );

    private:
        static pose_t estimateHeuristic( const Node &current, const Node &final );
		static Path buildPath( const Node &final, std::map<Node, Node> &cameFrom );
};

}   // namespace sauron

#endif  //__NAVIGATION_ASTAR_H__