#include "AStar.h"
#include <map>
#include <list>
#include <algorithm>

namespace sauron
{

Path AStar::searchPath( const Node &initial, const Node &final )
{
    std::list<Node>        openedNodes;
    std::list<Node>        closedNodes;
    std::map<Node, pose_t> avaliations;
    std::map<Node, pose_t> traveled;
    std::map<Node, Node>   goesTo;

    std::map<Node, pose_t>::const_iterator it;
    std::vector<Node *>::const_iterator adIt;

    traveled[ initial ]    = 0.0;
    avaliations[ initial ] = estimateHeuristic( initial, final );
    openedNodes.push_back( initial );

    while ( openedNodes.size() )
    {
        std::list<Node>::iterator openedIt = openedNodes.begin();
        it = avaliations.find( *openedIt );
        for (  ++openedIt; openedIt != openedNodes.end(); ++openedIt )
        {
            std::map<Node, pose_t>::iterator temp = avaliations.find( *openedIt );
            if ( temp->second < it->second )
                it = temp;
        }

        if ( it->first == final )
            return buildPath( initial, final, goesTo );

        closedNodes.push_back( it->first );
        openedNodes.remove( it->first );

        const std::vector<Node *> &adjs = it->first.getAdjacents();

        for ( adIt = adjs.begin(); adIt < adjs.end(); ++adIt )
        {
            if ( std::find( closedNodes.begin(), closedNodes.end(), **adIt ) != closedNodes.end() )
                continue;

            pose_t tempAval = traveled[ it->first ] +
                              it->first.getPosition().getDistance( (*adIt)->getPosition() );

            bool isBetter = true;

            if ( std::find( openedNodes.begin(), openedNodes.end(), **adIt ) == openedNodes.end() )
                openedNodes.push_back( **adIt );
            else if ( tempAval >= traveled[ **adIt ] )
                isBetter = false;

            if ( isBetter )
            {
                goesTo[ it->first ]   = **adIt;
                traveled[ **adIt ]    = tempAval;
                avaliations[ **adIt ] = tempAval + estimateHeuristic( **adIt, final );
            }
        }
    }

    return Path();
}


pose_t AStar::estimateHeuristic( const Node &current, const Node &final )
{
    return current.getPosition().getDistance( final.getPosition() );
}


Path AStar::buildPath( const Node &initial, const Node &final, std::map<Node, Node> &goesTo )
{
    Path path;
    Node next = goesTo[ initial ];

    while ( next != final )
    {
        path.push_back( next );
        next = goesTo[ next ];
    }

    path.push_back( next );
    return path;
}

}   // namespace sauron