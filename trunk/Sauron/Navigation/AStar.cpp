#include "AStar.h"
#include <map>
#include <set>
#include <list>
#include <algorithm>

namespace sauron
{

Path AStar::searchPath( const Node &initial, const Node &final )
{
    std::set<Node>        openedNodes;
    std::set<Node>        closedNodes;
    std::map<Node, pose_t> avaliations;
    std::map<Node, pose_t> traveled;
    std::map<Node, Node>   comesFrom;

    std::map<Node, pose_t>::const_iterator it;
    std::vector<Node *>::const_iterator adIt;

    traveled[ initial ]    = 0.0;
    avaliations[ initial ] = estimateHeuristic( initial, final );
    openedNodes.insert( initial );

    while ( openedNodes.size() )
    {
		// procura o nó aberto com o menor valor de f (f = g + h)
        std::set<Node>::iterator openedIt = openedNodes.begin();
        it = avaliations.find( *openedIt );
        for (  ++openedIt; openedIt != openedNodes.end(); ++openedIt )
        {
            std::map<Node, pose_t>::iterator temp = avaliations.find( *openedIt );
            if ( temp->second < it->second )
                it = temp;
        }

		// se o melhor nó aberto for o final, voilà
        if ( it->first == final )
            return buildPath( final, comesFrom );

		// vamos examinar este nó, então tiramos da lista dos fechados
        closedNodes.insert( it->first );
        openedNodes.erase( it->first );

		// para cada nó adjacente:
        const std::vector<Node *> &adjs = it->first.getAdjacents();
        for ( adIt = adjs.begin(); adIt < adjs.end(); ++adIt )
        {
			// se estiver no conjunto dos nós fechados, ignora (já vimos este adjacente)
            if ( std::find( closedNodes.begin(), closedNodes.end(), **adIt ) != closedNodes.end() )
                continue;

			// heurística de custo para este adjacente
            pose_t tempAval = traveled[ it->first ] +
                              it->first.getPosition().getDistance( (*adIt)->getPosition() );

            bool isBetter;

			// se este adjacente não estiver no conjunto de nós abertos
            if ( std::find( openedNodes.begin(), openedNodes.end(), **adIt ) == openedNodes.end() )
			{
                openedNodes.insert( **adIt );
				isBetter = true;
			}
			else if(tempAval < traveled[**adIt])
			{
				isBetter = true;
			}
			else
			{
				isBetter = false;
			}

            if ( isBetter )
            {
				comesFrom[ **adIt ]   = it->first;
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

Path AStar::buildPath(  const Node &goal, std::map<Node, Node> &cameFrom )
{
	Path reversePath = buildReversePath(goal, cameFrom);
	Path path;
	for(Path::reverse_iterator it = reversePath.rbegin(); it != reversePath.rend(); it++)
	{
		path.push_back(*it);
	}
	return path;
}

Path AStar::buildReversePath( const Node &current, std::map<Node, Node> &cameFrom )
{
	if(cameFrom.find(current) != cameFrom.end()) {
		Path p = buildReversePath(cameFrom[current], cameFrom);
		p.push_back(current);
		return p;
	} else {
		return Path();
	}
	/*
    Path path;
    Node next = goesTo[ initial ];

    while ( next != final )
    {
        path.push_back( next );
        next = goesTo[ next ];
    }

    path.push_back( next );
    return path;*/
}

}   // namespace sauron