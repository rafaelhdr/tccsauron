#include "WaypointLinker.h"
#include <algorithm>

namespace sauron
{

namespace util
{

void WaypointLinker::link( Graph &graph )
{
    Graph::iterator it;
    for ( it = graph.begin(); it != graph.end(); ++it )
        linkNodeToNearest( graph, *it, true );
}


void WaypointLinker::linkNodeToNearest( Graph &graph, Node &toLink, bool bidirectional )
{
    if ( toLink.Type() == Node::PRIMARY )
    {
        Graph::iterator closestIt;
        Graph::iterator tempIt;
        pose_t minDist = -1.0;

        // TODO Find a way to remove unnecessary links that are created between
        //      PRIMARY nodes and optimize them
        // TODO Check if the link don't lead to a crash with a wall or an object

        for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
        {
			if(*tempIt == toLink)
				continue;

            if ( tempIt->Type() == Node::SECONDARY )
                continue;

            const std::vector< Node *> adjs = toLink.getAdjacents();
            if ( std::find( adjs.begin(), adjs.end(), &(*tempIt) ) != adjs.end() )
                continue;

            if ( minDist < 0.0 )
            {
                closestIt = tempIt;
                minDist = toLink.getPosition().getDistance( closestIt->getPosition() );
            }
            else if ( minDist > toLink.getPosition().getDistance( tempIt->getPosition() ) )
            {
                closestIt = tempIt;
                minDist = toLink.getPosition().getDistance( tempIt->getPosition() );
            }
        }

        toLink.addAdjacent( *closestIt );
        if ( bidirectional )
            closestIt->addAdjacent( toLink );
    }
    else // secundary and temporary
    {
        Graph::iterator closestIt;
        Graph::iterator tempIt;
        pose_t minDist = -1.0;

        for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
        {
			if(*tempIt == toLink)
				continue;

            if ( tempIt->Type() == Node::SECONDARY )
                continue;

            if ( minDist < 0.0 )
            {
                closestIt = tempIt;
                minDist = toLink.getPosition().getDistance( closestIt->getPosition() );
            }
            else if ( minDist > toLink.getPosition().getDistance( tempIt->getPosition() ) )
            {
                closestIt = tempIt;
                minDist = toLink.getPosition().getDistance( tempIt->getPosition() );
            }
        }

		if(minDist > 0.0) {
			toLink.addAdjacent( *closestIt );
			if ( bidirectional )
				closestIt->addAdjacent( toLink );
		}
	}
}

void WaypointLinker::linkTemporaryNode( Graph &graph, Node &tempNode, const Node &goal )
{
	// a ideia é nunca linkar a um nó que nos deixará mais longe de onde já estamos
	Graph::iterator closestIt;
	Graph::iterator tempIt;
	pose_t minDist = -1;

	double distFromTempToGoal = tempNode.getPosition().getDistance( goal.getPosition() );

	for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
	{
		if(*tempIt == tempNode)
			continue;

		if ( tempIt->Type() == Node::SECONDARY && *tempIt != goal )
			continue;

		double distFromCurrentToGoal = tempIt->getPosition().getDistance( goal.getPosition() );

		if(distFromCurrentToGoal > distFromTempToGoal)
			continue;

		double distFromTempToCurrent = tempNode.getPosition().getDistance( tempIt->getPosition() );

		if( minDist < 0 || minDist > distFromTempToCurrent)
		{
			closestIt = tempIt;
			minDist =  distFromTempToCurrent;
		}
	}
	tempNode.addAdjacent( *closestIt );
}


}   // namespace util

}   // namespace sauron