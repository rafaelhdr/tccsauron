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
    {
        if ( it->Type() == Node::PRIMARY )
        {
            Graph::iterator closestIt;
            Graph::iterator tempIt;
            pose_t minDist = -1.0;

            // TODO Find a way to remove unnecessary links that are created between
            //      PRIMARY nodes and optimize them
            // TODO Check if the link don't lead to a crash with a wall or an object

            for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
            {
                if ( tempIt->Type() == Node::SECUNDARY )
                    continue;

                const std::vector< Node *> adjs = it->getAdjacents();
                if ( std::find( adjs.begin(), adjs.end(), &(*tempIt) ) != adjs.end() )
                    continue;

                if ( minDist < 0.0 )
                {
                    closestIt = tempIt;
                    minDist = it->getPosition().getDistance( closestIt->getPosition() );
                }
                else if ( minDist < it->getPosition().getDistance( tempIt->getPosition() ) )
                {
                    closestIt = tempIt;
                    minDist = it->getPosition().getDistance( tempIt->getPosition() );
                }
            }

            it->addAdjacent( *closestIt );
            closestIt->addAdjacent( *it );
        }
        else
        {
            Graph::iterator closestIt;
            Graph::iterator tempIt;
            pose_t minDist = -1.0;

            for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
            {
                if ( tempIt->Type() == Node::SECUNDARY )
                    continue;

                if ( minDist < 0.0 )
                {
                    closestIt = tempIt;
                    minDist = it->getPosition().getDistance( closestIt->getPosition() );
                }
                else if ( minDist < it->getPosition().getDistance( tempIt->getPosition() ) )
                {
                    closestIt = tempIt;
                    minDist = it->getPosition().getDistance( tempIt->getPosition() );
                }
            }

            it->addAdjacent( *closestIt );
            closestIt->addAdjacent( *it );
        }
    }
}


}   // namespace util

}   // namespace sauron