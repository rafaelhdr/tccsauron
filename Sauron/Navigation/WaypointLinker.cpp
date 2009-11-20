#include "WaypointLinker.h"
#include "Aria.h"
#include "log.h"
#include <iostream>
#include <algorithm>
#include <sstream>
#include "Sonar/Map.h"
#include "MathHelper.h"
#include "Sonar/LineSegment.h"
#include "AStar.h"

#define LINKER_LOG(level) FILE_LOG(level) << "WaypointLinker: " 

namespace sauron
{

namespace util
{

void WaypointLinker::link( Graph &graph, Map& map )
{
     Graph::iterator it;
    for ( it = graph.begin(); it != graph.end(); ++it )
        linkNodeToNearest( graph, *it, map );

	std::stringstream ss;
	for( it = graph.begin(); it != graph.end(); ++it )
	{
		ss << it->getName() << ": ";
		const std::vector<Node*>& adjacents = it->getAdjacents();
		for( std::vector<Node*>::const_iterator adIt = adjacents.begin();
			adIt != adjacents.end(); adIt++)
		{
			ss << (*adIt)->getName() << " ";
		}
		ss << std::endl;
	}
	std::cout << ss.str();
	LINKER_LOG(logINFO) << ss.str();
}

bool WaypointLinker::linkClosestPossibleRight( Graph& graph, Node& node, Map& map )
{
	Graph::iterator it;
	Graph::iterator closest;
	double minDistance = -1;
	for(it = graph.begin(); it != graph.end(); it++)
	{
		if(*it == node || it->Type() == Node::SECONDARY)
			continue;

		if(floating_point::isEqual(it->getPosition().Y(), node.getPosition().Y()))
		{
			if(it->getPosition().X() > node.getPosition().X())
			{
				if(isLinkPossible(*it, node, map))
				{
					double distance = node.getPosition().getDistance(it->getPosition());
					if(minDistance < 0 || minDistance > distance )
					{
						minDistance = distance;
						closest = it;
					}
				}
			}
		}
	}
	if(minDistance > 0)
	{
		node.addAdjacent(*closest);
		closest->addAdjacent(node);
		return true;
	}
	return false;
}
bool WaypointLinker::linkClosestPossibleLeft( Graph& graph, Node& node, Map& map )
{
	Graph::iterator it;
	Graph::iterator closest;
	double minDistance = -1;
	for(it = graph.begin(); it != graph.end(); it++)
	{
		if(*it == node || it->Type() == Node::SECONDARY)
			continue;

		if(floating_point::isEqual(it->getPosition().Y(), node.getPosition().Y()))
		{
			if(it->getPosition().X() < node.getPosition().X())
			{
				if(isLinkPossible(*it, node, map))
				{
					double distance = node.getPosition().getDistance(it->getPosition());
					if(minDistance < 0 || minDistance > distance )
					{
						minDistance = distance;
						closest = it;
					}
				}
			}
		}
	}
	if(minDistance > 0)
	{
		node.addAdjacent(*closest);
		closest->addAdjacent(node);
		return true;
	}
	return false;
}
bool WaypointLinker::linkClosestPossibleUp( Graph& graph, Node& node, Map& map )
{
	Graph::iterator it;
	Graph::iterator closest;
	double minDistance = -1;
	for(it = graph.begin(); it != graph.end(); it++)
	{
		if(*it == node || it->Type() == Node::SECONDARY)
			continue;

		if(floating_point::isEqual(it->getPosition().X(), node.getPosition().X()))
		{
			if(it->getPosition().Y() > node.getPosition().Y())
			{
				if(isLinkPossible(*it, node, map))
				{
					double distance = node.getPosition().getDistance(it->getPosition());
					if(minDistance < 0 || minDistance > distance )
					{
						minDistance = distance;
						closest = it;
					}
				}
			}
		}
	}
		if(minDistance > 0)
		{
			node.addAdjacent(*closest);
			closest->addAdjacent(node);
			return true;
		}
		return false;
}
bool WaypointLinker::linkClosestPossibleDown( Graph& graph, Node& node, Map& map )
{
	Graph::iterator it;
	Graph::iterator closest;
	double minDistance = -1;
	for(it = graph.begin(); it != graph.end(); it++)
	{
		if(*it == node || it->Type() == Node::SECONDARY)
			continue;

		if(floating_point::isEqual(it->getPosition().X(), node.getPosition().X()))
		{
			if(it->getPosition().Y() < node.getPosition().Y())
			{
				if(isLinkPossible(*it, node, map))
				{
					double distance = node.getPosition().getDistance(it->getPosition());
					if(minDistance < 0 || minDistance > distance )
					{
						minDistance = distance;
						closest = it;
					}
				}
			}
	}
	}
	if(minDistance > 0)
	{
		node.addAdjacent(*closest);
		closest->addAdjacent(node);
		return true;
	}
	return false;
}

bool WaypointLinker::isLinkPossible(Node& node1, Node& node2, Map& map)
{
	LineSegment segmentBetweenNodes(node1.getPosition().X(), node1.getPosition().Y(),
		node2.getPosition().X(), node2.getPosition().Y());
	std::vector<LineSegment>& lines = *map.getLines();
	for(std::vector<LineSegment>::iterator it = lines.begin(); it != lines.end(); it++)
	{
		ArPose pose;
		if(segmentBetweenNodes.intersects(&(*it), &pose)) {
			return false;
		}
	}
	return true;
}

void WaypointLinker::linkNodeToNearest( Graph &graph, Node &toLink, Map& map )
{
    if ( toLink.Type() == Node::PRIMARY )
    {
		bool down = linkClosestPossibleDown(graph, toLink, map);
		bool left = linkClosestPossibleLeft(graph, toLink, map);
		bool right = linkClosestPossibleRight(graph, toLink, map);
		bool up = linkClosestPossibleUp(graph, toLink, map);
		if(!(up || down || left || right)){
			std::cerr << "Ah, poxa! Nao linkou o no' " << toLink.getName() << "." << std::endl;
		}
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

            if ( tempIt->Type() == Node::SECONDARY || 
				!isLinkPossible(*tempIt, toLink, map))
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
			closestIt->addAdjacent( toLink );
		}
	}
}

void WaypointLinker::linkTemporaryNode( Graph &graph, Node &tempNode, const Node &goal, Map& map )
{
	// a ideia é nunca linkar a um nó que nos deixará mais longe de onde já estamos
	// para isso, pegamos a distância até o nó diretamente alcançável (isto é, que enxergamos
	// sem precisar atravessar paredes) mais próximo do destino.

	Graph::iterator closestToGoalIt;
	Graph::iterator closestIt;
	Graph::iterator tempIt;

	double minDistanceToGoal = -1;

	for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
	{
		if(*tempIt == tempNode)
			continue;

		if ( tempIt->Type() == Node::SECONDARY && *tempIt != goal )
			continue;

		if( !isLinkPossible( tempNode, *tempIt, map ) )
			continue;

		Path pathFromCurrentToGoal = AStar::searchPath(*tempIt, goal);

		if(pathFromCurrentToGoal.size() == 0 && *tempIt != goal)
			continue;

		double distFromCurrentToGoal = getPathLength(pathFromCurrentToGoal);

		if( minDistanceToGoal < 0 || minDistanceToGoal > distFromCurrentToGoal)
		{
			closestToGoalIt = tempIt;
			minDistanceToGoal =  distFromCurrentToGoal;
		}
	}

	if(minDistanceToGoal < 0)
	{
		std::cerr << "Ah, poxa! Nao achei ninguem perto do objetivo." << std::endl;
		return;
	}

	// o nó mais próximo do destino é apontado por closestToGoalIt.
	double distToClosest = closestToGoalIt->getPosition().getDistance(tempNode.getPosition());

	// agora, percorremos todos os nós do grafo e escolhemos o mais próximo que satisfaça duas
	// condições
	// 1) Seja possível (isLinkPossible)
	// 2) Não nos leve para mais longe do que já estamos (usando distToClosest)

	double minDistFromTemp = -1;
	for ( tempIt = graph.begin(); tempIt != graph.end(); ++tempIt )
	{
		if(*tempIt == tempNode)
			continue;

		if ( tempIt->Type() == Node::SECONDARY && *tempIt != goal )
			continue;

		if( !isLinkPossible( tempNode, *tempIt, map ) )
			continue;

		double distFromTempToCurrent = tempIt->getPosition().getDistance(tempNode.getPosition());
		if(minDistFromTemp < 0 || minDistFromTemp > distFromTempToCurrent)
		{
			closestIt = tempIt;
			minDistFromTemp = distFromTempToCurrent;
		}
	}

	if(minDistFromTemp < 0)
	{
		std::cerr << "Ah, poxa! Nao achei nenhum ponto perto de mim. A discussao e' pontual. Ou nao." << std::endl;
	} else {
		tempNode.addAdjacent( *closestIt );
		LINKER_LOG(logINFO) << "TempLink escolhido: " << closestIt->getName() << std::endl;
	}
}

pose_t WaypointLinker::getPathLength(const sauron::Path &path)
{
	pose_t length = 0;
	for(int i = 0; i < static_cast<int>(path.size()) - 1; i++)
	{
		length += path[i].getPosition().getDistance(path[i+1].getPosition());
	}
	return length;
}




}   // namespace util

}   // namespace sauron