#include "WaypointLinker.h"
#include "Aria.h"
#include "log.h"
#include <iostream>
#include <algorithm>
#include <sstream>
#include "Sonar/Map.h"
#include "MathHelper.h"
#include "Sonar/LineSegment.h"

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