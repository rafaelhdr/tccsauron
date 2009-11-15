#include <iostream>
#include <string>
#include "Node.h"
#include "MapFileParser.h"
#include "WaypointLinker.h"
#include "NodesPersistence.h"

using namespace sauron;
using namespace sauron::util;

int main( int argc, char *argv[] )
{
    Graph graph;

    std::string mapFilename;

    std::cout << "Arquivo do mapa: ";
    //std::cin >> mapFilename;
    mapFilename = "pavsup_mod.map";

    MapFileParser::loadWaypoints( mapFilename, graph );
    WaypointLinker::link( graph );
    NodesPersistence::saveToFile(  graph, "waypoints.txt" );
}