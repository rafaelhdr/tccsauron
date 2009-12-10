#include "MapFileParser.h"
#include <fstream>
#include <sstream>
#include <vector>
#include "Map.h"
#include "LineSegment.h"

namespace sauron
{

namespace util
{

bool MapFileParser::loadWaypoints( Map* p_map, Graph &graph )
{
    graph.clear();

    std::ifstream file;
	file.open( p_map->getOriginalMapFilename().c_str() );
    if ( !file.is_open() )
        return false;

    std::string line;
    while ( !file.eof() )
    {
        std::getline( file, line );
        if ( ( line.find( "Dock" ) != std::string::npos || 
               line.find( "Goal" ) != std::string::npos ) &&
               line.find( "MapInfo" ) == std::string::npos )
        {
            std::string temp;

            pose_t x;
            pose_t y;
            Node::NodeType type;

            std::stringstream ss( line );

            // removing "Cairn: "
            ss >> temp;   

            // checking waypoint type
            ss >> temp;
            if ( temp == "Dock" )
                type = Node::PRIMARY;
            else
                type = Node::SECONDARY;

            // reading position
            ss >> x >> y;
			x /= 10.0;
			y /= 10.0;

			// reading theta (useless), "", ICON
			ss >> temp >> temp >> temp;

			// read nodeName (this probably means there can't be any spaces in the node name)
			std::string nodeName;
			ss >> nodeName;

			// remove quotes around name
			nodeName = nodeName.substr(1, nodeName.size() - 2);

            graph.push_back( Node( Point2D<pose_t>(x, y), type, nodeName) );
        }
		else if(line.find( "ForbiddenLine" ) != std::string::npos)
		{
			std::stringstream ss( line );
			std::string temp;

			// Cairn: ForbiddenLine x y theta "" ICON "Esacda" 28008 10143 31062 10143
			ss >> temp >> temp >> temp >> temp >> temp >> temp >> temp >> temp;

			pose_t x1, y1, x2, y2;
			ss >> x1 >> y1 >> x2 >> y2;
			x1 /= 10;
			y1 /= 10;
			x2 /= 10;
			y2 /= 10;

			std::vector<LineSegment>* pforbiddenLines = p_map->getForbiddenLines();
			pforbiddenLines->push_back(LineSegment(ArLineSegment(x1, y1, x2, y2)));
		}
    }

    file.close();

    return true;
}

}   // namespace util

}   // namespace sauron