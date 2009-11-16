#include "MapFileParser.h"
#include <fstream>
#include <sstream>


namespace sauron
{

namespace util
{

bool MapFileParser::loadWaypoints( const std::string &filename, Graph &graph )
{
    graph.clear();

    std::ifstream file;
    file.open( filename.c_str() );
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
    }

    file.close();

    return true;
}

}   // namespace util

}   // namespace sauron