#include "NodesPersistence.h"
#include <fstream>
#include <map>

namespace sauron
{

bool NodesPersistence::saveToFile( const sauron::Graph &graph, const std::string &filename )
{
    std::ofstream file;
    file.open( filename.c_str() );
    if ( !file.is_open() )
        return false;

    int numNodes = graph.size();
    file.write( (const char *)&numNodes, sizeof( numNodes ) );

    std::map<Node, int> nodeIdMap;
    int index = 0;

    Graph::const_iterator it;
    for ( it = graph.begin(); it != graph.end(); ++it )
    {
        nodeIdMap[ *it ] = index++;

        pose_t x = it->getPosition().X();
        pose_t y = it->getPosition().Y();
        Node::NodeType type = it->Type();
        int numAdjs = it->getAdjacents().size();

        file.write( (const char *)&x,       sizeof( x ) );
        file.write( (const char *)&y,       sizeof( y ) );
        file.write( (const char *)&type,    sizeof( type ) );
        file.write( (const char *)&numAdjs, sizeof( numAdjs ) ); 
    }

    for ( it = graph.begin(); it != graph.end(); ++it )
    {
        std::vector<Node *>::const_iterator adjIt;
        const std::vector<Node *> adjVec = it->getAdjacents();
        for ( adjIt = adjVec.begin(); adjIt != adjVec.end(); ++adjIt )
        {
            int id = nodeIdMap[ **adjIt ];
            file.write( (const char *)&id, sizeof( id ) );
        }
    }

    file.close();

    return true;
}


bool NodesPersistence::loadFromFile( sauron::Graph &graph, const std::string &filename )
{
    std::ifstream file;
    file.open( filename.c_str() );
    if ( !file.is_open() )
        return false;

    int numNodes;
    file.read( (char *)&numNodes, sizeof( numNodes ) );

    graph.clear();

    std::vector<int> adjsNumVec;

    for ( register int i = 0; i < numNodes; ++i )
    {
        pose_t x;
        pose_t y;
        Node::NodeType type;
        int numAdjs;

        file.read( (char *)&x,       sizeof( x ) );
        file.read( (char *)&y,       sizeof( y ) );
        file.read( (char *)&type,    sizeof( type ) );
        file.read( (char *)&numAdjs, sizeof( numAdjs ) );

        graph.push_back( Node( Point2D<pose_t>( x, y ), type ) );       
        adjsNumVec.push_back( numAdjs );
    }

    for ( register int i = 0; i < numNodes; ++i )
    {
        for ( register int j = 0; j < adjsNumVec[i]; ++j )
        {
            int id;
            file.read( (char *)&id, sizeof( id ) );

            graph[i].addAdjacent( graph[id] );
        }
    }

    return true;
}

}   // namespace sauron