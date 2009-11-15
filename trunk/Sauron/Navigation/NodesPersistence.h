#ifndef __NAVIGATION_NODES_PERSISTENCE_H__
#define __NAVIGATION_NODES_PERSISTENCE_H__

#include "Node.h"

namespace sauron
{

class NodesPersistence
{
    public:
        static bool saveToFile( const Graph &graph, const std::string &filename );
        static bool loadFromFile( Graph &graph, const std::string &filename );
};

}   // namespace sauron

#endif  // __NAVIGATION_NODES_PERSISTENCE_H__