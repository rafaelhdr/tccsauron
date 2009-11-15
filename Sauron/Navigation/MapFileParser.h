#ifndef __MAP_FILE_PARSER_H__
#define __MAP_FILE_PARSER_H__

#include <string>
#include "Node.h"

namespace sauron
{

namespace util
{

class MapFileParser
{
    public:
        static bool loadWaypoints( const std::string &filename, Graph &graph );
};

}   // namespace util

}   // namespace sauron

#endif  //  __MAP_FILE_PARSER_H__
