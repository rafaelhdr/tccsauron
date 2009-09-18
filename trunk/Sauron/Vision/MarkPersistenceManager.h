#ifndef __MARK_PERSISTENCE_MANAGER_H__
#define __MARK_PERSISTENCE_MANAGER_H__

#include <string>
#include <fstream>
#include "Mark.h"

namespace sauron
{

class MarkPersistenceManager
{
    public:
        static void saveToFile( const std::string &filename, const MarkVector &marks );
        static void loadFromFile( const std::string &filename, MarkVector &marks );
};

}   // namespace sauron

#endif  // __MARK_PERSISTENCE_MANAGER_H__