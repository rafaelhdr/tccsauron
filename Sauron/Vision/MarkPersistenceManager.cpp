
#include "MarkPersistenceManager.h"

namespace sauron
{

void MarkPersistenceManager::saveToFile( const std::string &filename, const sauron::MarkVector &marks )
{
    if ( marks.empty() )
        return; // TODO Warn that the vector is empty

    std::ofstream file;
    file.open( filename.c_str() );
    if ( !file.is_open() )
        return; // TODO Throw an exception

    file << marks.size();

    MarkVector::const_iterator it;
    for ( it = marks.begin(); it != marks.end(); ++it )
        Mark::persist( *it, file );


    file.close();
}


void MarkPersistenceManager::loadFromFile( const std::string &filename, sauron::MarkVector &marks )
{
    std::ifstream file;
    file.open( filename.c_str() );
    if ( !file.is_open() )
        return;  // TODO Throw an exception

    marks.clear();

    uint numMarks;
    file >> numMarks;

    for ( register uint i = 0; i < numMarks; ++i )
        marks.push_back( Mark::restore( file ) );

    file.close();
}

}   // namespace sauron