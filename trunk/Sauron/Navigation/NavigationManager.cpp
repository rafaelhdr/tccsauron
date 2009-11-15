#include "NavigationManager.h"

namespace sauron
{

NavigationManager::NavigationManager(void)
{
}

NavigationManager::~NavigationManager(void)
{
    m_thread.join();
}

}   //  namespace sauron