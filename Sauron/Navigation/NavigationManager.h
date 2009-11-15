#ifndef __NAVIGATION_MANAGER_H__
#define __NAVIGATION_MANAGER_H__

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "AStar.h"
#include "NodesPersistence.h"

class ArRobot;

namespace sauron
{

class NavigationManager
{
    public:
        NavigationManager();
        ~NavigationManager();

        void mainLoop();

    private:
        boost::thread   m_thread;

        ArRobot *m_robot;

        double  m_maxSpeed;
        double  m_maxRotSpeed;
        double  m_currentSpeed;
        double  m_currentRotSpeed;
};

}   // namespace sauron

#endif  // __NAVIGATION_MANAGER_H__
