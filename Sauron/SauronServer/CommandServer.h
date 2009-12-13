#ifndef __COMMAND_SERVER_H__
#define __COMMAND_SERVER_H__

#include "ArNetworking.h"
#include <boost/thread/thread.hpp>

namespace sauron
{

class Sauron;

class CommandServer
{
    public:
        CommandServer( Sauron &sauron, ArServerHandlerMap* mapHandler );
	    ~CommandServer();

        void start();
        void stop();

        void commandServerLoop();

    private:
        boost::thread   m_thread;
        
        Sauron *mp_sauron;
        ArServerHandlerMap* mp_mapHandler;

        bool m_serverRunningFlag;
};

}   // namespace sauron

#endif // __COMMAND_SERVER_H__
