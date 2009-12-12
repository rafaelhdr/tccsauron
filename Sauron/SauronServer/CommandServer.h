#ifndef __COMMAND_SERVER_H__
#define __COMMAND_SERVER_H__

#include <boost/thread/thread.hpp>

namespace sauron
{

class Sauron;

class CommandServer
{
    public:
	    CommandServer( Sauron &sauron );
	    ~CommandServer();

        void start();
        void stop();

        void commandServerLoop();

    private:
        boost::thread   m_thread;
        
        Sauron *mp_sauron;

        bool m_serverRunningFlag;
};

}   // namespace sauron

#endif // __COMMAND_SERVER_H__
