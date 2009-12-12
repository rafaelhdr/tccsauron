#ifndef __STATUS_SERVER_H__
#define __STATUS_SERVER_H__

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "Sauron.h"

namespace sauron
{

class StatusServer
{
    public:
        StatusServer( Sauron &sauron );
        ~StatusServer();

        void start();
        void stop();
        
        void statusServerLoop();
        void statusServerMapCallback( MapPlannerStatus status, const Map* map );
        void statusServerPathCallback( PathPlannerStatus status, const Node *node );

    private:
        Sauron *mp_sauron;

        boost::thread                 m_thread;
        boost::asio::ip::tcp::socket *mp_socket;

        int m_pathCallbackId;
        int m_mapCallbackId;

        bool m_serverRunningFlag;
};

}   // namespace sauron

#endif  // __STATUS_SERVER_H__
