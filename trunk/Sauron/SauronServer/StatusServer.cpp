#include "StatusServer.h"
#include <string>
#include <sstream>

namespace sauron
{

StatusServer::StatusServer( Sauron &sauron )
    : mp_sauron( &sauron ),
      m_serverRunningFlag( false ),
      mp_socket( NULL )
{
}

StatusServer::~StatusServer()
{
    stop();
}


void StatusServer::start()
{
    if ( !m_serverRunningFlag )
    {
        m_pathCallbackId = mp_sauron->getMapPlanner()->getPathPlanner().addPathplanningCallback( 
            boost::bind( &StatusServer::statusServerPathCallback, this, _1, _2 ) );

        m_mapCallbackId = mp_sauron->getMapPlanner()->addMapPlannerCallback( 
            boost::bind( &StatusServer::statusServerMapCallback, this, _1, _2 ) );

        m_thread = boost::thread( &StatusServer::statusServerLoop, this );
        m_serverRunningFlag = true;       
    }
}


void StatusServer::stop()
{
    if ( m_serverRunningFlag )
    {
        mp_sauron->getMapPlanner()->getPathPlanner().removePathplanningCallback( m_pathCallbackId );
        mp_sauron->getMapPlanner()->removeMapPlannerCallback( m_mapCallbackId );

        if ( mp_socket )
        {
            delete mp_socket;
            mp_socket = NULL;
        }

        m_serverRunningFlag = false;
        m_thread.join();
    }
}


void StatusServer::statusServerLoop()
{
    boost::asio::io_service         service;
    boost::asio::ip::tcp::acceptor  acceptor( service, boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), 5006 ) );
    boost::system::error_code       error;

    service.run();

    while ( m_serverRunningFlag )
    {
        if ( mp_socket )
            boost::thread::yield();
        else
        {
            mp_socket = new boost::asio::ip::tcp::socket( service );
            acceptor.accept( *mp_socket );
        }
    }
}


void StatusServer::statusServerMapCallback( MapPlannerStatus status, const Map *map )
{
    boost::system::error_code  error;
    std::string toSend;

    if ( !mp_socket )
        return;

    switch( status )
	{
	    case GOAL_SAME_MAP:
            toSend = "nav_status MESMO_MAPA";
		    break;

	    case GOAL_OTHER_MAP:
            toSend = "nav_status OUTRO_MAPA";
		    break;

	    case MAP_CHANGED:
            toSend = "nav_status MUDOU_MAPA";
		    break;

	    default:
		    break;
	}

    if ( toSend.size() )
    {
        mp_socket->write_some( boost::asio::buffer( toSend ), error );
        if ( error )
        {
            mp_socket->close();
            delete mp_socket;
            mp_socket = NULL;
        }
    }
}


void StatusServer::statusServerPathCallback( PathPlannerStatus status, const Node *node )
{
    boost::system::error_code  error;
    std::string toSend;

    if ( !mp_socket )
        return;

    switch( status )
	{
	    case GOING_TO_WAYPOINT:
            toSend = std::string( "proximo " ) + node->getName();            
		    break;

	    case ARRIVED_AT_WAYPOINT:
            toSend = "nav_status CHEGOU_WAYPOINT";
		    break;

	    case GOAL_REACHED:
            toSend = "nav_status CHEGOU_DESTINO";
		    break;

	    case FAILED_COLLISION_AVOIDANCE:
            toSend = "nav_status EVITOU_COLISAO";
            break;

	    case FAILED_STRAYED:
            toSend = "nav_status DESVIOU";
            break;

        case FAILED_OBSTRUCTED_PATH:
            toSend = "nav_status OBSTRUIDO";
            break;

	    default:
		    break;
    }

    if ( toSend.size() )
    {
        mp_socket->write_some( boost::asio::buffer( toSend ), error );
        if ( error )
        {
            mp_socket->close();
            delete mp_socket;
            mp_socket = NULL;
        }
    }   
}


}   // namespace sauron