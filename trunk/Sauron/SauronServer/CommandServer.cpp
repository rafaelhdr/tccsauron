#include "CommandServer.h"
#include "Sauron.h"
#include <string>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>

#include <iostream>

namespace sauron
{

CommandServer::CommandServer( Sauron &sauron )
    : mp_sauron( &sauron ),
      m_serverRunningFlag( false )
{
}

CommandServer::~CommandServer(void)
{
    stop();
}

void CommandServer::start()
{
    if ( !m_serverRunningFlag )
    {
        m_thread = boost::thread( &CommandServer::commandServerLoop, this );
        m_serverRunningFlag = true;
    }
}


void CommandServer::stop()
{
    if ( m_serverRunningFlag )
    {
        m_serverRunningFlag = false;
        m_thread.join();
    }
}


void CommandServer::commandServerLoop()
{
    boost::asio::io_service         service;
    boost::asio::ip::tcp::acceptor  acceptor( service, boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), 5005 ) );
    boost::array< char, 128 >       buffer;
    boost::system::error_code       error;

    service.run();

    boost::asio::ip::tcp::socket *socket;
    socket = new boost::asio::ip::tcp::socket( service );

    acceptor.accept( *socket );

    std::string strInput;
    std::string statusCode;

    double x;
    double y;
    double theta;

    while ( m_serverRunningFlag )
    {
        int numBytes = socket->read_some( boost::asio::buffer( buffer ), error );
        if ( error == boost::asio::error::eof || error )
        {
            socket->close();
            delete socket;

            socket = new boost::asio::ip::tcp::socket( service );
            acceptor.accept( *socket );  
            continue;
        }

        std::stringstream ss;
        buffer.data()[numBytes] = '\0';
        ss << buffer.c_array();
        
        ss >> strInput;
        boost::to_lower( strInput );

        if ( strInput == "n" )
        {
            ss >> strInput;
            mp_sauron->goToAsync( strInput );
            statusCode = "SUCESSO";
        }
        else if ( strInput == "r" )
        {
            ss >> strInput >> theta;
            if ( mp_sauron->setPose( strInput, (pose_t)theta ) )
                statusCode = "SUCESSO";
            else
                statusCode = "ERRO";
        }
        else if ( strInput == "p" )
        {
            ss >> x >> y >> theta;
            mp_sauron->setPose( Pose(x, y, theta) );
            statusCode = "SUCESSO";
        }
        else if ( strInput == "i" )
        {
            ss >> strInput;
            if ( mp_sauron->setInitialMap( strInput ) )
                statusCode = "SUCESSO";
            else
                statusCode = "ERRO";
        }
        else if ( strInput == "freeze" )
        {
            mp_sauron->freeze();
            statusCode = "SUCESSO";
        }
        else if ( strInput == "halt" )
        {
            mp_sauron->halt();
            statusCode = "SUCESSO";
        }
        else
        {
            statusCode = "INVALIDO";
        }
        
        socket->write_some( boost::asio::buffer( statusCode ) );
    }

    socket->close();
    delete socket;
}

}   // namespace sauron