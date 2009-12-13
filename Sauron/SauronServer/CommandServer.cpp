#include "CommandServer.h"
#include "Sauron.h"
#include <string>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>

#include <iostream>

#include "log.h"
#define COMMAND_SERVER_LOG(level) FILE_LOG(level) << "CommandServer: "

namespace sauron
{

CommandServer::CommandServer( Sauron &sauron, ArServerHandlerMap* mapHandler )
    : mp_sauron( &sauron ), mp_mapHandler(mapHandler),
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

    boost::asio::ip::tcp::socket socket( service );
    //socket = new boost::asio::ip::tcp::socket( service );
    
    COMMAND_SERVER_LOG(logDEBUG4) << "Esperando conexao" << std::endl;
    acceptor.accept( socket );
    COMMAND_SERVER_LOG(logDEBUG4) << "Conexao estabelecida" << std::endl;

    std::string strInput;
    std::string statusCode;

    double x;
    double y;
    double theta;

    while ( m_serverRunningFlag )
    {
        COMMAND_SERVER_LOG(logDEBUG4) << "Recebendo dados..." << std::endl;
        int numBytes = socket.read_some( boost::asio::buffer( buffer ), error );
        COMMAND_SERVER_LOG(logDEBUG4) << "Numero de bytes recebidos: " << numBytes << std::endl;

        if ( error == boost::asio::error::eof || error )
        {
            COMMAND_SERVER_LOG(logDEBUG4) << "Conexao perdida" << std::endl;
            socket.close();
            /*delete socket;

            socket = new boost::asio::ip::tcp::socket( service );*/
            COMMAND_SERVER_LOG(logDEBUG4) << "Aguardando nova conexao" << std::endl;
            acceptor.accept( /***/socket );
            COMMAND_SERVER_LOG(logDEBUG4) << "Nova conexao realizada" << std::endl;
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
            COMMAND_SERVER_LOG(logDEBUG4) << "Navegando para " << strInput << std::endl;
            if ( mp_sauron->goToAsync( strInput ) )
                statusCode = "SUCESSO ESCOLHA DESTINO";
            else
                statusCode = "ERRO ESCOLHA DESTINO - JA ESTA NAVEGANDO";
            
        }
        else if ( strInput == "r" )
        {
            ss >> strInput >> theta;
            COMMAND_SERVER_LOG(logDEBUG4) << "Setando posicao no marco " << strInput << std::endl;
            if ( mp_sauron->setPose( strInput, (pose_t)theta ) )
                statusCode = "SUCESSO POSICAO MARCO";
            else
                statusCode = "ERRO POSICAO MARCO";
        }
        else if ( strInput == "p" )
        {
            ss >> x >> y >> theta;
            COMMAND_SERVER_LOG(logDEBUG4) << "Setando posicao " << x << ", " << y << ", " << theta << std::endl;
            mp_sauron->setPose( Pose(x, y, theta) );
            statusCode = "SUCESSO POSICAO ";
        }
        else if ( strInput == "i" )
        {
            ss >> strInput;
            COMMAND_SERVER_LOG(logDEBUG4) << "Definindo mapa" << strInput << std::endl;
            if ( mp_sauron->setInitialMap( strInput ) )
            {
                mp_mapHandler->loadMap(strInput.c_str());
                statusCode = "SUCESSO ESCOLHA MAPA";
            }
            else
                statusCode = "ERRO ESCOLHA MAPA";
        }
        else if ( strInput == "freeze" )
        {
            COMMAND_SERVER_LOG(logDEBUG4) << "Freeze" << std::endl;
            mp_sauron->freeze();
            statusCode = "SUCESSO FREEZE";
        }
        else if ( strInput == "halt" )
        {
            COMMAND_SERVER_LOG(logDEBUG4) << "Halt" << std::endl;
            mp_sauron->halt();
            statusCode = "SUCESSO HALT";
        }
        else
        {
            statusCode = "INVALIDO";
        }
        
        COMMAND_SERVER_LOG(logDEBUG4) << "Enviando status: " << statusCode << std::endl;
        socket.write_some( boost::asio::buffer( statusCode ), error );
        if ( error == boost::asio::error::eof || error )
        {
            COMMAND_SERVER_LOG(logDEBUG4) << "Conexao perdida" << std::endl;
            socket.close();
            /*delete socket;

            socket = new boost::asio::ip::tcp::socket( service );*/
            COMMAND_SERVER_LOG(logDEBUG4) << "Aguardando nova conexao" << std::endl;
            acceptor.accept( /***/socket );
            COMMAND_SERVER_LOG(logDEBUG4) << "Nova conexao realizada" << std::endl;
            continue;
        }

        COMMAND_SERVER_LOG(logDEBUG4) << "Status enviado" << std::endl;
    }

    socket.close();
    //delete socket;
}

}   // namespace sauron