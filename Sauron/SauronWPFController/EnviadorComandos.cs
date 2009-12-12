using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;

namespace SauronWPFController
{
    public class EnviadorComandos
    {

        public IPAddress IP { get; set; }

        private const int port = 5005;

        private Socket socket;

       
        private Socket Socket
        {
            get {

                
                if (socket == null)
                    socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.IP);

                try
                {
                    // se estiver conectado e o ip for certo, nem entra
                    if (!(socket.Connected && ((IPEndPoint)socket.RemoteEndPoint).Address == IP))
                    {
                        if (socket.Connected)
                        {
                            socket.Disconnect(true);
                        }

                        EndPoint endPoint = new IPEndPoint(IP, port);
                        socket.Connect(endPoint);
                    }
                }
                catch (Exception)
                {
                    return null;
                }

                return socket;
                }
            
        }


        public string Navigate(string markName)
        {
            string result = "Erro! Não conseguiu conectar no robô.";

            if (Socket != null)
            {
                Send("n " + markName);
                result = Receive();
            }

            return result;
        }


        private void Send(string data)
        {
            byte[] byteData = System.Text.Encoding.ASCII.GetBytes(data);
            Socket.Send(byteData);
        }

        private string Receive()
        {
            byte [] buffer = new byte[1024];
            int received = Socket.Receive(buffer);
            char[] chars = new char[received + 1];

            System.Text.Encoding.ASCII.GetDecoder().GetChars(buffer, 0,  received, chars, 0);

            return new String(chars);
        }

    }
}
