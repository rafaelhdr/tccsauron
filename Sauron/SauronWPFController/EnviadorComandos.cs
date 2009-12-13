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
        private Socket socket;
        private IPManager ipManager;

        public EnviadorComandos(IPManager ipManager)
        {
            this.ipManager = ipManager;
        }

        private void InitializeSocket()
        {
            if (socket == null)
            {
                socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.IP);
                socket.Connect(ipManager.GetCommandEndPoint());
            }
            else if (!socket.Connected)
            {
                socket.Connect(ipManager.GetCommandEndPoint());
            }
        }
        
        public string Navigate(string markName)
        {
            try
            {

                InitializeSocket();
                Send("n " + markName);
            }
            catch (Exception e)
            {
                return "Erro de conexão: "+e.Message;
            }

            return Receive();     
        }

        public string Freeze()
        {
            InitializeSocket();
            Send("freeze");
            return Receive();
        }

        public string Halt()
        {
            InitializeSocket();
            Send("halt");
            return Receive();
        }

        public string Position(double x, double y, double theta)
        {
            InitializeSocket();
            Send("p "+x+" "+y+" "+theta);
            return Receive();
        }

        public string Mark(string markName, double theta)
        {
            InitializeSocket();
            Send("r " + markName + " " + theta);
            return Receive();
        }

        public string Map(string mapName)
        {
            InitializeSocket();
            Send("i " + mapName);
            return Receive();
        }


        private void Send(string data)
        {
            byte[] byteData = System.Text.Encoding.ASCII.GetBytes(data);
            socket.Send(byteData);
        }

        private string Receive()
        {
            try
            {
                byte[] buffer = new byte[100];
                int received = socket.Receive(buffer, 0, buffer.Length, SocketFlags.None);
                char[] chars = new char[received + 1];
                socket.Close();
                socket = null;
                System.Text.Encoding.ASCII.GetDecoder().GetChars(buffer, 0, received, chars, 0);
                return new String(chars);
            }
            catch (Exception e)
            {
                return "Erro de Recepção:" + e.Message;
            }
        }

    }
}
