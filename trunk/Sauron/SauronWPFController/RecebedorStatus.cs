using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace SauronWPFController
{
    public class RecebedorStatus
    {
        private Socket socket;
        private IPManager ipManager;

        private List<Action<string>> listeners = new List<Action<string>>();
        public Action<string> ReceiveAction { get; set; }

        public RecebedorStatus(IPManager ipManager)
        {
            this.ipManager = ipManager;
            this.ipManager.AddListener(AtualizaSocket);
            Thread execution = new Thread(new ThreadStart(BeginReceive));
            execution.Start();
        }

        private void AtualizaSocket(IPAddress ip)
        {
            socket.Disconnect(true);
            socket.Close();
            socket = null;
            InitializeSocket();
        }

        public void AddListener(Action<string> action)
        {
            this.listeners.Add(action);
        }

        private void InitializeSocket()
        {
            if (socket == null)
            {
                try
                {
                    socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.IP);
                    socket.Connect(ipManager.GetStatusEndPoint());
                }
                catch (Exception e)
                {
                    socket = null;
                }
            }
        }

        private void BeginReceive()
        {
            while (true)
            {
                InitializeSocket();
                string result = Receive();
                if(ReceiveAction != null && result != null)
                    ReceiveAction(result);
            }
        }

        private string Receive()
        {
            try
            {
                byte[] buffer = new byte[128];
                int received = socket.Receive(buffer, 0, buffer.Length, SocketFlags.None);
                char[] chars = new char[received + 1];
                System.Text.Encoding.ASCII.GetDecoder().GetChars(buffer, 0, received, chars, 0);
                return new String(chars);
            }
            catch (Exception ex)
            {
                return null;
            }
        }
    }
}
