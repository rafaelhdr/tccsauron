using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;

namespace SauronWPFController
{
    public class IPManager
    {
        private IPAddress ipAddress;

        public IPAddress IP
        { 
            get { return ipAddress; } 
            set { ipAddress = value; InvokeListeners(); }
        }

        private List<Action<IPAddress>> actions = new List<Action<IPAddress>>();

        public IPManager() { this.IP = IPAddress.Parse("127.0.0.1"); }


        public IPEndPoint GetCommandEndPoint()
        {
            return new IPEndPoint(IP, 5005);
        }

        public IPEndPoint GetStatusEndPoint()
        {
            return new IPEndPoint(IP, 5006);
        }

        private void InvokeListeners()
        {
            actions.ForEach( action => action.Invoke(ipAddress) );
        }


        public void AddListener(Action<IPAddress> listener)
        {
            this.actions.Add(listener);
        }




    }
}
