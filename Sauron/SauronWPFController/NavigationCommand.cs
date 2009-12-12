using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Threading;

namespace SauronWPFController
{
    public class NavigationCommand
    {
        private string goal;
        private Dispatcher dispatcher;
        private EnviadorComandos enviador;
        private delegate void SimpleDelegate(string result);

        public NavigationCommand(EnviadorComandos enviador, string goal, Dispatcher dispatcher)
        {
            this.goal = goal;
            this.enviador = enviador;
            this.dispatcher = dispatcher;
        }

        public void Navigate()
        {
            string result = enviador.Navigate(goal);
           
        }

      


    }
}
