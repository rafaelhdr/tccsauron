using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Threading;

namespace SauronWPFController
{
    /// <summary>
    /// Interaction logic for NavigationWindow.xaml
    /// </summary>
    public partial class NavigationWindow : Window
    {

        private enum Status
        {
            Parado,
            Navegando,
            Congelado
        }


        private Dictionary<string, List<string>> goalsNames = new GoalsFinder().GetGoalsNames();
        private EnviadorComandos enviador;
        private Status status = Status.Parado;
        private delegate void SimpleDelegate(string result);
        private string goal;

        public NavigationWindow(EnviadorComandos enviador)
        {
            InitializeComponent();
            this.enviador = enviador;
            this.cmbMapNames.ItemsSource = goalsNames.Keys;
            this.cmbMapNames.SelectedIndex = 0;
        }

        private void comboBox1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            lstGoals.ItemsSource = goalsNames[cmbMapNames.SelectedItem.ToString()];
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            if(status == Status.Navegando)
            {
                // erro navegando
            }
            else if(lstGoals.SelectedIndex < 0)
            {
                // erro seleção
            }
            else{
                goal = lstGoals.SelectedItem.ToString();
                status = Status.Navegando;
                txtStatus.Content = status.ToString();
                txtObjetivo.Content = goal;
               Thread execution = new Thread(new ThreadStart(this.Navega));
               execution.Start();
                
            }
        }

        private void Navega()
        {
            string result = enviador.Navigate(goal);
            SimpleDelegate del = new SimpleDelegate(this.AtualizaStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }


        private void AtualizaStatus(string result)
        {
            this.txtMsgRobo.Content = result;
        }

    }
}

