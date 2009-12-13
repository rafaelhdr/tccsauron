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
using System.Text.RegularExpressions;

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
        private RecebedorStatus recebedor;

        private Status status = Status.Parado;



        private bool close = false;
        private string goal;

        public NavigationWindow(EnviadorComandos enviador, RecebedorStatus recebedor)
        {
            InitializeComponent();
            this.enviador = enviador;
            this.recebedor = recebedor;
            this.Closing += new System.ComponentModel.CancelEventHandler(NavigationWindow_Closing);       
            lblErro.Visibility = Visibility.Hidden;
            this.cmbMapNames.ItemsSource = goalsNames.Keys;
            this.cmbMapNames.SelectedIndex = 0;
        }

        public void ForceClose()
        {
            close = true;
            this.Close();
        }

        void NavigationWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (!close)
            {
                e.Cancel = true;
                this.Hide();
            }
        }

        private void Erro(string message)
        {
            lblErro.Content = message;
            lblErro.Visibility = Visibility.Visible;
        }

        private void comboBox1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            lstGoals.ItemsSource = goalsNames[cmbMapNames.SelectedItem.ToString()];
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            lblErro.Visibility = Visibility.Hidden;
            if (status == Status.Navegando)
            {
                Erro("Erro! O robô já está navegando para " + goal);
            }
            else if (lstGoals.SelectedIndex < 0)
            {
                Erro("Erro! Selecione um destino para o robô.");
            }
            else
            {
                goal = lstGoals.SelectedItem.ToString();
                Thread execution = new Thread(new ThreadStart(this.Navega));
                execution.Start();
            }
        }



        private void Navega()
        {
            string result = enviador.Navigate(goal);
            DispatchStatus(result);
        }

        public void DispatchStatus(string result)
        {
            StatusDelegate del = new StatusDelegate(this.AtualizaStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }

        public void Continue()
        {
            if (!string.IsNullOrEmpty(goal) && goal != "-")
            {
                Thread execution = new Thread(new ThreadStart(this.Navega));
                execution.Start();
            }
            else
            {
                Erro("Não pode continuar algo que nunca começou.");
            }
        }

        public void AtualizaStatus(string result)
        {
            if (!string.IsNullOrEmpty(result))
            {
                result = result.Replace("\0", null);
                this.txtMsgRobo.Text += result + Environment.NewLine;
                this.txtMsgRobo.ScrollToEnd();

                Regex proximoRegex = new Regex("proximo ([^\\s]*)");
                if (proximoRegex.IsMatch(result))
                {
                    this.txtNextWaypoint.Content = proximoRegex.Match(result).Groups[1].Value;
                }

                // se chegou nao eh possivel continuar
                else if(result.Contains("CHEGOU_DESTINO"))
                {
                    status = Status.Parado;
                    txtStatus.Content = status.ToString();
                    goal = "-";
                    txtObjetivo.Content = goal;
                    this.txtNextWaypoint.Content = "-";
                }
                // se halt, freeze, ou parou por obstrução ou erro, então pode continuar, nao zera o goal
                else if (result.Contains("SUCESSO HALT") || result.Contains("SUCESSO FREEZE") || result.Contains("Erro de conexão") || result.Contains("OBSTRUIDO"))
                {
                    status = Status.Parado;
                    txtStatus.Content = status.ToString();
                    txtObjetivo.Content = goal;
                    this.txtNextWaypoint.Content = "-";
                }
                else if (result.Contains("SUCESSO ESCOLHA DESTINO"))
                {
                    status = Status.Navegando;
                    txtStatus.Content = status.ToString();
                    txtObjetivo.Content = goal;
                }

            }
        }
    }
}

