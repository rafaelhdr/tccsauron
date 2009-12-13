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
using System.Globalization;

namespace SauronWPFController
{
    /// <summary>
    /// Interaction logic for PositionConfigurator.xaml
    /// </summary>
    public partial class PositionConfigurator : Window
    {

        private EnviadorComandos enviador;
        private Dictionary<string, List<string>> goalsNames = new GoalsFinder().GetWaypoints();

        string mapName;
        string markName;
        double x, y, theta;

        public PositionConfigurator(EnviadorComandos enviador)
        {
            InitializeComponent();
            this.enviador = enviador;

            clearLabels();

            this.cmbMapas.ItemsSource = goalsNames.Keys;
            this.cmbMapas.SelectedIndex = 0;


            string sCurrentCulture = System.Threading.Thread.CurrentThread.CurrentCulture.Name;
            CultureInfo ci = new CultureInfo(sCurrentCulture);
            ci.NumberFormat.NumberDecimalSeparator = ".";
            System.Threading.Thread.CurrentThread.CurrentCulture = ci;
        }

        private void clearLabels()
        {
            this.lblErroCoordenadas.Visibility = Visibility.Hidden;
            this.lblErroMarco.Visibility = Visibility.Hidden;
            this.lblSucessoCoordenadas.Visibility = Visibility.Hidden;
            this.lblSucessoMarco.Visibility = Visibility.Hidden;
        }

        private void cancel_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private void cmbMapas_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            lstObjetivos.ItemsSource = goalsNames[cmbMapas.SelectedItem.ToString()];
        }

        private void btnSetMarkPosition_Click(object sender, RoutedEventArgs e)
        {
            clearLabels();
            try
            {
                theta = double.Parse(txtAngulo.Text);
            }
            catch(Exception)
            {
                lblErroMarco.Content = "Erro! Valor de ângulo inválido.";
                lblErroMarco.Visibility = Visibility.Visible;
                return;
            }

            if (lstObjetivos.SelectedIndex < 0)
            {
                lblErroMarco.Content = "Erro! Nenhum marco selecionado.";
                lblErroMarco.Visibility = Visibility.Visible;
                return;
            }

            mapName = cmbMapas.SelectedItem.ToString();
            markName = lstObjetivos.SelectedItem.ToString();
            Thread execution = new Thread(new ThreadStart(this.SetMarkPosition));
            execution.Start();
        }


        

        private void SetMarkPosition()
        {

            string result = enviador.Map(mapName+".map");
            if (result.Contains("SUCESSO"))
            {
                result = enviador.Mark(markName, theta);
                DispatchMarkResult(result);
            }
            else
            {
                DispatchMarkResult(result);
            }
            
        }

        private void DispatchMarkResult(string result)
        {
            StatusDelegate del = new StatusDelegate(this.AtualizaMarkStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }

        private void AtualizaMarkStatus(string result)
        {
            if (result.Contains("SUCESSO"))
            {
                lblSucessoMarco.Content = "Posição atualizada com sucesso!";
                lblSucessoMarco.Visibility = Visibility.Visible;
            }
            else 
            {
                if(result.Contains("ERRO ESCOLHA MAPA"))
                {
                    lblErroMarco.Content = "Erro ao atualizar mapa.";
                }

                else if (result.Contains("ERRO POSICAO MARCO"))
                {
                    lblErroMarco.Content = "Erro ao atualizar posição.";
                }
                else
                {
                    lblErroMarco.Content = "Erro ao enviar comando.";
                }
                lblErroMarco.Visibility = Visibility.Visible;
            }
        }


        private void btnSetCoordinatesPosition_Click(object sender, RoutedEventArgs e)
        {
            clearLabels();
            try
            {
                x = double.Parse(txtPositionX.Text);
                y = double.Parse(txtPositionY.Text);
                theta = double.Parse(txtPositionTheta.Text);
            }
            catch (Exception)
            {
                lblErroCoordenadas.Content = "Erro! Valores das coordenadas inválidos.";
                lblErroCoordenadas.Visibility = Visibility.Visible;
                return;
            }

            
            mapName = cmbMapas.SelectedItem.ToString();
            Thread execution = new Thread(new ThreadStart(this.SetCoordinatePosition));
            execution.Start();
        }




        private void SetCoordinatePosition()
        {

            string result = enviador.Map(mapName+".map");
            if (result.Contains("SUCESSO"))
            {
                result = enviador.Position(x, y, theta);
                DispatchPositionResult(result);
            }
            else
            {
                DispatchPositionResult(result);
            }

        }

        private void DispatchPositionResult(string result)
        {
            StatusDelegate del = new StatusDelegate(this.AtualizaPositionStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }

        private void AtualizaPositionStatus(string result)
        {
            if (result.Contains("SUCESSO"))
            {
                lblSucessoCoordenadas.Content = "Posição atualizada com sucesso!";
                lblSucessoCoordenadas.Visibility = Visibility.Visible;
            }
            else
            {
                if (result.Contains("ERRO ESCOLHA MAPA"))
                {
                    lblErroCoordenadas.Content = "Erro ao atualizar mapa.";
                }

                else if (result.Contains("ERRO"))
                {
                    lblErroCoordenadas.Content = "Erro ao atualizar posição.";
                }
                else
                {
                    lblErroCoordenadas.Content = "Erro ao enviar comando.";
                }
                lblErroCoordenadas.Visibility = Visibility.Visible;
            }
        }

    }
}
