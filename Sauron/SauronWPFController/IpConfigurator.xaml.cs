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
using System.Net;

namespace SauronWPFController
{
    /// <summary>
    /// Interaction logic for IpConfigurator.xaml
    /// </summary>
    public partial class IpConfigurator : Window
    {
        private IPManager ipManager;

        public IpConfigurator(IPManager ipManager)
        {
            InitializeComponent();
            this.ipManager = ipManager;
            this.txtIP.Text = ipManager.IP.ToString();
            lblErro.Visibility = Visibility.Hidden;
        }

        private void btnOK_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                ipManager.IP = IPAddress.Parse(txtIP.Text);
                this.Close();
            }
            catch (Exception)
            {
                lblErro.Visibility = Visibility.Visible;
            }

        }

        private void btnCancel_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }





    }
}
