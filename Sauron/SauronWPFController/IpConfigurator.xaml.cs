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
        private SauronController controller;

        public IpConfigurator(SauronController controller)
        {
            InitializeComponent();
            this.controller = controller;
            if (controller.IP != null)
            {
                this.txtIP.Text = controller.IP.ToString();
            }
            lblErro.Visibility = Visibility.Hidden;
        }

        private void btnOK_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                controller.IP = IPAddress.Parse(txtIP.Text);
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
