﻿using System;
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
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Timers;
using System.Windows.Threading;
using System.Net;
using System.Threading;

namespace SauronWPFController
{


    public delegate void StatusDelegate(string result);

    /// <summary>
    /// Interaction logic for Window1.xaml
    /// </summary>
    public partial class SauronController : Window
    {
        public class MyTimer : System.Timers.Timer
        {
            public MyTimer(int index, double across) { this.Index = index; this.Across = across; }
            public int Index { get; set; }
            public double Across { get; set; }
        }
      
        private Image[] images;
        private string[] names;
        private System.Timers.Timer timePassing;
        private MyTimer moveMouseTimer;
        private int moveCount = 0;
        private IPManager ipManager = new IPManager();
        private EnviadorComandos enviador;
        private RecebedorStatus recebedor;
        private delegate void SimpleDelegate();


        private NavigationWindow navigationMonitor;

        private int MAXCOUNT = 10;
        private int range = 2;
        private int max = 125;
        private int min = 75;

        public SauronController()
        {
            InitializeComponent();
            images = new Image[] { null, image1, image2, image3, image4, image5 };
            names = new string[] { null, "Mark", "Stop", "Navigation", "Freeze", "Position" };
            this.enviador = new EnviadorComandos(ipManager);
            this.recebedor = new RecebedorStatus(ipManager);
            this.navigationMonitor = new NavigationWindow(enviador, recebedor);
            this.ipManager.AddListener(ip => this.txtIpSauron.Content = ip.ToString());

            this.Closing += new System.ComponentModel.CancelEventHandler(SauronController_Closing);

            this.image1.MouseMove += new MouseEventHandler(image1_MouseMove);
            this.image2.MouseMove += new MouseEventHandler(image2_MouseMove);
            this.image3.MouseMove += new MouseEventHandler(image3_MouseMove);
            this.image4.MouseMove += new MouseEventHandler(image4_MouseMove);
            this.image5.MouseMove += new MouseEventHandler(image5_MouseMove);
                     
            timePassing = new System.Timers.Timer();
            timePassing.Elapsed += new ElapsedEventHandler(LeaveEvent);
            timePassing.Interval = 10;
            timePassing.Start();
        }

        void SauronController_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.navigationMonitor.ForceClose();
        }

     

        void navigationClick(object sender, MouseButtonEventArgs e)
        {
            navigationMonitor.Show();
            navigationMonitor.Focus();
        }

        void markClick(object sender, MouseButtonEventArgs e)
        {

        }

        void positionClick(object sender, MouseButtonEventArgs e)
        {
        }

        void freezeClick(object sender, MouseButtonEventArgs e)
        {
            Thread execution = new Thread(new ThreadStart(this.Freeze));
            execution.Start();
        }

        private void Freeze()
        {
            string result = enviador.Freeze();
            StatusDelegate del = new StatusDelegate(navigationMonitor.AtualizaStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }



        void stopClick(object sender, MouseButtonEventArgs e)
        {
            string result = enviador.Halt();
            StatusDelegate del = new StatusDelegate(navigationMonitor.AtualizaStatus);
            this.Dispatcher.BeginInvoke(del, result);
        }

        void configureClick(object sender, MouseButtonEventArgs e)
        {
            IpConfigurator ipConfigurator = new IpConfigurator(ipManager);
            ipConfigurator.Show();

        }



       
       

       private void LeaveEvent(object source, ElapsedEventArgs e )
       {

            SimpleDelegate del = delegate()
            {
                bool isOver = false;
                for (int i = 1; i <= 5; i++)
                {
                    if (images[i].IsMouseOver)
                    {
                        isOver = true;
                        break;
                    }
                }

                if (!isOver)
                {
                    lblTitle.Visibility = Visibility.Hidden;
                    double step = (max - min) / MAXCOUNT;

                    for (int i = 1; i <= 5; i++)
                    {
                        double newWidth = min;

                        newWidth = images[i].Width - step;

                        if (newWidth < min)
                        {
                            newWidth = min;
                        }
                        Resize(images[i], newWidth);
                    }
                }
            };

            this.Dispatcher.BeginInvoke(DispatcherPriority.Send, del);

       }

       void image_MouseMove(int index, MouseEventArgs e)
       {
           if(moveMouseTimer != null)
           {
               moveMouseTimer.Stop();
           }
           double across = e.GetPosition(images[index]).X / images[index].Width;
           moveCount = 0;
           moveMouseTimer = new MyTimer(index, across);
           moveMouseTimer.Elapsed += new ElapsedEventHandler(MouseEvent);
           moveMouseTimer.Interval = 1;
           moveMouseTimer.Start();
       }

       private void MouseEvent(object source, ElapsedEventArgs e)
       {
           int index = ((MyTimer)source).Index;
           moveCount++;
           SimpleDelegate del = delegate()
           {
               lblTitle.Content = names[index];
               lblTitle.Visibility = Visibility.Visible;
               MoveMouse(index, (MyTimer)source);
           };
           this.Dispatcher.BeginInvoke(DispatcherPriority.Send, del);
           if (moveCount > MAXCOUNT)
           {
               ((MyTimer)source).Stop();
           }
       }


       bool DeveContinuar(out double newWidth, double finalWidth, double actual, double step)
       {
           if (actual > finalWidth)
           {
               // se atual eh maior, deve diminuir
               newWidth = actual - step;
               if (newWidth <= finalWidth)
               {
                   newWidth = finalWidth;
                   return false;
               }
           }
           else
           {
               // se atual eh menor, deve aumentar
               newWidth = actual + step;
               if (newWidth >= finalWidth)
               {
                   newWidth = finalWidth;
                   return false;
               }
           }
           return true;
       }

       void MoveMouse(int index, MyTimer mytimer)
        {
            
            double step = (max - min) / MAXCOUNT;
            double across = mytimer.Across;
            bool deveContinuar = false;

           for (int i = 1; i <= 5; i++)
            {
                // check whether the icon is in the range to be resized
                if (i < index - range || i > index + range)
                {
                    double newWidth;
                    deveContinuar = DeveContinuar(out newWidth, min, images[i].Width, step);
                    Resize(images[i], newWidth);
                }
                else if (i == index)
                {
                    double newWidth;
                    deveContinuar = DeveContinuar(out newWidth, max, images[i].Width, step);
                    Resize(images[i], newWidth);
                }
                else if (i < index)
                {
                    double finalWidth = min + Math.Round((max - min - 1) * (Math.Cos((i - index - across + 1) / range * Math.PI) + 1) / 2);
                    double newWidth;
                    deveContinuar = DeveContinuar(out newWidth, finalWidth, images[i].Width, step);
                    Resize(images[i], newWidth);
                }
                else
                {
                    double finalWidth = min + Math.Round((max - min - 1) * (Math.Cos((i - index - across) / range * Math.PI) + 1) / 2);
                    double newWidth;
                    deveContinuar = DeveContinuar(out newWidth, finalWidth, images[i].Width, step);
                    Resize(images[i], newWidth);
                }
            }

           if (!deveContinuar)
           {
               mytimer.Stop();
           }

        }

        void image1_MouseMove(object sender, MouseEventArgs e)
        {
            image_MouseMove(1, e);
        }

        void image2_MouseMove(object sender, MouseEventArgs e)
        {
            image_MouseMove(2, e);
        }

        void image3_MouseMove(object sender, MouseEventArgs e)
        {
            image_MouseMove(3, e);
        }

        void image4_MouseMove(object sender, MouseEventArgs e)
        {
            image_MouseMove(4, e);
        }

        void image5_MouseMove(object sender, MouseEventArgs e)
        {
            image_MouseMove(5, e);
        }

       


        void Resize(Image image, double newWidth)
        {
                double alfa = image.Width;
                image.Width = newWidth;
                alfa = image.Width / alfa;
                image.Height = alfa * image.Height;
        }
       
    }
}
