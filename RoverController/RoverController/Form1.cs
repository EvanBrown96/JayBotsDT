using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Threading;
using System.Windows.Forms;

namespace RoverController
{
    public partial class Form1 : Form
    {

        private List<RoverInfo> infos = new List<RoverInfo>();

        public Form1()
        {
            InitializeComponent();
        }

        private void addConn_Click(object sender, EventArgs e)
        {
            IP_Dialog dialog = new IP_Dialog(this);
            dialog.Show();
            dialog.Focus();
        }

        class bgWorkerArgs
        {
            public string ip_addr;
            public RoverInfo info;

            public bgWorkerArgs(string ip_addr, RoverInfo info)
            {
                this.ip_addr = ip_addr;
                this.info = info;
            }
        };

        public void do_addConn(string name, string ip_addr)
        {
            RoverInfo info = new RoverInfo();
            info.Parent = groupBox1;
            info.Controls.Find("label1", false)[0].Text = name;
            info.Controls.Find("label2", false)[0].Text = ip_addr;
            info.Location = new Point(7, 22+53*infos.Count);
            infos.Add(info);
            info.Show();

            BackgroundWorker bg = new BackgroundWorker();
            bg.DoWork += Bg_DoWork;

            bg.RunWorkerAsync(new bgWorkerArgs(ip_addr, info));

        }

        private void Bg_DoWork(object sender, DoWorkEventArgs e)
        { 
            bgWorkerArgs args = e.Argument as bgWorkerArgs;
            string ip_addr = args.ip_addr;
            RoverInfo info = args.info;
            PictureBox indicator = info.Controls.Find("pictureBox1", false)[0] as PictureBox;
            IPAddress addr = IPAddress.Parse(ip_addr);
            IPEndPoint ep = new IPEndPoint(addr, 10001);
            Socket client = new Socket(addr.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

            bool connected = false;

            while (true)
            {
                try
                {
                    client.Connect(ep);
                    connected = true;
                    indicator.Image = Properties.Resources.wifi;

                    while (true)
                    {
                        client.Send(Encoding.UTF8.GetBytes("HELLO"));
                        Thread.Sleep(1000);
                    }

                }
                catch
                {
                    if (connected)
                    {
                        connected = false;
                        indicator.Image = Properties.Resources.nowifi;
                        Thread.Sleep(1000);
                    }
                }
            }

        }
    }
}
