using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;
using System.Net;
using System.Net.Sockets;

namespace RoverController
{
    class RoverContainer
    {

        public RoverInfo info_control;
        private string name, ip_addr;
        private Thread conn_thread;
        private volatile bool kill_thread = false;

        public RoverContainer(string name, string ip_addr, Control info_parent)
        {
            this.name = name;
            this.ip_addr = ip_addr;
            this.info_control = new RoverInfo();
            this.info_control.Parent = info_parent;
            this.info_control.Controls.Find("label1", false)[0].Text = name;
            this.info_control.Controls.Find("label2", false)[0].Text = ip_addr;
            this.info_control.Margin = new Padding(3);
            this.info_control.Size = new Size(info_parent.Width - 12, info_control.Size.Height);

            conn_thread = new Thread(new ThreadStart(socket_code));
            conn_thread.Start();
        }

        public void stop_socket_clean()
        {
            kill_thread = true;
        }

        private void socket_code()
        {
            PictureBox indicator = info_control.Controls.Find("pictureBox1", false)[0] as PictureBox;
            IPAddress addr = IPAddress.Parse(ip_addr);
            IPEndPoint ep = new IPEndPoint(addr, 10001);
            Socket client = new Socket(addr.AddressFamily, SocketType.Stream, ProtocolType.Tcp);

            bool connected = false;

            while (true)
            {
                if (kill_thread) return;

                try
                {
                    client.Connect(ep);
                    connected = true;

                    // thread safety - updates GUI on main thread, instead of BG thread
                    indicator.Invoke((MethodInvoker)delegate
                    {
                        indicator.Image = Properties.Resources.wifi;
                    });

                    while (true)
                    {
                        if (kill_thread) return;
                        client.Send(Encoding.UTF8.GetBytes("HELLO"));
                        Thread.Sleep(1000);
                    }

                }
                catch
                {
                    if (connected)
                    {
                        connected = false;
                        indicator.Invoke((MethodInvoker)delegate
                        {
                            indicator.Image = Properties.Resources.nowifi;
                        });
                        Thread.Sleep(1000);
                    }
                }
            }
        }
    }
}
