using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
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

        /// <summary>
        /// The sidebar information panel for this rover
        /// </summary>
        public RoverInfo info_control;

        /// <summary>
        /// The reference name and IP address of this rover
        /// </summary>
        private string name, ip_addr;

        /// <summary>
        /// Thread object running communication operations with the rover
        /// </summary>
        private Thread conn_thread;

        /// <summary>
        /// Flag for telling the thread to stop
        /// </summary>
        private volatile bool kill_thread = false;

        /// <summary>
        /// Queue of messages waiting to be sent to the rover
        /// </summary>
        ConcurrentQueue<byte[]> control_msgs = new ConcurrentQueue<byte[]>();

        /// <summary>
        /// Event trigger to tell the rover that a message is ready to be sent
        /// </summary>
        ManualResetEventSlim wait_for_control_msg = new ManualResetEventSlim(false);

        /// <summary>
        /// Create a rover container
        /// </summary>
        /// <param name="name">Name of the rover</param>
        /// <param name="ip_addr">IP address of the rover</param>
        /// <param name="info_parent">Parent control to put the rover info into</param>
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

            // start connection thread
            conn_thread = new Thread(new ThreadStart(socket_code));
            conn_thread.Start();
        }

        /// <summary>
        /// Tells the connection thread to stop
        /// </summary>
        public void stop_socket_clean()
        {
            kill_thread = true;
            wait_for_control_msg.Set();
        }

        /// <summary>
        /// Adds a command to be sent to the rover
        /// </summary>
        /// <param name="command">The command to send</param>
        public void enqueue_command(string command)
        {
            control_msgs.Enqueue(Encoding.UTF8.GetBytes(command));
            wait_for_control_msg.Set();
        }

        /// <summary>
        /// Socket code running on the connection thread
        /// </summary>
        private void socket_code()
        {
            PictureBox indicator = info_control.Controls.Find("pictureBox1", false)[0] as PictureBox;
            IPAddress addr = IPAddress.Parse(ip_addr);
            IPEndPoint ep = new IPEndPoint(addr, 10001);
            Socket client = null;

            bool connected = false;
            byte[] cur_msg;

            while (true)
            {
                // check if the thread has been killed
                if (kill_thread) return;

                try
                {
                    // initiate connection
                    client = new Socket(addr.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
                    client.Connect(ep);
                    connected = true;

                    // thread safety - updates GUI on main thread, instead of BG thread
                    indicator.Invoke((MethodInvoker)delegate
                    {
                        indicator.Image = Properties.Resources.wifi;
                    });

                    while (true)
                    {
                        // wait until a message is ready, then send
                        wait_for_control_msg.Wait();
                        if (kill_thread) return;
                        while (control_msgs.TryDequeue(out cur_msg))
                        {
                            client.Send(cur_msg);
                        }
                    }

                }
                catch // an error occurred somewhere in the connection
                {
                    if (connected)
                    {
                        // update connection indicator on GUI
                        connected = false;
                        indicator.Invoke((MethodInvoker)delegate
                        {
                            indicator.Image = Properties.Resources.nowifi;
                        });
                        // clear messages
                        control_msgs = new ConcurrentQueue<byte[]>();

                        // reset connection
                        client.Shutdown(SocketShutdown.Both);
                        client.Disconnect(true);
                    }
                    Thread.Sleep(1000);
                }
            }
        }
    }
}
