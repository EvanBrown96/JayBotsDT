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
using System.Text.RegularExpressions;

namespace RoverController
{

    public class SocketWrapper
    {

        public Socket sock = null;
        public IPEndPoint ep;

        public async virtual Task<(bool, SocketWrapper)> Send()
        {
            return (true, this);
        }

        public async virtual Task<(bool, SocketWrapper)> Receive()
        {
            return (true, this);
        }

        public async Task<(bool, SocketWrapper)> Connect()
        {
            try
            {
                await sock.ConnectAsync(ep);
                return (true, this);
            }
            catch (SocketException)
            {
                return (false, this);
            }
        }

        public SocketWrapper(IPAddress addr, int port)
        {
            this.ep = new IPEndPoint(addr, port);
        }

    }


    public class MovementSocket : SocketWrapper
    {

        /// <summary>
        /// Queue of messages waiting to be sent to the rover
        /// </summary>
        public static ConcurrentQueue<byte[]> control_msgs = new ConcurrentQueue<byte[]>();

        public override async Task<(bool, SocketWrapper)> Send()
        {
            byte[] cur_msg;
            if (control_msgs.TryDequeue(out cur_msg))
            {
                try
                {
                    int bytes = await this.sock.SendAsync(new ArraySegment<byte>(cur_msg), SocketFlags.None);
                    Console.WriteLine(bytes);
                }
                catch (SocketException)
                {
                    return (false, this);
                }
            }

            return (true, this);
        }

        public MovementSocket(IPAddress addr, int port) : base(addr, port)
        {
        }

    }

    public class RoverContainer
    {

        /// <summary>
        /// The sidebar information panel for this rover
        /// </summary>
        public RoverInfo info_control;
        public Control info_parent;

        public TabControl tab_parent;
        public TabPage rover_tab_page;
        public RoverTab rover_tab;

        /// <summary>
        /// The reference name and IP address of this rover
        /// </summary>
        private string name, ip_addr;

        /// <summary>
        /// Thread object running communication operations with the rover
        /// </summary>
        private Thread conn_thread, conn_thread2;

        /// <summary>
        /// Flag for telling the thread to stop
        /// </summary>
        private volatile bool kill_thread = false;

        /// <summary>
        /// Create a rover container
        /// </summary>
        /// <param name="name">Name of the rover</param>
        /// <param name="ip_addr">IP address of the rover</param>
        /// <param name="info_parent">Parent control to put the rover info into</param>
        public RoverContainer(string name, string ip_addr, Control info_parent, TabControl tab_parent)
        {
            this.name = name;
            this.ip_addr = ip_addr;

            this.info_parent = info_parent;
            info_control = new RoverInfo();
            info_control.Parent = info_parent;
            info_control.Controls.Find("label1", false)[0].Text = name;
            info_control.Controls.Find("label2", false)[0].Text = ip_addr;
            info_control.Margin = new Padding(3);
            info_control.Width = info_parent.Width - 12;

            this.tab_parent = tab_parent;
            rover_tab_page = new TabPage(name);
            rover_tab_page.Parent = tab_parent;
            rover_tab_page.BackColor = Color.White;
            rover_tab = new RoverTab(this);
            rover_tab.Parent = rover_tab_page;
            rover_tab.Width = rover_tab_page.Width - 12;
            rover_tab.Height = rover_tab_page.Height - 12;
            rover_tab.Location = new Point(6, 6);
            rover_tab.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            rover_tab.BackColor = Color.White;
            rover_tab_page.Show();
            rover_tab.Show();

            //socket_code();
            // start connection thread
            conn_thread = new Thread(new ThreadStart(socket_code));
            conn_thread.Start();

            //conn_thread2 = new Thread(new ThreadStart(socket_code2));
            //conn_thread2.Start();*/
        }

        public void destroy()
        {
            info_parent.Controls.Remove(info_control);
            info_control.Dispose();
            tab_parent.Controls.Remove(rover_tab_page);
            rover_tab_page.Dispose();
            stop_socket_clean();
            Master.rovers.Remove(this);
            Master.reorder_info();
        }

        /// <summary>
        /// Tells the connection thread to stop
        /// </summary>
        public void stop_socket_clean()
        {
            kill_thread = true;
        }

        /// <summary>
        /// Adds a command to be sent to the rover
        /// </summary>
        /// <param name="command">The command to send</param>
        public void enqueue_command(string command)
        {
            MovementSocket.control_msgs.Enqueue(Encoding.UTF8.GetBytes(command));
        }

        private async void socket_code()
        {
            PictureBox indicator = info_control.Controls.Find("pictureBox1", false)[0] as PictureBox;

            bool overall_connected = false;

            Queue<SocketWrapper> uninitialized = new Queue<SocketWrapper>();
            Queue<SocketWrapper> connected = new Queue<SocketWrapper>();

            IPAddress addr = IPAddress.Parse(ip_addr);
            SocketWrapper mvmt_cmd = new MovementSocket(addr, 10001);
            uninitialized.Enqueue(mvmt_cmd);

            List<Task<(bool, SocketWrapper)>> awaiting = new List<Task<(bool, SocketWrapper)>>();

            SocketWrapper sw;
            bool success;

            while (!kill_thread)
            {
                while (uninitialized.Count() > 0)
                {
                    sw = uninitialized.Dequeue();
                    sw.sock = new Socket(sw.ep.Address.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
                    awaiting.Add(sw.Connect());
                }
                while (connected.Count() > 0)
                {
                    sw = connected.Dequeue();
                    awaiting.Add(
                        Task.Run(
                            async () =>
                            {
                                Task<(bool, SocketWrapper)> send = sw.Send();
                                Task<(bool, SocketWrapper)> recv = sw.Receive();
                                (bool send_success, _) = await send;
                                (bool recv_success, _) = await recv;
                                return (send_success && recv_success, sw);
                            }
                        )
                    );
                }

                if (awaiting.Count() > 0)
                {
                    int task_index = Task.WaitAny(awaiting.ToArray());
                    Task<(bool, SocketWrapper)> completed = awaiting[task_index] as Task<(bool, SocketWrapper)>;
                    (success, sw) = await completed;
                    if (success)
                    {
                        connected.Enqueue(sw);
                    }
                    else
                    {
                        sw.sock.Dispose();
                        uninitialized.Enqueue(sw);
                    }
                    awaiting.RemoveAt(task_index);
                    if (success != overall_connected)
                    {
                        overall_connected = success;
                        if (success) indicator.Invoke((MethodInvoker) delegate { indicator.Image = Properties.Resources.wifi; });
                        else indicator.Invoke((MethodInvoker) delegate { indicator.Image = Properties.Resources.nowifi; });
                    }
                }
            }

            Task.WaitAll(awaiting.ToArray());
            foreach(Task<(bool, SocketWrapper)> t in awaiting)
            {
                (success, sw) = await t;
                if (success) sw.sock.Disconnect(false);
                else sw.sock.Dispose();
            }
            foreach (SocketWrapper sw_ in uninitialized) sw_.sock.Dispose();
            foreach (SocketWrapper sw_ in connected) sw_.sock.Disconnect(false);

        }

        /// <summary>
        /// Socket code running on the connection thread
        /// </summary>
        /*private void socket_code()
        {
            PictureBox indicator = info_control.Controls.Find("pictureBox1", false)[0] as PictureBox;
            IPAddress addr = IPAddress.Parse(ip_addr);

            SocketState mvmt_cmd = new SocketState(addr, 10001);
            List<ManualResetEvent> waiting_events = new List<ManualResetEvent>();

            while (true)
            {
                for(Socket in sockets)
                {
                    try
                    {
                        if (mvmt_cmd.state == SocketState.Uninitialized)
                        {
                            waiting_events.Add(mvmt_cmd.Connect());
                        }
                        else if (mvmt_cmd.state == SocketState.Connected) 
                        {
                            if (mvmt_cmd.ready_for_send) waiting_events.Add(mvmt_cmd.Send());
                            if (mvmt_cmd.ready_for_receive) waiting_events.Add(mvmt_cmd.Receive());
                        }
                    }


                    if (mvmt_cmd.sock == null)
                    {
                        ManualResetEvent connect_event = mvmt_cmd.Connect();
                        if(connect_event != null) waiting_events.Add(connect_event);
                    }
                    else if (mvmt_cmd.sock.Connected)
                    {
                        if ()
                        {
                            byte[] send_attempt;

                            ManualResetEvent send_event = mvmt_cmd.Send(control_msgs.TryDequeue())
                        }
                        //mvmt_cmd.receiveDone.Reset();
                        //mvmt_cmd.sock.BeginReceive(mvmt_cmd.msg_buf, 0, SocketState.BufferSize, SocketFlags.None, receiveCallback, mvmt_cmd);
                        //waiting_events.Add(mvmt_cmd.receiveDone);

                        if(mvmt_cmd.sendDone.)
                        mvmt_cmd.sendDone.Reset();

                    }
                    else
                    {
                        mvmt_cmd.sock.Dispose();
                        mvmt_cmd.sock = null;
                    }
                    
                    ManualResetEvent.WaitAny(waiting_events.ToArray());


                }
                else
                {
                    client.Shutdown(SocketShutdown.Both);
                    client.Disconnect(true);
                }
                
            }

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
        }*/

        private void socket_code2()
        {
            Label depth_msg = rover_tab.Controls.Find("label1", true)[0] as Label;
            IPAddress addr = IPAddress.Parse(ip_addr);
            IPEndPoint ep = new IPEndPoint(addr, 10002);
            Socket client = null;

            bool connected = false;

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

                    while (true)
                    {
                        // wait until a message is ready, then send
                        
                        byte[] rcv_msg = new byte[32];
                        if (client.Receive(rcv_msg) > 0)
                        {
                            string msg = Encoding.UTF8.GetString(rcv_msg);
                            msg = "Depth:\n" + msg.Replace(",", "\n");
                            depth_msg.Invoke((MethodInvoker)delegate
                            {
                                depth_msg.Text = msg;
                            });
                        }
                    }

                }
                catch // an error occurred somewhere in the connection
                {
                    if (connected)
                    {
                        // update connection indicator on GUI
                        connected = false;
                        
                        // clear messages
                        //control_msgs = new ConcurrentQueue<byte[]>();

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
