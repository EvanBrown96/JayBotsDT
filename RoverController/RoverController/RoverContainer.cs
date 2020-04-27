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
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using rosapi = RosSharp.RosBridgeClient.MessageTypes.Rosapi;
using RosSharp.RosBridgeClient.MessageTypes.RemoteApp;
using RosSharp.RosBridgeClient;
using System.Diagnostics;
using System.IO;


namespace RoverController
{
    public class RoverContainer
    {

        static readonly string uri = string.Format("ws://{0}:9090", Environment.GetEnvironmentVariable("WS_ADDR"));
        private RosSocket ros_socket;
        private String cmd_pub_id, syscommand_id;

        private ReaderWriterLock socket_lock = new ReaderWriterLock();
        private volatile bool killed = false;

        /// <summary>
        /// references to GUI elements
        /// </summary>
        private IWin32Window form_handle;

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
        /// Event trigger to tell the rover that a message is ready to be sent
        /// </summary>
        ManualResetEventSlim wait_for_control_msg = new ManualResetEventSlim(false);

        /// <summary>
        /// Create a rover container
        /// </summary>
        /// <param name="name">Name of the rover</param>
        /// <param name="ip_addr">IP address of the rover</param>
        /// <param name="info_parent">Parent control to put the rover info into</param>
        public RoverContainer(string name, string ip_addr, IWin32Window form_handle, Control info_parent, TabControl tab_parent)
        {
            this.name = name;
            this.ip_addr = ip_addr;

            this.form_handle = form_handle;
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

            ros_socket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri));
            cmd_pub_id = ros_socket.Advertise<std_msgs.String>(string.Format("/{0}/user_cmd", name));
            syscommand_id = ros_socket.Advertise<std_msgs.String>(string.Format("/{0}/syscommand", name));

            ros_socket.CallService<LaunchRoverRequest, LaunchRoverResponse>(
                "/launch_rover", LaunchHandler, new LaunchRoverRequest(name, ip_addr));
            // TODO; add error handling
        }

        private void LaunchHandler(LaunchRoverResponse response)
        {
            if (response.err != "") {
                try
                {
                    preDestroy();

                    // thread safety - updates GUI on main thread, instead of BG thread
                    rover_tab_page.Invoke((MethodInvoker)delegate
                    {
                        MessageBox.Show(form_handle, "An error occurred launching ROS:\n" + response.err,
                            "Error Launching ROS", MessageBoxButtons.OK, MessageBoxIcon.Error, MessageBoxDefaultButton.Button1);

                        Console.WriteLine("poo");

                        destroy();
                    });
                }
                catch (AlreadyDestroyed) { }
            }
            else
            {
                socket_lock.AcquireReaderLock(-1);
                // TODO; add error handling
                if (!killed) ros_socket.CallService<PollRoverRequest, PollRoverResponse>("/poll_rover", PollLoop, new PollRoverRequest(name));
                socket_lock.ReleaseReaderLock();
            }
        }

        private async void PollLoop(PollRoverResponse response)
        {

            if (response.alive)
            {
                if (killed) return;

                await Task.Delay(4000);

                socket_lock.AcquireReaderLock(-1);
                // TODO: add error handling
                if(!killed) ros_socket.CallService<PollRoverRequest, PollRoverResponse>("/poll_rover", PollLoop, new PollRoverRequest(name));
                socket_lock.ReleaseReaderLock();
            }
            else
            {

                try
                {
                    preDestroy();
                    rover_tab_page.Invoke((MethodInvoker)delegate
                    {
                        if (response.err == "")
                        {
                            MessageBox.Show(form_handle, "Nodes exited gracefully, no errors detected.",
                                "Lost Contact with " + name, MessageBoxButtons.OK, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button1);
                        }
                        else
                        {
                            MessageBox.Show(form_handle, "An error occurred: "+response.err,
                                "Lost Contact with " + name, MessageBoxButtons.OK, MessageBoxIcon.Error, MessageBoxDefaultButton.Button1);
                        }

                        destroy();
                    });
                }
                catch (AlreadyDestroyed) { }
            }
            
        }

        public void resetMap()
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.Publish(syscommand_id, new std_msgs.String { data = "reset" });
            socket_lock.ReleaseReaderLock();
        }

        public void viewMap()
        {
            string base_config = Path.GetDirectoryName(Path.GetDirectoryName(Path.GetDirectoryName(Path.GetDirectoryName(AppDomain.CurrentDomain.BaseDirectory)))) + "\\base.rviz";
            string base_contents = File.ReadAllText(base_config);
            string temp_contents = base_contents.Replace("<machine_name>", name);

            string temp_config = string.Format("{0}\\AppData\\Local\\Temp\\{1}.rviz", Environment.GetEnvironmentVariable("USERPROFILE"), name);
            File.WriteAllText(temp_config, temp_contents);

            Process proc = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "cmd.exe",
                    Arguments = string.Format("/C \"c:\\opt\\ros\\melodic\\x64\\setup.bat && rviz -d {0}\"", temp_config),
                    UseShellExecute = false,
                    RedirectStandardOutput = false,
                    CreateNoWindow = true
                }
            };
            proc.Start();
        }

        public void startMap()
        {
            //startMotor((std_msgs.EmptyResponse _) =>
            //{
                setMapParamsOn((rosapi.SetParamResponse _) =>
                {
                    refreshMapParams();
                    startBlinker((std_msgs.EmptyResponse __) => { });
                });
            //});
        }

        public void stopMap()
        {
            setMapParamsOff((rosapi.SetParamResponse _) =>
            {
                refreshMapParams();
                stopBlinker((std_msgs.EmptyResponse __) => { });
            });
        }

        private void startMotor(RosSharp.RosBridgeClient.ServiceResponseHandler<std_msgs.EmptyResponse> chain) 
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService(string.Format("/{0}/start_motor", name), chain, new std_msgs.EmptyRequest());
            socket_lock.ReleaseReaderLock();
        }

        private void stopMotor(RosSharp.RosBridgeClient.ServiceResponseHandler<std_msgs.EmptyResponse> chain)
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService(string.Format("/{0}/stop_motor", name), chain, new std_msgs.EmptyRequest());
            socket_lock.ReleaseReaderLock();
        }

        private void startBlinker(RosSharp.RosBridgeClient.ServiceResponseHandler<std_msgs.EmptyResponse> chain)
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService(string.Format("/{0}/start_blinker", name), chain, new std_msgs.EmptyRequest());
            socket_lock.ReleaseReaderLock();
        }

        private void stopBlinker(RosSharp.RosBridgeClient.ServiceResponseHandler<std_msgs.EmptyResponse> chain)
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService(string.Format("/{0}/stop_blinker", name), chain, new std_msgs.EmptyRequest());
            socket_lock.ReleaseReaderLock();
        }

        private void setMapParamsOn(ServiceResponseHandler<rosapi.SetParamResponse> chain)
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService("/rosapi/set_param",
                (rosapi.SetParamResponse _) =>
                {
                    socket_lock.AcquireReaderLock(-1);
                    if (!killed) ros_socket.CallService("/rosapi/set_param",
                        chain,
                        new rosapi.SetParamRequest(string.Format("/{0}/hector_mapping/update_factor_occupied", name), "0.9"));
                    socket_lock.ReleaseReaderLock();
                },
                new rosapi.SetParamRequest(string.Format("/{0}/hector_mapping/update_factor_free", name), "0.4")
                );
            socket_lock.ReleaseReaderLock();
        }

        private void setMapParamsOff(ServiceResponseHandler<rosapi.SetParamResponse> chain)
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.CallService("/rosapi/set_param",
                (rosapi.SetParamRequest __) =>
                {
                    socket_lock.AcquireReaderLock(-1);
                    if (!killed) ros_socket.CallService("/rosapi/set_param",
                        chain,
                        new rosapi.SetParamRequest(string.Format("/{0}/hector_mapping/update_factor_occupied", name), "0.5"));
                    socket_lock.ReleaseReaderLock();
                },
                new rosapi.SetParamRequest(string.Format("/{0}/hector_mapping/update_factor_free", name), "0.5")
                );
            socket_lock.ReleaseReaderLock();
        }

        private void refreshMapParams()
        {
            socket_lock.AcquireReaderLock(-1);
            if (!killed) ros_socket.Publish(syscommand_id, new std_msgs.String { data = "refresh_update_factors" });
            socket_lock.ReleaseReaderLock();
        }

        public void simpleDestroy()
        {
            try
            {
                preDestroy();
                destroy();
            }
            catch (AlreadyDestroyed) { }
        }

        private class AlreadyDestroyed : Exception { }

        private void preDestroy()
        {
            socket_lock.AcquireWriterLock(-1);
            if (killed)
            {
                socket_lock.ReleaseWriterLock();
                throw new AlreadyDestroyed();
            }
            killed = true;
            socket_lock.ReleaseWriterLock();
        }

        private void destroy()
        {
            // preDestroy MUST be called before this 
            info_parent.Controls.Remove(info_control);
            info_control.Dispose();
            tab_parent.Controls.Remove(rover_tab_page);
            rover_tab_page.Dispose();

            ros_socket.CallService<KillRoverRequest, KillRoverResponse>("/kill_rover", (KillRoverResponse response) => { }, new KillRoverRequest(name));
            ros_socket.Unadvertise(cmd_pub_id);
            ros_socket.Unadvertise(syscommand_id);
            ros_socket.Close();

            Master.rovers.Remove(this);
            Master.reorder_info();

        }

        /// <summary>
        /// Adds a command to be sent to the rover
        /// </summary>
        /// <param name="command">The command to send</param>
        public void enqueue_command(string command)
        {
            std_msgs.String msg = new std_msgs.String { data = command };
            ros_socket.Publish(cmd_pub_id, msg);
            // TODO: add error handling if publish call fails
        }

    }
}
