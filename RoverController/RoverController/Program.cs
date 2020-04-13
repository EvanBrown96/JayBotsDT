using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Threading;

namespace RoverController
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Master());
            /*string uri = "ws://127.0.0.1:9090";
            RosSocket r = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri));
            string sub_id = r.Subscribe<std_msgs.String>("/chatter", SubHandler);
            Thread.Sleep(10000);*/
        }

        /*private static void SubHandler(std_msgs.String msg)
        {
            Console.WriteLine(msg.data);
        }*/
    }
}
