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
        }
    }
}
