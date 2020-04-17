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
    public partial class Master : Form
    {

        /// <summary>
        /// A list of rover abstractions which can be used to interact with each rover
        /// </summary>
        public static List<RoverContainer> rovers = new List<RoverContainer>();

        public Master()
        {
            InitializeComponent();
        }

        public static void reorder_info()
        {
            for(int i = 0; i < rovers.Count(); i++)
            {
                rovers[i].info_control.Location = get_info_location(i);
            }
        }

        private static Point get_info_location(int index)
        {
            return new Point(6, 22 + 53 * index);
        }
        
        /// <summary>
        /// Prompt user for connection details upon clicking "Add Connection" button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void addConn_Click(object sender, EventArgs e)
        {
            IP_Dialog dialog = new IP_Dialog(this);
            dialog.Show();
            dialog.Focus();
        }

        /// <summary>
        /// Create a new rover object in the GUI and initialize the connection
        /// </summary>
        /// <param name="name">Reference name of the rover</param>
        /// <param name="ip_addr">IP address of the rover</param>
        public void do_addConn(string name, string ip_addr)
        {
            RoverContainer container = new RoverContainer(name, ip_addr, Control.FromHandle(this.Handle), groupBox1, tabControl1);
            container.info_control.Location = get_info_location(rovers.Count);
            rovers.Add(container);
            container.info_control.Show();
            container.rover_tab_page.Show();
            container.rover_tab.Show();
        }

        /// <summary>
        /// Cleans up rover connection threads upon application closure
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            while(rovers.Count > 0)
            {
                rovers[0].simpleDestroy();
            }
        }
       
    }
}

