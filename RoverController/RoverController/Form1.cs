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

        /// <summary>
        /// A list of rover abstractions which can be used to interact with each rover
        /// </summary>
        private List<RoverContainer> rovers = new List<RoverContainer>();

        public Form1()
        {
            InitializeComponent();
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
            RoverContainer container = new RoverContainer(name, ip_addr, groupBox1);
            container.info_control.Location = new Point(6, 22 + 53 * rovers.Count);
            rovers.Add(container);
            container.info_control.Show();
        }

        /// <summary>
        /// Cleans up rover connection threads upon application closure
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            foreach (RoverContainer container in rovers)
            {
                container.stop_socket_clean();
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            rovers[0].enqueue_command("Hi There");
        }
    }
}

