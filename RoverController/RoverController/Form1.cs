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

        private List<RoverContainer> rovers = new List<RoverContainer>();

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

        public void do_addConn(string name, string ip_addr)
        {
            RoverContainer container = new RoverContainer(name, ip_addr, groupBox1);
            container.info_control.Location = new Point(6, 22 + 53 * rovers.Count);
            rovers.Add(container);
            container.info_control.Show();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            Console.WriteLine("CLOSING");
            foreach (RoverContainer container in rovers)
            {
                container.stop_socket_clean();
            }
        }
    }
}

