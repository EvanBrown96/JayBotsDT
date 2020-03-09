using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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

        public void do_addConn(string name, string ip_addr)
        {
            RoverInfo info = new RoverInfo();
            info.Parent = groupBox1;
            info.Controls.Find("label1", false)[0].Text = name;
            info.Controls.Find("label2", false)[0].Text = ip_addr;
            info.Location = new Point(7, 22+53*infos.Count);
            infos.Add(info);
            info.Show();
        }
    }
}
