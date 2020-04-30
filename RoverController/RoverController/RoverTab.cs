using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RoverController
{
    public partial class RoverTab : UserControl
    {
        private RoverContainer container;
        private ManualController mc;

        public RoverTab(RoverContainer container)
        {
            InitializeComponent();
            this.container = container;

            mc = new ManualController(container);
            mc.Parent = groupBox2;
            mc.Size = new Size(groupBox2.Width - 12, groupBox2.Height - 21 - 6);
            mc.Location = new Point(6, 21);
            mc.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            mc.Hide();
        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            if ((sender as RadioButton).Checked)
            {
                mc.Show();
                container.enqueue_command("m-ss");
            }
            else
            {
                mc.Hide();
            }
        }

        private void disconnect_Click(object sender, EventArgs e)
        {
            container.simpleDestroy();
        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            if((sender as RadioButton).Checked)
            {
                speed.Enabled = false;
                container.startMap();
            }
            else
            {
                container.stopMap();
                speed_ValueChanged(sender, e);
            }
        }

        private void reset_Click(object sender, EventArgs e)
        {
            container.resetMap();
        }

        private void view_Click(object sender, EventArgs e)
        {
            container.viewMap();
        }

        private void speed_ValueChanged(object sender, EventArgs e)
        {
            byte speed_val = (byte)speed.Value;
            container.setSpeed(speed_val, container.endChain);
        }
    }
}
