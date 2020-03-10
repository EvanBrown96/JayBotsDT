using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Windows.Forms;

namespace RoverController
{
    public partial class IP_Dialog : Form
    {
        private Form1 original_form;

        public IP_Dialog(Form1 original_form)
        {
            this.original_form = original_form;
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (!validateIP(textBox2)) return;
            this.original_form.do_addConn(textBox1.Text, textBox2.Text);
            this.Close();
        }

        private bool validateIP(TextBox tb)
        {
            IPAddress test_addr;
            if (!IPAddress.TryParse(tb.Text, out test_addr)) return false;
            return (test_addr.ToString() == tb.Text);
        }

        private void button2_Click(object sender, EventArgs e)
        {
            this.Close();
        }

    }
}
