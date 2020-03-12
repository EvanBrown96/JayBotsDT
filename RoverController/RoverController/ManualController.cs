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

    public partial class ManualController : UserControl
    {
        private Dictionary<Keys, bool> key_states = new Dictionary<Keys, bool>();
        private Keys[] control_keys = { Keys.Up, Keys.Down, Keys.Left, Keys.Right, Keys.Space };
        private RoverContainer container;

        public void ManualController_KeyDown(object sender, KeyEventArgs e)
        {
            if (key_states.ContainsKey(e.KeyCode))
            {
                if (!key_states[e.KeyCode])
                {
                    key_states[e.KeyCode] = true;
                    Refresh();
                    send_message();
                }
            }
        }

        public void ManualController_KeyUp(object sender, KeyEventArgs e)
        {
            if (key_states.ContainsKey(e.KeyCode))
            {
                key_states[e.KeyCode] = false;
                Refresh();
                send_message();
            }
        }

        public ManualController(RoverContainer container)
        {
            InitializeComponent();
            // setting these flags prevents control from flickering on redraw
            this.SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint, true);
            this.container = container;

            foreach (Keys k in control_keys)
            {
                key_states[k] = false;
            }
        }

        private void ManualController_Click(object sender, EventArgs e)
        {
            //this.Select();
            this.Focus();
        }

        private void ManualController_PreviewKeyDown(object sender, PreviewKeyDownEventArgs e)
        {
            if (key_states.ContainsKey(e.KeyCode)) e.IsInputKey = true;
        }

        private void ManualController_Paint(object sender, PaintEventArgs e)
        {
            int size, width_start = 0, height_start = 0;
            if(this.Width > this.Height)
            {
                size = this.Height;
                width_start = (this.Width - this.Height) / 2;
            }
            else
            {
                size = this.Width;
                height_start = (this.Height - this.Width) / 2;
            }
            
            int half_size = size / 2;
            int border_size = size / 20;
            int dot_dist = (3 * size) / 8;
            int diag_dist = (int) ((3 * size) / (8 * Math.Sqrt(2)));
            int dot_radius = 10;
            int dot_diameter = dot_radius * 2;
            int mid_radius = 5;

            SolidBrush light_slate_gray = new SolidBrush(Color.LightSlateGray);
            SolidBrush black = new SolidBrush(Color.Black);
            SolidBrush red = new SolidBrush(Color.LightSalmon);
            e.Graphics.FillEllipse(black, new Rectangle(width_start, height_start, size, size));
            e.Graphics.FillEllipse(light_slate_gray, new Rectangle(width_start + border_size, height_start + border_size, size - border_size * 2, size - border_size * 2));
            e.Graphics.FillEllipse(red, new Rectangle(width_start + half_size - mid_radius, height_start + half_size - mid_radius, mid_radius * 2, mid_radius * 2));

            Rectangle pos_rect = new Rectangle(width_start + half_size - dot_radius, height_start + half_size - dot_radius, dot_diameter, dot_diameter);
            if (!key_states[Keys.Space])
            {
                bool fwd = false, bck = false, left = false, right = false;
                if (key_states[Keys.Up] && !key_states[Keys.Down]) fwd = true;
                else if (!key_states[Keys.Up] && key_states[Keys.Down]) bck = true;
                if (key_states[Keys.Left] && !key_states[Keys.Right]) left = true;
                else if (!key_states[Keys.Left] && key_states[Keys.Right]) right = true;

                if (fwd)
                {
                    if (left) pos_rect = new Rectangle(width_start + half_size - diag_dist - dot_radius, height_start + half_size - diag_dist - dot_radius, dot_diameter, dot_diameter);
                    else if (right) pos_rect = new Rectangle(width_start + half_size + diag_dist - dot_radius, height_start + half_size - diag_dist - dot_radius, dot_diameter, dot_diameter);
                    else pos_rect = new Rectangle(width_start + half_size - dot_radius, height_start + half_size - dot_dist - dot_radius, dot_diameter, dot_diameter);
                }
                else if (bck)
                {
                    if (left) pos_rect = new Rectangle(width_start + half_size - diag_dist - dot_radius, height_start + half_size + diag_dist - dot_radius, dot_diameter, dot_diameter);
                    else if (right) pos_rect = new Rectangle(width_start + half_size + diag_dist - dot_radius, height_start + half_size + diag_dist - dot_radius, dot_diameter, dot_diameter);
                    else pos_rect = new Rectangle(width_start + half_size - dot_radius, height_start + half_size + dot_dist - dot_radius, dot_diameter, dot_diameter);
                }
                else
                {
                    if (left) pos_rect = new Rectangle(width_start + half_size - dot_dist - dot_radius, height_start + half_size - dot_radius, dot_diameter, dot_diameter);
                    else if (right) pos_rect = new Rectangle(width_start + half_size + dot_dist - dot_radius, height_start + half_size - dot_radius, dot_diameter, dot_diameter);
                }
            }

            e.Graphics.FillEllipse(black, pos_rect);
        }

        private void send_message()
        {
            StringBuilder message = new StringBuilder("m-ss");
            if (!key_states[Keys.Space])
            {
                if (key_states[Keys.Up] && !key_states[Keys.Down]) message[2] = 'f';
                else if (!key_states[Keys.Up] && key_states[Keys.Down]) message[2] = 'b';
                if (key_states[Keys.Left] && !key_states[Keys.Right]) message[3] = 'l';
                else if (!key_states[Keys.Left] && key_states[Keys.Right]) message[3] = 'r';
            }
            container.enqueue_command(message.ToString());
        }
    }
}
