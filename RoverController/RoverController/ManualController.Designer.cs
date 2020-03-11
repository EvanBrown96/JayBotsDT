namespace RoverController
{
    partial class ManualController
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.SuspendLayout();
            // 
            // ManualController
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Name = "ManualController";
            this.Size = new System.Drawing.Size(220, 220);
            this.Click += new System.EventHandler(this.ManualController_Click);
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.ManualController_Paint);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.ManualController_KeyDown);
            this.KeyUp += new System.Windows.Forms.KeyEventHandler(this.ManualController_KeyUp);
            this.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.ManualController_PreviewKeyDown);
            this.ResumeLayout(false);

        }

        #endregion
    }
}
