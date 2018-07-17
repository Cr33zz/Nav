namespace NavMeshViewer
{
    partial class NavMeshViewer
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(NavMeshViewer));
            this.refresh_timer = new System.Windows.Forms.Timer(this.components);
            this.update_timer = new System.Windows.Forms.Timer(this.components);
            this.SuspendLayout();
            // 
            // refresh_timer
            // 
            this.refresh_timer.Enabled = true;
            this.refresh_timer.Interval = 50;
            this.refresh_timer.Tick += new System.EventHandler(this.refresh_timer_Tick);
            // 
            // NavMeshViewer
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(848, 571);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "NavMeshViewer";
            this.Text = "NavMeshViewer";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.OnClosing);
            this.Load += new System.EventHandler(this.OnLoad);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.NavMeshViewer_KeyPress);
            this.MouseMove += new System.Windows.Forms.MouseEventHandler(this.NavMeshViewer_MouseMove);
            this.MouseUp += new System.Windows.Forms.MouseEventHandler(this.NavMeshViewer_MouseUp);
            this.MouseWheel += new System.Windows.Forms.MouseEventHandler(this.NavMeshViewer_MouseWheel);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Timer refresh_timer;
        private System.Windows.Forms.Timer update_timer;
    }
}

