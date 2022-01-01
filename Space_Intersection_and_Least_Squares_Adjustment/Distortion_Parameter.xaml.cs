using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using ImageProcessing;
using LOC.Photogrammetry;
using System.IO;


namespace Space_Intersection
{
    /// <summary>
    /// Distortion_Parameter.xaml 的互動邏輯
    /// </summary>
    public partial class Distortion_Parameter : Window
    {
        public static string ShowImg;
        public Distortion_Parameter()
        {
            InitializeComponent();
        }

        private void Distparameter_Backward_Click(object sender, RoutedEventArgs e)
        {
            double Tarx = 0, Tary = 0;

            InOrientationCV IOPInput = new InOrientationCV();

            LensDistortionCV.SetIOP_CV(Fx.Text, Fy.Text, Cx.Text, Cy.Text, K1.Text, K2.Text, P1.Text, P2.Text, K3.Text, ref IOPInput);

            MainWindow.IOPSet = IOPInput;

            for (int img = 0; img < MainWindow.Image.Count; img++)
            {
                LOCImage distortedImg = new LOCImage(MainWindow.Image[img], Int32Rect.Empty);
                LOCImage undistortedImg = new LOCImage(1920, 1080, 96, 96, PixelFormats.Gray8, null);

                if (!Directory.Exists(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG"))
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG");
                }

                for (int i = 0; i < undistortedImg.Width; i++)
                {
                    for (int j = 0; j < undistortedImg.Height; j++)
                    {
                        LensDistortionCV.AddLensDistortionCV(i, j, ref Tarx, ref Tary, MainWindow.IOPSet);

                        //int pixID = j * 3 * distortedImg.Width + 3 * i;

                        if (Tarx > 0 && Tary > 0 && Tarx < (undistortedImg.Width - 2) && Tary < undistortedImg.Height - 2)
                        {
                            undistortedImg.ByteData[j * undistortedImg.Width + i] = (byte)(((Interpolation.Bicubic(distortedImg, Tarx, Tary, 0)) + (Interpolation.Bicubic(distortedImg, Tarx, Tary, 1)) + (Interpolation.Bicubic(distortedImg, Tarx, Tary, 2))) / 3);
                        }
                    }
                }

                this.Close();

                undistortedImg.Save(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG" + "/" + "undistorted_" + Path.GetFileName(MainWindow.Image[img]), LOC.ImageFormat.Png);
                ShowImg = Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG" + "/" + "undistorted_" + Path.GetFileName(MainWindow.Image[img]);
            }

            
        }

        /*private void Distparameter_Forward_Click(object sender, RoutedEventArgs e)
        {
            float Tarx = 0, Tary = 0;
            string t;

            InOrientationCV IOPInput = new InOrientationCV();
            LensDistortionCV.SetIOP_CV(Fx.Text, Fy.Text, Cx.Text, Cy.Text, K1.Text, K2.Text, P1.Text, P2.Text, K3.Text, ref IOPInput);

            t = Convert.ToString(IOPInput.Cx);
            MainWindow.DebugTXT = t;

            MainWindow.IOPSet = IOPInput;

            for (int img = 0; img < MainWindow.Image.Count; img++)
            {
                LOCImage distortedImg = new LOCImage(MainWindow.Image[img], Int32Rect.Empty);
                LOCImage undistortedImg = new LOCImage(1920, 1080, 96, 96, PixelFormats.Gray8, null);

                if (!Directory.Exists(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG"))
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG");
                }

                for (int i = 0; i < undistortedImg.Width; i++)
                {
                    for (int j = 0; j < undistortedImg.Height; j++)
                    {
                        //ImageDistortion.DistortionRemove(i, j, ref Tarx, ref Tary);
                        //ImageDistortion.DistortionAdd(i, j, ref Tarx, ref Tary);
                        LensDistortionCV.RemoveDistortionCV(i, j, ref Tarx, ref Tary, MainWindow.IOPSet);

                        int pixID = j * 3 * distortedImg.Width + 3 * i;
                        if (Tarx > 0 && Tary > 0 && Tarx < (undistortedImg.Width - 2) && Tary < undistortedImg.Height - 2)
                        {
                            undistortedImg.ByteData[(int)Tary* undistortedImg.Width + (int)Tarx] = (byte)(((distortedImg.ByteData[pixID]) + (distortedImg.ByteData[pixID + 1]) + (distortedImg.ByteData[pixID + 2])) / 3);
                        }
                    }
                }

                this.Close();

                undistortedImg.Save(Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG" + "/" + "undistorted_" + Path.GetFileName(MainWindow.Image[img]), LOC.ImageFormat.Png);
                ShowImg = Path.GetDirectoryName(MainWindow.Savedir) + "/" + "UndistortedIMG" + "/" + "undistorted_" + Path.GetFileName(MainWindow.Image[img]);
            }
        }*/

        private void Exit_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }

        private void Set_IOP_Only_Click(object sender, RoutedEventArgs e)
        {
            InOrientationCV IOPInput = new InOrientationCV();

            LensDistortionCV.SetIOP_CV(Fx.Text, Fy.Text, Cx.Text, Cy.Text, K1.Text, K2.Text, P1.Text, P2.Text, K3.Text, ref IOPInput);

            MainWindow.IOPSet = IOPInput;

            this.Close();

        }


    }
}
