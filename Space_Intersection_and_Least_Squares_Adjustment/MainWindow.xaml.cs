using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Drawing.Imaging;
using System.IO;
using Microsoft.Win32;
using LOC.Photogrammetry;
using SLAM;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;



namespace Space_Intersection
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public static string Savedir;
        public static string TXTdir;
        public static string DebugTXT;
        public static string OutlierList;

        public static InOrientationCV IOPSet = new InOrientationCV();
        public static List<string> Image = new List<string>();
        public static List<KeyFrame> KFList = new List<KeyFrame>();
        public static List<ControlPoint> KFComapre = new List<ControlPoint>();
        public static List<ControlPoint> CPList = new List<ControlPoint>();
        public static List<double[]> IntersectControlPointind = new List<double[]>();

        public static double Scale7para;
        public static Matrix<double> Rot7para;
        public static Matrix<double> Trans7para;

        public static int Weighting_Type = 0;
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Load_Image_Click(object sender, RoutedEventArgs e)
        {
            //Open Image File
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            Image_name.Text = OFD.FileName;
            Savedir = Path.GetDirectoryName(Image_name.Text);
            Image.Add(Image_name.Text);
            Debug.Text = Image[0];
        }

        private void LoadAll_Click(object sender, RoutedEventArgs e)
        {
            //Put all the file in the folder to a list
            Dirname.Text = "Images in " + Path.GetDirectoryName(Image_name.Text) + " have been loaded";
            Savedir = Path.GetDirectoryName(Image_name.Text);

            foreach (string filename in Directory.GetFileSystemEntries(Path.GetDirectoryName(Image_name.Text), "*.png"))
            {
                if (filename != Image_name.Text) Image.Add(filename);
            }
        }

        private void Undistorted_Img_Click(object sender, RoutedEventArgs e)
        {
            var main = new Distortion_Parameter();
            //this.Close();
            main.Show();
        }

        private void button_Click(object sender, RoutedEventArgs e)
        {
            using (var stream = new FileStream(Distortion_Parameter.ShowImg, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                Debug_Image.Source = BitmapFrame.Create(stream, BitmapCreateOptions.None, BitmapCacheOption.OnLoad);
                Debug.Text = DebugTXT;
            }
        }

        private void Load_KF_Info_Click(object sender, RoutedEventArgs e)
        {
            //The output in KF.txt belongs to Tcw.
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line;
            int KFID, FrameID;
            double X, Y, Z, R11, R12, R13, R21, R22, R23, R31, R32, R33;

            TXTdir = OFD.FileName;
            File_XYZ_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            TXTfile.ReadLine(); //header

            //atrix<double> t = DenseMatrix.CreateDiagonal(3, 1, 0);
            //Matrix<double> tcw = DenseMatrix.CreateDiagonal(3, 1, 0);

            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] KFInfo = line.Split(' ');

                KFID = Convert.ToInt32(KFInfo[0]);
                X = double.Parse(KFInfo[2]);
                Y = double.Parse(KFInfo[3]);
                Z = double.Parse(KFInfo[4]);
                R11 = double.Parse(KFInfo[5]);
                R12 = double.Parse(KFInfo[6]);
                R13 = double.Parse(KFInfo[7]);
                R21 = double.Parse(KFInfo[8]);
                R22 = double.Parse(KFInfo[9]);
                R23 = double.Parse(KFInfo[10]);
                R31 = double.Parse(KFInfo[11]);
                R32 = double.Parse(KFInfo[12]);
                R33 = double.Parse(KFInfo[13]);
                FrameID = Convert.ToInt32(KFInfo[15]);

                //tcw[0, 0] = X; tcw[1, 0] = Y; tcw[2, 0] = Z;
                //t = tcw + t;
                //X = t[0, 0]; Y = t[1, 0]; Z = t[2, 0];                

                RotationMatrix Rcw = new RotationMatrix(R11, R12, R13, R21, R22, R23, R31, R32, R33);
                ExOrietation EO = new ExOrietation(X, Y, Z, Rcw);

                KeyFrame KF = new KeyFrame(KFID, EO, FrameID);
                KFList.Add(KF);
            }
        }

        private void Load_Control_Point_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line, cpid;
            int fid;
            double cpx97, cpy97, cpz97, imgx, imgy;

            TXTdir = OFD.FileName;
            File_CP_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            TXTfile.ReadLine(); //header

            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] CPInfo = line.Split(' ');

                cpid = CPInfo[0];
                cpx97 = Convert.ToDouble(CPInfo[1]);
                cpy97 = Convert.ToDouble(CPInfo[2]);
                cpz97 = Convert.ToDouble(CPInfo[3]);

                List<KeyPoint> KPList = new List<KeyPoint>();

                for (int i = 4; i < CPInfo.Length; i += 3)
                {
                    fid = Convert.ToInt32(CPInfo[i]);
                    imgx = Convert.ToDouble(CPInfo[i + 1]);
                    imgy = Convert.ToDouble(CPInfo[i + 2]);

                    KeyPoint KP = new KeyPoint(fid, imgx, imgy);
                    KPList.Add(KP);
                }

                ControlPoint CP = new ControlPoint(cpid, cpx97, cpy97, cpz97, KPList);
                CPList.Add(CP);
            }
        }

        private void Map_Transformation_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line, mpid;
            double mpx, mpy, mpz;
            List<ControlPoint> MPList = new List<ControlPoint>();

            TXTdir = OFD.FileName;
            File_Map_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            Matrix<double> XYZ = DenseMatrix.Create(3, 1, 0);
            Matrix<double> ENH;
            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] MPInfo = line.Split(' ');

                mpid = MPInfo[0];
                mpx = Convert.ToDouble(MPInfo[1]); XYZ[0, 0] = mpx;
                mpy = Convert.ToDouble(MPInfo[2]); XYZ[1, 0] = mpy;
                mpz = Convert.ToDouble(MPInfo[3]); XYZ[2, 0] = mpz;

                ENH = Scale7para * Rot7para.Multiply(XYZ) + Trans7para.Transpose();

                ControlPoint MP = new ControlPoint(mpid, ENH[0, 0], ENH[1, 0], ENH[2, 0]);
                MPList.Add(MP);
            }

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "MP_XYZ_Transformed.txt", FileMode.Create, FileAccess.Write))
            {
                double ToDeg = Convert.ToInt32(180f / Math.PI);

                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    foreach (ControlPoint mp in MPList) sb.AppendLine(mp.CPID + " " + ToStr(mp.CPX97) + " " + ToStr(mp.CPY97) + " " + ToStr(mp.CPZ97));
                    
                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }

        private void Foreard_Intersection_Click(object sender, RoutedEventArgs e)
        {
            //Forward Intersection with OpenCV

            List<ControlPoint> CPList_ind = new List<ControlPoint>();

            foreach (ControlPoint cp in CPList)
            {
                Collinearity CL = new Collinearity();
                ExOrietation[] EO = new ExOrietation[cp.KeyPointList.Count];
                ImagePoints.Coordinate[] IMGXY = new ImagePoints.Coordinate[cp.KeyPointList.Count];

                int i = 0;

                foreach (KeyFrame kf in KFList)
                {
                    foreach (KeyPoint kp in cp.KeyPointList)
                    {
                        if (kf.FrameID == kp.FID)
                        {
                            EO[i] = kf.EO;
                            IMGXY[i] = kp.IMGXY;

                            i += 1;
                        }
                    }
                }

                double[] IntersectXYZ = CL.FowardIntersectionCV(EO, IOPSet, IMGXY);
                ControlPoint CP_ind = new ControlPoint(cp.CPID, IntersectXYZ[0], IntersectXYZ[1], IntersectXYZ[2]);
                CPList_ind.Add(CP_ind);
            }

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Control_Point_Ind.csv", FileMode.Create, FileAccess.Write))
            {
                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    foreach (ControlPoint cp in CPList_ind)
                    {
                        sb.AppendLine(cp.CPID + " " + ToStr(cp.CPX97) + " " + ToStr(cp.CPY97) + " " + ToStr(cp.CPZ97));
                    }
                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }

        }

        private void Load_TUM_Traj_Click(object sender, RoutedEventArgs e)
        {
            //The output in TUM belongs to Ow.
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line;
            int KFID, FrameID;
            double X, Y, Z, R11, R12, R13, R21, R22, R23, R31, R32, R33;

            TXTdir = OFD.FileName;
            Savedir = Path.GetDirectoryName(TXTdir);
            File_XYZ_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            TXTfile.ReadLine(); //header

            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] KFInfo = line.Split(' ');

                KFID = Convert.ToInt32(KFInfo[0]);
                X = double.Parse(KFInfo[2]);
                Y = double.Parse(KFInfo[3]);
                Z = double.Parse(KFInfo[4]);
                R11 = double.Parse(KFInfo[5]);
                R12 = double.Parse(KFInfo[6]);
                R13 = double.Parse(KFInfo[7]);
                R21 = double.Parse(KFInfo[8]);
                R22 = double.Parse(KFInfo[9]);
                R23 = double.Parse(KFInfo[10]);
                R31 = double.Parse(KFInfo[11]);
                R32 = double.Parse(KFInfo[12]);
                R33 = double.Parse(KFInfo[13]);
                FrameID = Convert.ToInt32(KFInfo[15]);

                RotationMatrix R = new RotationMatrix(R11, R12, R13, R21, R22, R23, R31, R32, R33);
                ExOrietation EO = new ExOrietation(X, Y, Z, R);

                KeyFrame KF = new KeyFrame(KFID, EO, FrameID);
                KFList.Add(KF);
            }
        }

        private void Load_Trajectory_1_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line;
            int FrameID = 0;
            double X97, Y97, Z97;

            TXTdir = OFD.FileName;
            File_XYZ_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);

            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] KFInfo = line.Split(' ');

                X97 = double.Parse(KFInfo[1]);
                Y97 = double.Parse(KFInfo[2]);
                Z97 = double.Parse(KFInfo[3]);

                ControlPoint KF = new ControlPoint(KFList[FrameID].KFID, FrameID, X97, Y97, Z97, true); FrameID += 1;
                KFComapre.Add(KF);
            }
        }

        private void Load_Trajectory_2_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog OFD = new OpenFileDialog();
            OFD.ShowDialog();

            string line;
            int FrameID = 0;
            double Xind, Yind, Zind;

            TXTdir = OFD.FileName;
            File_XYZ_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);

            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] KFInfo = line.Split(' ');

                Xind = double.Parse(KFInfo[1]);
                Yind = double.Parse(KFInfo[2]);
                Zind = double.Parse(KFInfo[3]);

                KFComapre[FrameID].CPXind = Xind;
                KFComapre[FrameID].CPYind = Yind;
                KFComapre[FrameID].CPZind = Zind;

                FrameID += 1;
            }
        }

        private void Batch_Compare_Click(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < 6; i++)
            {
                Weighting_Type = i;
                Trajectory_Compare_Click(sender, e);
                Traj_Transformation_Batch(Savedir + "\\SLAM_Traj_TUM.txt");
                Map_Transformation_Batch(Savedir + "\\MP_XYZ.txt");
            }
        }

        private void Batch_Transform_CP_Click(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < 6; i++)
            {
                Weighting_Type = i;
                MT_Adjustment_Click(sender, e);
                Traj_Transformation_Batch(Savedir + "\\SLAM_Traj_TUM.txt");
                Map_Transformation_Batch(Savedir + "\\MP_XYZ.txt");
            }
        }

        private void Traj_Transformation_Batch(string FileRoute)
        {
            string line, mpid;
            double mpx, mpy, mpz;
            List<ControlPoint> MPList = new List<ControlPoint>();

            TXTdir = FileRoute;
            File_Map_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            Matrix<double> XYZ = DenseMatrix.Create(3, 1, 0);
            Matrix<double> ENH;
            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] MPInfo = line.Split(' ');

                mpid = MPInfo[0];
                mpx = Convert.ToDouble(MPInfo[1]); XYZ[0, 0] = mpx;
                mpy = Convert.ToDouble(MPInfo[2]); XYZ[1, 0] = mpy;
                mpz = Convert.ToDouble(MPInfo[3]); XYZ[2, 0] = mpz;

                ENH = Scale7para * Rot7para.Multiply(XYZ) + Trans7para.Transpose();

                ControlPoint MP = new ControlPoint(mpid, ENH[0, 0], ENH[1, 0], ENH[2, 0]);
                MPList.Add(MP);
            }

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Traj" + "_" + Convert.ToString(Weighting_Type) + "_Transformed.txt", FileMode.Create, FileAccess.Write))
            {
                double ToDeg = Convert.ToInt32(180f / Math.PI);

                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    foreach (ControlPoint mp in MPList) sb.AppendLine(mp.CPID + " " + ToStr(mp.CPX97) + " " + ToStr(mp.CPY97) + " " + ToStr(mp.CPZ97));

                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }

        private void Map_Transformation_Batch(string FileRoute)
        {
            //OpenFileDialog OFD = new OpenFileDialog();
            //OFD.ShowDialog();
            
            string line, mpid;
            double mpx, mpy, mpz;
            List<ControlPoint> MPList = new List<ControlPoint>();

            TXTdir = FileRoute;
            File_Map_name.Text = TXTdir;
            StreamReader TXTfile = new StreamReader(TXTdir);
            Matrix<double> XYZ = DenseMatrix.Create(3, 1, 0);
            Matrix<double> ENH;
            while ((line = TXTfile.ReadLine()) != null)
            {
                string[] MPInfo = line.Split(' ');

                mpid = MPInfo[0];
                mpx = Convert.ToDouble(MPInfo[1]); XYZ[0, 0] = mpx;
                mpy = Convert.ToDouble(MPInfo[2]); XYZ[1, 0] = mpy;
                mpz = Convert.ToDouble(MPInfo[3]); XYZ[2, 0] = mpz;

                ENH = Scale7para * Rot7para.Multiply(XYZ) + Trans7para.Transpose();

                ControlPoint MP = new ControlPoint(mpid, ENH[0, 0], ENH[1, 0], ENH[2, 0]);
                MPList.Add(MP);
            }

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Map" + "_" + Convert.ToString(Weighting_Type) +  "_Transformed.txt", FileMode.Create, FileAccess.Write))
            {
                double ToDeg = Convert.ToInt32(180f / Math.PI);

                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    foreach (ControlPoint mp in MPList) sb.AppendLine(mp.CPID + " " + ToStr(mp.CPX97) + " " + ToStr(mp.CPY97) + " " + ToStr(mp.CPZ97));

                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }

        private void Trajectory_Compare_Click(object sender, RoutedEventArgs e)
        {
            double[] MassCenter = MassCenterCaculation(KFComapre);
            Matrix<double> RA = DenseMatrix.Create(3, 3, 0); ;
            double Apx_Scale = 999; //double minscore = 999999999999; int PA = 0; int PB = 0; int PC = 0;
            
            /*
            for (int i = (int)(2 * KFComapre.Count / 3); i < KFComapre.Count; i++){
                for (int j = (int)(KFComapre.Count / 3); j < (int)(2 * KFComapre.Count / 3); j++){
                    if (i <= j) break;
                    for (int k = 0; k < (int)(KFComapre.Count/3); k++){
                        if (j <= k || i <= k) break;

                        Apx_Scale = ApproximateScale(KFComapre[i], KFComapre[j], KFComapre[k]);
                        double scores = InitialValueTraj(KFComapre[i], KFComapre[j], KFComapre[k], MassCenter, RA, Apx_Scale);

                        if (scores < minscore){
                            minscore = scores;
                            PA = i; PB = j; PC = k;
                        }}}}*/

            ControlPoint PointA = KFComapre[0];
            ControlPoint PointB = KFComapre[50];
            ControlPoint PointC = KFComapre[100];

            RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter);
            double[,] Umeyama = { { -0.0301624, -0.4414884, -0.99856953 }, { 0.9992328, 0.02363594, -0.03122742 }, { -0.02498078, -0.99874532, 0.04340205 } };
            RA = DenseMatrix.OfArray(Umeyama);
            //initial Rotation
            double Phi = Math.Asin(RA[2, 0]);
            double Omega = Math.Asin(-RA[2, 1] / cos(Phi));
            double Kappa = Math.Asin(-RA[1, 0] / cos(Phi));

            //Caculate Approximate Scale
            Apx_Scale = ApproximateScale(PointA, PointB, PointC);

            Matrix<double> A = DenseMatrix.Create(3 * KFComapre.Count, 7, 0);
            Matrix<double> B = DenseMatrix.Create(3 * KFComapre.Count, 3 * KFComapre.Count, 0);
            Matrix<double> W = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> X = DenseMatrix.Create(7, 1, 0);
            Matrix<double> P = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> V = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> T = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Xcorr = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Xcorr0 = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Qxx = DenseMatrix.Create(7, 7, 0);
            Matrix<double> Qvv = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> QLL = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> Qww = DenseMatrix.Create(3 * KFComapre.Count, 3 * KFComapre.Count, 0);
            Matrix<double> Qwwinv, AT, BT, VT; double sigma0 = 0, sigma0_0 = 0; int count = 1;

            //First caculation by initial value
            T = Translation(Apx_Scale, RA, MassCenter);
            X[0, 0] = T[0, 0]; X[1, 0] = T[1, 0]; X[2, 0] = T[2, 0]; X[3, 0] = Omega; X[4, 0] = Phi; X[5, 0] = Kappa; X[6, 0] = Apx_Scale;
            //X[0, 0] = T[0, 0]; X[1, 0] = T[1, 0]; X[2, 0] = T[2, 0]; X[3, 0] = 1.52; X[4, 0] = 0.025; X[5, 0] = -1.6; X[6, 0] = Apx_Scale;

            //Create the necessary matrix for adjustment
            A = CreateA_MTA(X, KFComapre); B = CreateB_MTA_NoCP(X, KFComapre); W = CreateW_MTA_B(B, X, KFComapre); AT = A.Transpose(); BT = B.Transpose();
            QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();

            //Adjustment
            Xcorr = (AT.Multiply(Qwwinv).Multiply(A)).Inverse().Multiply(AT.Multiply(Qwwinv).Multiply(W));            

            for (int i = 0; i < 50; i++)
            {
                V = QLL.Multiply(BT).Multiply(Qwwinv).Multiply(W - A.Multiply(Xcorr)); VT = V.Transpose();
                //sigma0 = Math.Sqrt(VT.Multiply(P).Multiply(V)[0, 0] / (V.RowCount - 7));
                sigma0 = Sigma0(V, P, Weighting_Type);

                //Blunder Detection
                Qxx = (AT.Multiply(Qwwinv).Multiply(A)).Inverse();
                Qvv = P.Multiply(BT).Multiply(Qwwinv - Qwwinv.Multiply(A.Multiply(Qxx).Multiply(AT).Multiply(Qwwinv))).Multiply(B.Multiply(QLL));
                //MessageBox.Show(Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.065)));

                //Parameter Update
                X = X + Xcorr; Xcorr0 = Xcorr; sigma0_0 = sigma0; 
                P = CreateP_MTA(P, V, sigma0, Weighting_Type, count, Qvv, KFComapre); count += 1;

                //Convergence Check. Go into another iteration if not convergent.
                A = CreateA_MTA(X, KFComapre); B = CreateB_MTA_NoCP(X, KFComapre); W = CreateW_MTA_B(B, X, KFComapre); AT = A.Transpose(); BT = B.Transpose();
                QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();
                Xcorr = (AT.Multiply(Qwwinv).Multiply(A)).Inverse().Multiply(AT.Multiply(Qwwinv).Multiply(W));

                if (Convergence(Xcorr, Xcorr0, 0.0001)) break;
            }

            //Final Adjustment Result Caculation and set as public var for map transformation
            double[,] Trans = { { X[0, 0], X[1, 0], X[2, 0] } };
            Scale7para = X[6, 0]; Rot7para = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);
            Trans7para = DenseMatrix.OfArray(Trans);

            //Sigma0_97 Caculation
            Matrix<double> Vxyz97 = Vto97(V, KFComapre);
            double sigma0_97 = Sigma0(Vxyz97, P, Weighting_Type);
            //Math.Sqrt((Vxyz97.Transpose().Multiply(P).Multiply(Vxyz97))[0, 0] / (Vxyz97.RowCount - 7));

            MessageBox.Show(Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.065)));
            MessageBox.Show("Sigma0 = " + ToStr(sigma0_0) + "    Sigma0_97 = " + ToStr(sigma0_97) + 
                            "   Iter = " + Convert.ToString(count - 1));

            ResidualEachPoint(V, Weighting_Type, KFComapre);
            
            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Precision_Report_Traj_" + Convert.ToString(Weighting_Type) + ".txt", FileMode.Create, FileAccess.Write))
            {
                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    sb.AppendLine("Outlier Count = " + Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.065))); sb.AppendLine(" ");
                    sb.AppendLine("Sigma0 = " + ToStr(sigma0_0) + "    Sigma0_97 = " + ToStr(sigma0_97) + "   Iter = " + Convert.ToString(count - 1)); sb.AppendLine(" ");
                    sb.AppendLine("Tx = " + ToStr(X[0, 0]) + " Ty = " + ToStr(X[1, 0]) + " Tz = " + ToStr(X[2, 0]));
                    sb.AppendLine("Omega = " + ToStr(X[3, 0]) + " Phi = " + ToStr(X[4, 0]) + " Kappa = " + ToStr(X[5, 0]));
                    sb.AppendLine("Scale = " + ToStr(X[6, 0]));sb.AppendLine(" "); sb.AppendLine(OutlierList); sb.AppendLine(" ");
                    sb.AppendLine(StatisticP(P)); sb.AppendLine(" ");

                    sw.Write(sb.ToString()); sw.Close();
                }
            }
        }

        private string StatisticP(Matrix<double> P)
        {
            string PDetail; string Plocation = "P != 1 : "; int Pequal_1count = 0;
            Matrix<double> Prow = DenseMatrix.Create(P.RowCount, 1, 0);
            double Pmax = -99999999, Pmin = 99999999, Pmean = 0;
            

            for (int i = 0; i < P.RowCount; i++)
            { 
                Prow[i, 0] = P[i, i];

                if (Pmax < P[i, i]) Pmax = P[i, i];
                if (Pmin > P[i, i]) Pmin = P[i, i];

                if (P[i, i] == 1) Pequal_1count += 1;
                else
                {
                    Pmean += P[i, i];
                    Plocation += ToStr(i / 3f) + "/ ";
                }
            }

            Pmean = Pmean / (P.RowCount - Pequal_1count);
            PDetail = "P_not_1 = " + ToStr(P.RowCount - Pequal_1count) + " Pmax = " + ToStr(Pmax) + " Pmin = " + ToStr(Pmin) + " P_not_1_mean = " + ToStr(Pmean) + "\n" + Plocation;

            return PDetail;
        }

        private void MT_Adjustment_Click(object sender, RoutedEventArgs e)
        {
            //MTA = Mixed Type Adjustment

            double[] MassCenter = MassCenterCaculation(CPList);
            Matrix<double> RA = DenseMatrix.Create(3, 3, 0); ;
            double Apx_Scale = 999; //double minscore = 999999999999; int PA = 0; int PB = 0; int PC = 0;

            /*
            for (int i = (int)(2 * KFComapre.Count / 3); i < KFComapre.Count; i++){
                for (int j = (int)(KFComapre.Count / 3); j < (int)(2 * KFComapre.Count / 3); j++){
                    if (i <= j) break;
                    for (int k = 0; k < (int)(KFComapre.Count/3); k++){
                        if (j <= k || i <= k) break;

                        Apx_Scale = ApproximateScale(KFComapre[i], KFComapre[j], KFComapre[k]);
                        double scores = InitialValueTraj(KFComapre[i], KFComapre[j], KFComapre[k], MassCenter, RA, Apx_Scale);

                        if (scores < minscore){
                            minscore = scores;
                            PA = i; PB = j; PC = k;
                        }}}}*/

            ControlPoint PointA = CPList[0];
            ControlPoint PointB = CPList[7];
            ControlPoint PointC = CPList[13];

            RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter);
            //double[,] Umeyama = { { -0.0301624, -0.4414884, -0.99856953 }, { 0.9992328, 0.02363594, -0.03122742 }, { -0.02498078, -0.99874532, 0.04340205 } };
            //RA = DenseMatrix.OfArray(Umeyama);

            //initial Rotation
            double Phi = Math.Asin(RA[2, 0]);
            double Omega = Math.Asin(-RA[2, 1] / cos(Phi));
            double Kappa = Math.Asin(-RA[1, 0] / cos(Phi));

            //Caculate Approximate Scale
            Apx_Scale = ApproximateScale(PointA, PointB, PointC);

            Matrix<double> A = DenseMatrix.Create(3 * CPList.Count, 7, 0);
            Matrix<double> B = DenseMatrix.Create(3 * CPList.Count, 3 * CPList.Count, 0);
            Matrix<double> W = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> X = DenseMatrix.Create(7, 1, 0);
            Matrix<double> P = DenseMatrix.CreateIdentity(3 * CPList.Count);
            Matrix<double> V = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> T = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Xcorr = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Xcorr0 = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Qxx = DenseMatrix.Create(7, 7, 0);
            Matrix<double> Qvv = DenseMatrix.CreateIdentity(3 * CPList.Count);
            Matrix<double> QLL = DenseMatrix.CreateIdentity(3 * CPList.Count);
            Matrix<double> Qww = DenseMatrix.Create(3 * CPList.Count, 3 * CPList.Count, 0);
            Matrix<double> Qwwinv, AT, BT, VT; double sigma0 = 0, sigma0_0 = 0; int count = 1;

            //First caculation by initial value
            T = Translation(Apx_Scale, RA, MassCenter);
            X[0, 0] = T[0, 0]; X[1, 0] = T[1, 0]; X[2, 0] = T[2, 0]; X[3, 0] = Omega; X[4, 0] = Phi; X[5, 0] = -Kappa - Math.PI; X[6, 0] = Apx_Scale;
            //X[0, 0] = T[0, 0]; X[1, 0] = T[1, 0]; X[2, 0] = T[2, 0]; X[3, 0] = 1.52; X[4, 0] = 0.025; X[5, 0] = -1.6; X[6, 0] = Apx_Scale;

            //Create the necessary matrix for adjustment
            A = CreateA_MTA(X, CPList); B = CreateB_MTA_NoCP(X, CPList); W = CreateW_MTA_B(B, X, CPList); AT = A.Transpose(); BT = B.Transpose();
            QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();

            //Adjustment
            Xcorr = (AT.Multiply(Qwwinv).Multiply(A)).Inverse().Multiply(AT.Multiply(Qwwinv).Multiply(W));

            for (int i = 0; i < 50; i++)
            {
                V = QLL.Multiply(BT).Multiply(Qwwinv).Multiply(W - A.Multiply(Xcorr)); VT = V.Transpose();
                //sigma0 = Math.Sqrt(VT.Multiply(P).Multiply(V)[0, 0] / (V.RowCount - 7));
                sigma0 = Sigma0(V, P, Weighting_Type);

                //Blunder Detection
                Qxx = (AT.Multiply(Qwwinv).Multiply(A)).Inverse();
                Qvv = P.Multiply(BT).Multiply(Qwwinv - Qwwinv.Multiply(A.Multiply(Qxx).Multiply(AT).Multiply(Qwwinv))).Multiply(B.Multiply(QLL));
                //MessageBox.Show(Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.1935)));

                //Parameter Update
                X = X + Xcorr; Xcorr0 = Xcorr; sigma0_0 = sigma0;
                P = CreateP_MTA(P, V, sigma0, Weighting_Type, count, Qvv, CPList); count += 1;

                //Convergence Check. Go into another iteration if not convergent.
                A = CreateA_MTA(X, CPList); B = CreateB_MTA_NoCP(X, CPList); W = CreateW_MTA_B(B, X, CPList); AT = A.Transpose(); BT = B.Transpose();
                QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();
                Xcorr = (AT.Multiply(Qwwinv).Multiply(A)).Inverse().Multiply(AT.Multiply(Qwwinv).Multiply(W));

                if (Convergence(Xcorr, Xcorr0, 0.0001)) break;
            }

            //Final Adjustment Result Caculation and set as public var for map transformation
            double[,] Trans = { { X[0, 0], X[1, 0], X[2, 0] } };
            Scale7para = X[6, 0]; Rot7para = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);
            Trans7para = DenseMatrix.OfArray(Trans);

            //Sigma0_97 Caculation
            Matrix<double> Vxyz97 = Vto97(V, CPList);
            double sigma0_97 = Sigma0(Vxyz97, P, Weighting_Type);

            MessageBox.Show(Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.1935))); //95%
            MessageBox.Show("Sigma0 = " + ToStr(sigma0_0) + "    Sigma0_97 = " + ToStr(sigma0_97) +
                            "   Iter = " + Convert.ToString(count - 1));

            ResidualEachPoint(V, Weighting_Type, CPList);

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Precision_Report_Traj_" + Convert.ToString(Weighting_Type) + ".txt", FileMode.Create, FileAccess.Write))
            {
                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    sb.AppendLine("Outlier Count = " + Convert.ToString(Dau_Test(V, Qvv, P, sigma0, 1.1935))); sb.AppendLine(" ");
                    sb.AppendLine("Sigma0 = " + ToStr(sigma0_0) + "    Sigma0_97 = " + ToStr(sigma0_97) + "   Iter = " + Convert.ToString(count - 1)); sb.AppendLine(" ");
                    sb.AppendLine("Tx = " + ToStr(X[0, 0]) + " Ty = " + ToStr(X[1, 0]) + " Tz = " + ToStr(X[2, 0]));
                    sb.AppendLine("Omega = " + ToStr(X[3, 0]) + " Phi = " + ToStr(X[4, 0]) + " Kappa = " + ToStr(X[5, 0]));
                    sb.AppendLine("Scale = " + ToStr(X[6, 0])); sb.AppendLine(" "); sb.AppendLine(OutlierList); sb.AppendLine(" ");
                    sb.AppendLine(StatisticP(P)); sb.AppendLine(" ");

                    sw.Write(sb.ToString()); sw.Close();
                }
            }
        }

        private double InitialValueTraj(ControlPoint PointA, ControlPoint PointB, ControlPoint PointC, double[] MassCenter, Matrix<double> RA, double Apx_Scale)
        {
            RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter);

            Apx_Scale = ApproximateScale(PointA, PointB, PointC);

            Matrix<double> Coordinate = DenseMatrix.Create(3, 1, 0); 
            Matrix<double> ENH = DenseMatrix.Create(3, 1, 0);

            Matrix<double> T = Translation(Apx_Scale, RA, MassCenter);
            List<double> f = new List<double>();
            for (int i = 0; i < KFComapre.Count; i += 5 ) 
            {
                Coordinate[0, 0] = KFComapre[i].CPXind; Coordinate[1, 0] = KFComapre[i].CPYind; Coordinate[2, 0] = KFComapre[i].CPZind;
                ENH[0, 0] = KFComapre[i].CPX97; ENH[1, 0] = KFComapre[i].CPY97; ENH[2, 0] = KFComapre[i].CPZ97;

                T = Apx_Scale * RA.Multiply(Coordinate) + T;
                double dx = Math.Pow(T[0, 0] - ENH[0, 0], 2); double dy = Math.Pow(T[1, 0] - ENH[1, 0], 2); double dz = Math.Pow(T[2, 0] - ENH[2, 0], 2);
                double dxyz = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                f.Add(dxyz);

                //f[i + 0, 0] = T[0, 0] - ENH[0, 0]; f[i + 1, 0] = T[1, 0] - ENH[1, 0]; f[i + 2, 0] = T[2, 0] - ENH[2, 0];
            }

            double score = 0;
            for (int i = 0; i < f.Count; i++) score += f[i];
            return score;
        }

        private void Collinearity_Click(object sender, RoutedEventArgs e)
        {
            List<ControlPoint> CPList_ind = new List<ControlPoint>();

            foreach (ControlPoint cp in CPList)
            {
                Collinearity CL = new Collinearity();
                ExOrietation[] EO = new ExOrietation[cp.KeyPointList.Count];
                ImagePoints.Coordinate[] IMGXY = new ImagePoints.Coordinate[cp.KeyPointList.Count];

                int i = 0;

                foreach (KeyFrame kf in KFList)
                {
                    foreach (KeyPoint kp in cp.KeyPointList)
                    {
                        if (kf.FrameID == kp.FID)
                        {
                            EO[i] = kf.EO;
                            IMGXY[i] = kp.IMGXY;

                            i += 1;
                        }
                    }
                }

                double[] IntersectXYZ = CL.FowardIntersection(EO, IOPSet, IMGXY);
                cp.CPXind = IntersectXYZ[0]; cp.CPYind = IntersectXYZ[1]; cp.CPZind = IntersectXYZ[2];

                ControlPoint CP_ind = new ControlPoint(cp.CPID, IntersectXYZ[0], IntersectXYZ[1], IntersectXYZ[2]);
                CPList_ind.Add(CP_ind);
            }

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Control_Point_Ind_Collinearity.csv", FileMode.Create, FileAccess.Write))
            {
                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    foreach (ControlPoint cp in CPList_ind)
                    {
                        sb.AppendLine(cp.CPID + " " + ToStr(cp.CPX97) + " " + ToStr(cp.CPY97) + " " + ToStr(cp.CPZ97));
                    }
                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }        

        private Matrix<double> CreateA_MTA(Matrix<double> X0, List<ControlPoint> CPList)
        {
            Matrix<double> A = DenseMatrix.Create(3 * CPList.Count, 7, 0);

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                double[] partialvalue = LinearizedEquationMTA(X0, cp.CPXind, cp.CPYind, cp.CPZind);

                A[i * 3 + 0, 3] = partialvalue[0]; A[i * 3 + 0, 4] = partialvalue[1]; A[i * 3 + 0, 5] = partialvalue[2 ]; A[i * 3 + 0, 6] = partialvalue[3 ];
                A[i * 3 + 1, 3] = partialvalue[4]; A[i * 3 + 1, 4] = partialvalue[5]; A[i * 3 + 1, 5] = partialvalue[6 ]; A[i * 3 + 1, 6] = partialvalue[7 ];
                A[i * 3 + 2, 3] = partialvalue[8]; A[i * 3 + 2, 4] = partialvalue[9]; A[i * 3 + 2, 5] = partialvalue[10]; A[i * 3 + 2, 6] = partialvalue[11];

                A[i * 3 + 0, 0] = 1; A[i * 3 + 0, 1] = 0; A[i * 3 + 0, 2] = 0;
                A[i * 3 + 1, 0] = 0; A[i * 3 + 1, 1] = 1; A[i * 3 + 1, 2] = 0;
                A[i * 3 + 2, 0] = 0; A[i * 3 + 2, 1] = 0; A[i * 3 + 2, 2] = 1;
            }

            return A;
        }

        private double[] LinearizedEquationMTA(Matrix<double> X0, double x, double y, double z)
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33, XpS, XpOmega, XpPhi, XpKappa, YpS, YpOmega, YpPhi, YpKappa, ZpS, ZpOmega, ZpPhi, ZpKappa, Xpx, Xpy, Xpz, Ypx, Ypy, Ypz, Zpx, Zpy, Zpz;
            double s = X0[6, 0]; double omega = X0[3, 0]; double phi = X0[4, 0]; double kappa = X0[5, 0]; double[] partialresult = new double[21];

            m11 = cos(phi) * cos(kappa); m12 = sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa); m13 = -cos(omega) * sin(phi) * cos(kappa) + sin(omega) * sin(kappa);
            m21 = -cos(phi) * sin(kappa); m22 = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa); m23 = cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);
            m31 = sin(phi); m32 = -sin(omega) * cos(phi); m33 = cos(omega) * cos(phi);

            XpS = m11 * x + m12 * y + m13 * z;
            XpOmega = s * ((cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * y + (sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa)) * z);
            XpPhi = s * (-sin(phi) * cos(kappa) * x + (sin(omega) * cos(phi) * cos(kappa)) * y + (-cos(omega) * cos(phi) * cos(kappa)) * z);
            XpKappa = s * (-cos(phi) * sin(kappa) * x + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * y + (cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa)) * z);

            YpS = m21 * x + m22 * y + m23 * z;
            YpOmega = s * ((-cos(omega) * sin(phi) * sin(kappa) - sin(omega) * cos(kappa)) * y + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * z);
            YpPhi = s * (sin(phi) * sin(kappa) * x + (-sin(omega) * cos(phi) * sin(kappa)) * y + (cos(omega) * cos(phi) * sin(kappa)) * z);
            YpKappa = s * (-cos(phi) * cos(kappa) * x + (-sin(omega) * sin(phi) * cos(kappa) - cos(omega) * sin(kappa)) * y + (cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * z);

            ZpS = m31 * x + m32 * y + m33 * z;
            ZpOmega = s * (-cos(omega) * cos(phi) * y + -sin(omega) * cos(phi) * z);
            ZpPhi = s * (cos(phi) * x + sin(omega) * sin(phi) * y + -cos(omega) * sin(phi) * z);
            ZpKappa = s * (0);

            Xpx = s * m11; Xpy = s * m12; Xpz = s * m13;
            Ypx = s * m21; Ypy = s * m22; Ypz = s * m23;
            Zpx = s * m31; Zpy = s * m32; Zpz = s * m33;

            partialresult[0] = XpOmega; partialresult[1] = XpPhi; partialresult[2] = XpKappa; partialresult[3] = XpS;
            partialresult[4] = YpOmega; partialresult[5] = YpPhi; partialresult[6] = YpKappa; partialresult[7] = YpS;
            partialresult[8] = ZpOmega; partialresult[9] = ZpPhi; partialresult[10] = ZpKappa; partialresult[11] = ZpS;

            partialresult[12] = Xpx; partialresult[13] = Xpy; partialresult[14] = Xpz;
            partialresult[15] = Ypx; partialresult[16] = Ypy; partialresult[17] = Ypz;
            partialresult[18] = Zpx; partialresult[19] = Zpy; partialresult[20] = Zpz;

            return partialresult;
        }

        private Matrix<double> CreateB_MTA_NoCP(Matrix<double> X0, List<ControlPoint> CPList)
        {
            Matrix<double> B = DenseMatrix.Create(3 * CPList.Count, 3 * CPList.Count, 0);

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                double[] partialvalue = LinearizedEquationMTA(X0, cp.CPXind, cp.CPYind, cp.CPZind);

                B[3 * i + 0, 3 * i + 0] = partialvalue[12]; B[3 * i + 0, 3 * i + 1] = partialvalue[13]; B[3 * i + 0, 3 * i + 2] = partialvalue[14];
                B[3 * i + 1, 3 * i + 0] = partialvalue[15]; B[3 * i + 1, 3 * i + 1] = partialvalue[16]; B[3 * i + 1, 3 * i + 2] = partialvalue[17];
                B[3 * i + 2, 3 * i + 0] = partialvalue[18]; B[3 * i + 2, 3 * i + 1] = partialvalue[19]; B[3 * i + 2, 3 * i + 2] = partialvalue[20];

                //B[3 * i + 0, 6 * i + 3] = -1; B[3 * i + 0, 6 * i + 4] =  0; B[3 * i + 0, 6 * i + 5] =  0;
                //B[3 * i + 1, 6 * i + 3] =  0; B[3 * i + 1, 6 * i + 4] = -1; B[3 * i + 1, 6 * i + 5] =  0;
                //B[3 * i + 2, 6 * i + 3] =  0; B[3 * i + 2, 6 * i + 4] =  0; B[3 * i + 2, 6 * i + 5] = -1;
            }

            return B;
        }

        private Matrix<double> CreateW_MTA_B(Matrix<double> B, Matrix<double> X0, List<ControlPoint> CPList)
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33;
            double s = X0[6, 0]; double omega = X0[3, 0]; double phi = X0[4, 0]; double kappa = X0[5, 0];
            //double[] partialresult = new double[21];
            //Matrix<double> L = DenseMatrix.Create(6 * KFComapre.Count, 1, 0);
            Matrix<double> L = DenseMatrix.Create(3 * CPList.Count, 1, 0);

            m11 =  cos(phi) * cos(kappa); m12 =  sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa); m13 = -cos(omega) * sin(phi) * cos(kappa) + sin(omega) * sin(kappa);
            m21 = -cos(phi) * sin(kappa); m22 = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa); m23 =  cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);
            m31 =  sin(phi);              m32 = -sin(omega) * cos(phi);                                        m33 =  cos(omega) * cos(phi);

            Matrix<double> W = DenseMatrix.Create(3 * CPList.Count, 1, 0);

            for (int i = 0; i < CPList.Count; i++) 
            {
                ControlPoint cp = CPList[i];
                //L[6 * i + 0, 0] = cp.CPXind; L[6 * i + 1, 0] = cp.CPYind; L[6 * i + 2, 0] = cp.CPZind;
                //L[6 * i + 3, 0] = cp.CPX97 ; L[6 * i + 4, 0] = cp.CPY97 ; L[6 * i + 5, 0] = cp.CPZ97 ;
                L[3 * i + 0, 0] = cp.CPXind; L[3 * i + 1, 0] = cp.CPYind; L[3 * i + 2, 0] = cp.CPZind;
            }

            //Matrix<double> BL = B.Multiply(L);

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];

                //W[3 * i + 0, 0] = (s * m11 * cp.CPXind + s * m12 * cp.CPYind + s * m13 * cp.CPZind + X0[0, 0]) + BL[3 * i + 0, 0];
                //W[3 * i + 1, 0] = (s * m21 * cp.CPXind + s * m22 * cp.CPYind + s * m23 * cp.CPZind + X0[1, 0]) + BL[3 * i + 1, 0];
                //W[3 * i + 2, 0] = (s * m31 * cp.CPXind + s * m32 * cp.CPYind + s * m33 * cp.CPZind + X0[2, 0]) + BL[3 * i + 2, 0];
                W[3 * i + 0, 0] = cp.CPX97 - (s * m11 * cp.CPXind + s * m12 * cp.CPYind + s * m13 * cp.CPZind + X0[0, 0]);
                W[3 * i + 1, 0] = cp.CPY97 - (s * m21 * cp.CPXind + s * m22 * cp.CPYind + s * m23 * cp.CPZind + X0[1, 0]);
                W[3 * i + 2, 0] = cp.CPZ97 - (s * m31 * cp.CPXind + s * m32 * cp.CPYind + s * m33 * cp.CPZind + X0[2, 0]);
                //W[3 * i + 0, 0] = - cp.CPX97 + (s * m11 * cp.CPXind + s * m12 * cp.CPYind + s * m13 * cp.CPZind + X0[0, 0]);
                //W[3 * i + 1, 0] = - cp.CPY97 + (s * m21 * cp.CPXind + s * m22 * cp.CPYind + s * m23 * cp.CPZind + X0[1, 0]);
                //W[3 * i + 2, 0] = - cp.CPZ97 + (s * m31 * cp.CPXind + s * m32 * cp.CPYind + s * m33 * cp.CPZind + X0[2, 0]);
                //W[3 * i + 0, 0] = cp.CPX97 - BL[3 * i + 0, 0] - X0[0, 0];
                //W[3 * i + 1, 0] = cp.CPY97 - BL[3 * i + 1, 0] - X0[1, 0];
                //W[3 * i + 2, 0] = cp.CPZ97 - BL[3 * i + 2, 0] - X0[2, 0];
            }

            return W;
        }

        private double Sigma0(Matrix<double> V, Matrix<double> P, int WeightingType)
        {
            double sigma0;

            sigma0 = Math.Sqrt(V.Transpose().Multiply(P).Multiply(V)[0, 0] / (V.RowCount - 7));
            //if (WeightingType == 0) sigma0 = Math.Sqrt(V.Transpose().Multiply(P).Multiply(V)[0, 0] / (V.RowCount - 7));
            //else sigma0 = Math.Sqrt(V.Transpose().Multiply(P).Multiply(V)[0, 0] / ( P.Trace() * (V.RowCount - 7) ) );

            return sigma0;
        }

        private Matrix<double> CreateP_MTA(Matrix<double> P, Matrix<double> V, double sigma0, int type, int count, Matrix<double> Qvv, List<ControlPoint> CPList)
        {
            double k, a, c, u;

            switch (type)
            {                
                case 0: //Equal Weight
                    break;

                /*case 1: //Different Weight for XYZ and ENH

                    for (int i = 0; i < P.RowCount; i++)
                    {
                        if (i % 6 > 2) P[i, i] = 10;
                    }

                    break;*/

                case 1: //Min Residual Abs

                    for (int i = 0; i < P.RowCount; i++)
                    {
                        P[i, i] = Math.Abs(1 / V[i, 0]);
                    }

                    break;

                case 2: //Min Norm

                    k = 0; c = 0.00001;

                    for (int i = 0; i < P.RowCount; i++)
                    {
                        P[i, i] = 1 / (Math.Pow(Math.Abs(V[i, 0]), 2 - k) + c);
                    }

                    break;

                case 3: //Danish 01  (Kubik et al. 1988)

                    a = 1; c = 0.5;

                    for (int i = 0; i < P.RowCount; i++)
                    {
                        if (Math.Abs(V[i, 0]/sigma0) <= 2) P[i, i] = 1;
                        else P[i, i] = a * Math.Pow(Math.E, -c * V[i, 0] * V[i, 0]);
                    }

                    break;

                case 4: //Huber

                    Matrix<double> Vmed = DenseMatrix.Create(3 * CPList.Count, 1, MedianV(V));
                    Matrix<double> Vabs = DenseMatrix.Create(3 * CPList.Count, 1, 0);                    

                    for (int i = 0; i < V.RowCount; i++)
                    {
                        Vabs[i, 0] = Math.Abs((V - Vmed)[i, 0]);
                    }

                    double Vabsmed = MedianV(Vabs); c = 1.345;

                    for (int i = 0; i < P.RowCount; i++)
                    {
                        u = V[i, 0] / (Vabsmed / 0.6745);
                        if (c < Math.Abs(u)) P[i, i] = c / Math.Abs(u);
                        else P[i, i] = 1;
                    }

                    break;

                case 5: //Weight Iteration  (Lee, 1984)

                    double Ti;

                    if (count <= 3) k = 1;
                    else k = 3.29;

                    for (int i = 0; i < V.RowCount; i++)
                    {
                        Ti = V[i, 0] * V[i, 0] / (sigma0 * sigma0 * Qvv[i, i]);

                        if (Math.Sqrt(Ti) <= k) P[i, i] = 1;
                        else P[i, i] = 1 / Ti;
                    }

                    break;
            }

            return P;
        }

        private double MedianV(Matrix<double> V)
        {
            Matrix<double> VSort = DenseMatrix.Create(V.RowCount, 1, 0);
            Matrix<double> V0 = V.Clone();
            double med; double max; int max_location = 0;

            for (int i = 0; i < V.RowCount; i++)
            {
                max = -8888888888;
                for (int j = 0; j < V0.RowCount; j++)
                {
                    if (V0[j, 0] > max)
                    {
                        max = V[j, 0];
                        max_location = j;
                    }
                }

                VSort[i, 0] = max;
                V0[max_location, 0] = -999999999999;
            }

            if (VSort.RowCount % 2 == 0) med = 0.5 * VSort[VSort.RowCount / 2, 0] + 0.5 * VSort[(VSort.RowCount - 2) / 2, 0];
            else med = VSort[(VSort.RowCount - 1) / 2, 0];

            return med;
        }

        private Matrix<double> Vto97(Matrix<double> Vxyz, List<ControlPoint> CPList)
        {
            Matrix<double> Vxyzto97 = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Vxyz97 = DenseMatrix.Create(3 * CPList.Count, 1, 0);

            for (int i = 0; i < CPList.Count; i++)
            {
                Vxyzto97[0, 0] = Vxyz[3 * i + 0, 0]; Vxyzto97[1, 0] = Vxyz[3 * i + 1, 0]; Vxyzto97[2, 0] = Vxyz[3 * i + 2, 0];

                Vxyzto97 = Scale7para * Rot7para.Multiply(Vxyzto97);
                Vxyz97[3 * i + 0, 0] = Vxyzto97[0, 0];
                Vxyz97[3 * i + 1, 0] = Vxyzto97[1, 0];
                Vxyz97[3 * i + 2, 0] = Vxyzto97[2, 0];
            }

            return Vxyz97;
        }

        private void ResidualEachPoint(Matrix<double> V, int Weighting_Type, List<ControlPoint> CPList)
        {
            Matrix<double> Vxyz = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> V97 = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> Vxyzto97 = DenseMatrix.Create(3, 1, 0);
            Matrix<double> D = DenseMatrix.Create(CPList.Count, 1, 0);
            Matrix<double> D97 = DenseMatrix.Create(CPList.Count, 1, 0);

            double maxD = -99999999, minD = 99999999, meanD = 0, medD, stdD = 0, SSED = 0, RMSED = 0;
            double maxD97 = -99999999, minD97 = 99999999, meanD97 = 0, medD97, stdD97 = 0, SSED97 = 0, RMSED97 = 0;
            double vmaxE97 = -99999999, vminE97 = 99999999, vmaxN97 = -99999999, vminN97 = 99999999, vmaxH97 = -99999999, vminH97 = 99999999;

            for (int i = 0; i < CPList.Count; i++)
            {
                Vxyz[3 * i + 0, 0] = V[3 * i + 0, 0]; Vxyzto97[0, 0] = V[3 * i + 0, 0];
                Vxyz[3 * i + 1, 0] = V[3 * i + 1, 0]; Vxyzto97[1, 0] = V[3 * i + 1, 0];
                Vxyz[3 * i + 2, 0] = V[3 * i + 2, 0]; Vxyzto97[2, 0] = V[3 * i + 2, 0];

                Vxyzto97 = Scale7para * Rot7para.Multiply(Vxyzto97);
                V97[3 * i + 0, 0] = Vxyzto97[0, 0];
                V97[3 * i + 1, 0] = Vxyzto97[1, 0];
                V97[3 * i + 2, 0] = Vxyzto97[2, 0];

                D[i, 0] = Math.Sqrt(V[3 * i + 0, 0] * V[3 * i + 0, 0] + V[3 * i + 1, 0] * V[3 * i + 1, 0] + V[3 * i + 2, 0] * V[3 * i + 2, 0]);
                D97[i, 0] = Math.Sqrt(V97[3 * i + 0, 0] * V97[3 * i + 0, 0] + V97[3 * i + 1, 0] * V97[3 * i + 1, 0] + V97[3 * i + 2, 0] * V97[3 * i + 2, 0]);
                
                if (D[i, 0] > maxD) maxD = D[i, 0]; if (D97[i, 0] > maxD97) maxD97 = D97[i, 0];
                if (D[i, 0] < minD) minD = D[i, 0]; if (D97[i, 0] < minD97) minD97 = D97[i, 0];
                if (Vxyzto97[0, 0] > vmaxE97) vmaxE97 = Vxyzto97[0, 0]; if (Vxyzto97[0, 0] < vminE97) vminE97 = Vxyzto97[0, 0];
                if (Vxyzto97[1, 0] > vmaxN97) vmaxN97 = Vxyzto97[1, 0]; if (Vxyzto97[1, 0] < vminN97) vminN97 = Vxyzto97[1, 0];
                if (Vxyzto97[2, 0] > vmaxH97) vmaxH97 = Vxyzto97[2, 0]; if (Vxyzto97[2, 0] < vminH97) vminH97 = Vxyzto97[2, 0];
            }

            medD = MedianV(D); medD97 = MedianV(D97);

            for (int i = 0; i < D.RowCount; i++)
            {
                SSED   += D[i, 0] * D[i, 0];     meanD += D[i, 0]; 
                SSED97 += D97[i, 0] * D97[i, 0]; meanD97 += D97[i, 0];
            }

            meanD = meanD / CPList.Count; 
            meanD97 = meanD97 / CPList.Count;

            stdD = Math.Sqrt(SSED/ D.RowCount - meanD * meanD);
            stdD97 = Math.Sqrt(SSED97/ D.RowCount - meanD97 * meanD97);

            RMSED = Math.Sqrt(SSED / D.RowCount);
            RMSED97 = Math.Sqrt(SSED97 / D97.RowCount);

            /*MessageBox.Show(" Max = " + ToStr(maxD) + " Min = " + ToStr(minD) + " Mean = " + ToStr(meanD) +
                            " Median = " + ToStr(medD) + " STD = " + ToStr(stdD));*/

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Residuals_Control_Points_" + Convert.ToString(Weighting_Type)+".txt", FileMode.Create, FileAccess.Write))
            {
                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    sb.AppendLine("Max_ind = " + ToStr(maxD) + " Min_ind = " + ToStr(minD) + " Mean_ind = " + ToStr(meanD) +
                                  " Median_ind = " + ToStr(medD) + " STD_ind = " + ToStr(stdD) + " RMSE  = " + ToStr(RMSED) + " SSE_ind  = " + ToStr(SSED));
                    sb.AppendLine("VmaxE = " + ToStr(vmaxE97) + " VminE = " + ToStr(vminE97) + " VmaxN = " + ToStr(vmaxN97) +
                                  " VminN = " + ToStr(vminN97) + " VmaxH = " + ToStr(vmaxH97) + " VminH = " + ToStr(vminH97));
                    sb.AppendLine("Max_97  = " + ToStr(maxD97) + " Min_97  = " + ToStr(minD97) + " Mean_97  = " + ToStr(meanD97) +
                                  " Median_97  = " + ToStr(medD97) + " STD_97  = " + ToStr(stdD97) + " RMSE_97  = " + ToStr(RMSED97) + " SSE_97  = " + ToStr(SSED97)); sb.AppendLine(" ");
                    sb.AppendLine("CPID" + " " + "Vx" + " " + "Vy" + " " + "Vz" + " " + "Vxyz" + " " + "V97x" + " " + "V97y" + " " + "V97z" + " " + "V97xyz" + " " + "E" + " " + "N" + " " + "H");
                    
                    for (int i = 0; i < CPList.Count; i++)
                    {
                        sb.AppendLine(CPList[i].CPID + " " + ToStr(Vxyz[3 * i + 0, 0]) + " " + ToStr(Vxyz[3 * i + 1, 0]) + " " + ToStr(Vxyz[3 * i + 2, 0]) + " " + ToStr(D[i, 0]) + " " +
                                      ToStr(V97[3 * i + 0, 0]) + " " + ToStr(V97[3 * i + 1, 0]) + " " + ToStr(V97[3 * i + 2, 0]) + " " + ToStr(D97[i, 0]) + " " +
                                      ToStr(CPList[i].CPX97) + " " + ToStr(CPList[i].CPY97) + " " + ToStr(CPList[i].CPZ97));
                    }

                    sw.Write(sb.ToString()); sw.Close();
                }
            }            
        }

        private double Dau_Test(Matrix<double> V, Matrix<double> Qvv, Matrix<double> P, double sigma0, double dau_Score)
        {
            double count = 0; double sigmaLi = 0; double dau = 0; double root_ri = 0;
            Matrix<double> QvvP = Qvv.Multiply(P); string outlier = "Outlier = ";

            for (int i = 0; i < V.RowCount; i++)
            {
                root_ri = Math.Sqrt(QvvP[i, i]);
                sigmaLi = sigma0 / Math.Sqrt(P[i, i]);
                dau = Math.Abs(V[i, 0]) / (sigmaLi * root_ri);
                if (dau > dau_Score)
                {
                    outlier += " /" + ToStr(i / 3f);
                    count += 1;  //(Dau2 = 2F/(1+F)) 95(F 95,95 1.48) 1.1935 95(F 450,450 1.14) 1.065
                } 
            }

            OutlierList = outlier;
            //MessageBox.Show(outlier);
            return count;
        }

        private bool Convergence(Matrix<double> Xcorr, Matrix<double> Xcorr0, double Threshold_Ratio)
        {
            Matrix<double> dXcorr = Xcorr - Xcorr0; bool Check = true;

            for (int i = 0; i < dXcorr.RowCount; i++)
            {
                if (dXcorr[i, 0] > Threshold_Ratio) Check = false;
            }

            return Check;
        }

        private Matrix<double> Translation(double Scale, Matrix<double> Rot, double[] MassCenter)
        {
            Matrix<double> TENH = DenseMatrix.Create(3, 1, 0);
            Matrix<double> TXYZ = DenseMatrix.Create(3, 1, 0);

            TXYZ[0, 0] = MassCenter[3]; TXYZ[1, 0] = MassCenter[4]; TXYZ[2, 0] = MassCenter[5];
            TENH[0, 0] = MassCenter[0]; TENH[1, 0] = MassCenter[1]; TENH[2, 0] = MassCenter[2];

            TENH = TENH - Scale * Rot.Multiply(TXYZ);

            return TENH;
        }            

        private double InitialValue(ControlPoint PointA, ControlPoint PointB, ControlPoint PointC, double[] MassCenter, Matrix<double> RA, double Apx_Scale)
        {
            RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter);
            Apx_Scale = ApproximateScale(PointA, PointB, PointC);

            //Adjustment
            Matrix<double> f = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> Coordinate = DenseMatrix.Create(3 * CPList.Count, 1, 0);

            Createf(f, RA, Apx_Scale, MassCenter, Coordinate);

            double score = 0;
            for (int i = 0; i < f.RowCount - 1; i++) score += f[i, 0] * f[i, 0];
            return 1 / score;
        }

        //The Code used often but Annoying when Debugging
        private Matrix<double> RotationMatrix(double omega, double phi, double kappa)
        {
            double[,] Omega = { { 1, 0, 0 }, { 0, cos(omega), sin(omega) }, { 0, -sin(omega), cos(omega) } };
            double[,] Phi = { { cos(phi), 0, -sin(phi) }, { 0, 1, 0 }, { sin(phi), 0, cos(phi) } };
            double[,] Kappa = { { cos(kappa), sin(kappa), 0 }, { -sin(kappa), cos(kappa), 0 }, { 0, 0, 1 } };

            Matrix<double> RotO = DenseMatrix.OfArray(Omega);
            Matrix<double> RotP = DenseMatrix.OfArray(Phi);
            Matrix<double> RotK = DenseMatrix.OfArray(Kappa);

            return RotK.Multiply(RotP.Multiply(RotO));
        }

        private Matrix<double> InitRotbyNorm(ControlPoint PointA, ControlPoint PointB, ControlPoint PointC, double[] MassCenter)
        {
            double[] PointA_ind = { PointA.CPXind - MassCenter[3], PointA.CPYind - MassCenter[4], PointA.CPZind - MassCenter[5] };
            double[] PointB_ind = { PointB.CPXind - MassCenter[3], PointB.CPYind - MassCenter[4], PointB.CPZind - MassCenter[5] };
            double[] PointC_ind = { PointC.CPXind - MassCenter[3], PointC.CPYind - MassCenter[4], PointC.CPZind - MassCenter[5] };

            double[] VecA_ind = { PointB_ind[0] - PointA_ind[0], PointB_ind[1] - PointA_ind[1], PointB_ind[2] - PointA_ind[2] };
            double[] VecB_ind = { PointC_ind[0] - PointA_ind[0], PointC_ind[1] - PointA_ind[1], PointC_ind[2] - PointA_ind[2] };
            double LengthA_ind = Math.Sqrt(VecA_ind[0] * VecA_ind[0] + VecA_ind[1] * VecA_ind[1] + VecA_ind[2] * VecA_ind[2]);
            double LengthB_ind = Math.Sqrt(VecB_ind[0] * VecB_ind[0] + VecB_ind[1] * VecB_ind[1] + VecB_ind[2] * VecB_ind[2]);
            double[] Unit_VecA_ind = { VecA_ind[0] / LengthA_ind, VecA_ind[1] / LengthA_ind, VecA_ind[2] / LengthA_ind };
            double[] Unit_VecB_ind = { VecB_ind[0] / LengthB_ind, VecB_ind[1] / LengthB_ind, VecB_ind[2] / LengthB_ind };

            //Caculate the angle between Unit Vector A and Unit Vector B
            double Angle_UAUB_ind = Dot(Unit_VecA_ind, Unit_VecB_ind);

            //Cross the Unit Vector A and Unit Vector B to get Unit Vector P that perpendicular to plane AB
            double[] Unit_VecP_ind = Cross(Unit_VecB_ind, Unit_VecA_ind, Angle_UAUB_ind);

            //Caculate the Unit Vector Q which is perpemdicular to Unit Vector A and Unit Vector P
            double[] Unit_VecQ_ind = Cross(Unit_VecA_ind, Unit_VecP_ind, Math.PI/2);

            //Form the Rotation Matrix R1 with{[Q], [A], [P]}
            Matrix<double> R1 = DenseMatrix.Create(3, 3, 0);
            R1[0, 0] = Unit_VecQ_ind[0]; R1[0, 1] = Unit_VecQ_ind[1]; R1[0, 2] = Unit_VecQ_ind[2];
            R1[1, 0] = Unit_VecA_ind[0]; R1[1, 1] = Unit_VecA_ind[1]; R1[1, 2] = Unit_VecA_ind[2];
            R1[2, 0] = Unit_VecP_ind[0]; R1[2, 1] = Unit_VecP_ind[1]; R1[2, 2] = Unit_VecP_ind[2];

            //Caculate vector a and b and their Unit vector
            double[] PointA_97 = { PointA.CPX97 - MassCenter[0], PointA.CPY97 - MassCenter[1], PointA.CPZ97 - MassCenter[2] };
            double[] PointB_97 = { PointB.CPX97 - MassCenter[0], PointB.CPY97 - MassCenter[1], PointB.CPZ97 - MassCenter[2] };
            double[] PointC_97 = { PointC.CPX97 - MassCenter[0], PointC.CPY97 - MassCenter[1], PointC.CPZ97 - MassCenter[2] };

            double[] VecA_97 = { PointB_97[0] - PointA_97[0], PointB_97[1] - PointA_97[1], PointB_97[2] - PointA_97[2] };
            double[] VecB_97 = { PointC_97[0] - PointA_97[0], PointC_97[1] - PointA_97[1], PointC_97[2] - PointA_97[2] };
            double LengthA_97 = Math.Sqrt(VecA_97[0] * VecA_97[0] + VecA_97[1] * VecA_97[1] + VecA_97[2] * VecA_97[2]);
            double LengthB_97 = Math.Sqrt(VecB_97[0] * VecB_97[0] + VecB_97[1] * VecB_97[1] + VecB_97[2] * VecB_97[2]);
            double[] Unit_VecA_97 = { VecA_97[0] / LengthA_97, VecA_97[1] / LengthA_97, VecA_97[2] / LengthA_97 };
            double[] Unit_VecB_97 = { VecB_97[0] / LengthB_97, VecB_97[1] / LengthB_97, VecB_97[2] / LengthB_97 };

            //Caculate the angle between Unit Vector A and Unit Vector B
            double Angle_UAUB_97 = Dot(Unit_VecA_97, Unit_VecB_97);

            //Cross the Unit Vector A and Unit Vector B to get Unit Vector P that perpendicular to plane AB
            double[] Unit_VecP_97 = Cross(Unit_VecB_97, Unit_VecA_97, Angle_UAUB_97);

            //Caculate the Unit Vector Q which is perpemdicular to Unit Vector A and Unit Vector P
            double[] Unit_VecQ_97 = Cross(Unit_VecA_97, Unit_VecP_97, Math.PI / 2);

            //Form the Rotation Matrix R2 with{[Q], [A], [P]}
            Matrix<double> R2 = DenseMatrix.Create(3, 3, 0);
            R2[0, 0] = Unit_VecQ_97[0]; R2[0, 1] = Unit_VecQ_97[1]; R2[0, 2] = Unit_VecQ_97[2];
            R2[1, 0] = Unit_VecA_97[0]; R2[1, 1] = Unit_VecA_97[1]; R2[1, 2] = Unit_VecA_97[2];
            R2[2, 0] = Unit_VecP_97[0]; R2[2, 1] = Unit_VecP_97[1]; R2[2, 2] = Unit_VecP_97[2];

            //Return the approximate Rotation Matrix RA

            return R2.Transpose().Multiply(R1);
        }

        private double[] MassCenterCaculation(List<ControlPoint> CPList)
        {
            double[] MassCenter = { 0, 0, 0, 0, 0, 0 };
            double X = 0, Y = 0, Z = 0, E = 0, N = 0, H = 0;

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                E += cp.CPX97; N += cp.CPY97; H += cp.CPZ97;
                X += cp.CPXind; Y += cp.CPYind; Z += cp.CPZind;
            }

            MassCenter[0] = E / CPList.Count; MassCenter[1] = N / CPList.Count; MassCenter[2] = H / CPList.Count;
            MassCenter[3] = X / CPList.Count; MassCenter[4] = Y / CPList.Count; MassCenter[5] = Z / CPList.Count;

            return MassCenter;
        }

        private double ApproximateScale(ControlPoint PointA, ControlPoint PointB, ControlPoint PointC)
        {
            double lineAB_97 = Math.Sqrt((PointA.CPX97 - PointB.CPX97) * (PointA.CPX97 - PointB.CPX97) + (PointA.CPY97 - PointB.CPY97) * (PointA.CPY97 - PointB.CPY97) + (PointA.CPZ97 - PointB.CPZ97) * (PointA.CPZ97 - PointB.CPZ97));
            double lineAB_ind = Math.Sqrt((PointA.CPXind - PointB.CPXind) * (PointA.CPXind - PointB.CPXind) + (PointA.CPYind - PointB.CPYind) * (PointA.CPYind - PointB.CPYind) + (PointA.CPZind - PointB.CPZind) * (PointA.CPZind - PointB.CPZind));
            double lineBC_97 = Math.Sqrt((PointB.CPX97 - PointC.CPX97) * (PointB.CPX97 - PointC.CPX97) + (PointB.CPY97 - PointC.CPY97) * (PointB.CPY97 - PointC.CPY97) + (PointB.CPZ97 - PointC.CPZ97) * (PointB.CPZ97 - PointC.CPZ97));
            double lineBC_ind = Math.Sqrt((PointB.CPXind - PointC.CPXind) * (PointB.CPXind - PointC.CPXind) + (PointB.CPYind - PointC.CPYind) * (PointB.CPYind - PointC.CPYind) + (PointB.CPZind - PointC.CPZind) * (PointB.CPZind - PointC.CPZind));
            double lineAC_97 = Math.Sqrt((PointA.CPX97 - PointC.CPX97) * (PointA.CPX97 - PointC.CPX97) + (PointA.CPY97 - PointC.CPY97) * (PointA.CPY97 - PointC.CPY97) + (PointA.CPZ97 - PointC.CPZ97) * (PointA.CPZ97 - PointC.CPZ97));
            double lineAC_ind = Math.Sqrt((PointA.CPXind - PointC.CPXind) * (PointA.CPXind - PointC.CPXind) + (PointA.CPYind - PointC.CPYind) * (PointA.CPYind - PointC.CPYind) + (PointA.CPZind - PointC.CPZind) * (PointA.CPZind - PointC.CPZind));

            return (lineAB_97 / lineAB_ind + lineAC_97 / lineAC_ind + lineBC_97 / lineBC_ind) / 3;
        }

        private double Dot(double[] UVecA, double[] UVecB)
        {
            double Angle, CosAngle;

            CosAngle = UVecA[0] * UVecB[0] + UVecA[1] * UVecB[1] + UVecA[2] * UVecB[2];
            Angle = Math.Acos(CosAngle);

            return Angle;
        }

        private double[] Cross(double[] UVecB, double[] UVecA, double Angle)
        {
            double[] P = { UVecB[1] * UVecA[2] - UVecB[2] * UVecA[1], -UVecB[0] * UVecA[2] + UVecB[2] * UVecA[0], UVecB[0] * UVecA[1] - UVecB[1] * UVecA[0] };
            P[0] = P[0] / Math.Sin(Angle); P[1] = P[1] / Math.Sin(Angle); P[2] = P[2] / Math.Sin(Angle);
            return P;
        }

        private double cos(double a)
        {
            return Math.Cos(a);
        }

        private double sin(double a)
        {
            return Math.Sin(a);
        }
        /// <summary>
        /// The following code are the abandoned code
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void S_P_T_Click(object sender, RoutedEventArgs e)
        {
            //Followed by <3D COORDINATE TRANSFORMATIONS> R E DEAKIN 1998 

            //Caculate Mass Center, [012] for ENH in TWD97, [345] for XYZ in ind. And substract 3 points in Ind. 
            double[] MassCenter = MassCenterCaculation(CPList);
            Matrix<double> RA = DenseMatrix.Create(3, 3, 0); ;
            double Apx_Scale = 999; double maxscore = -99999999; int PA = 0; int PB = 0; int PC = 0;

            for (int i = 2; i < CPList.Count; i++)
            {
                for (int j = 1; j < CPList.Count - 1; j++)
                {
                    if (i <= j) break;

                    for (int k = 0; k < CPList.Count - 2; k++)
                    {
                        if (j <= k || i <= k) break;

                        double scores = InitialValue(CPList[i], CPList[j], CPList[k], MassCenter, RA, Apx_Scale);

                        if (scores > maxscore)
                        {
                            maxscore = scores;
                            PA = i; PB = j; PC = k;
                        }
                    }
                }
            }

            ControlPoint PointA = CPList[PA];
            ControlPoint PointB = CPList[PB];
            ControlPoint PointC = CPList[PC];

            RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter);

            //initial Rotation
            double Phi = Math.Asin(RA[2, 0]);
            double Omega = Math.Asin(-RA[2, 1] / cos(Phi));
            double Kappa = Math.Asin(-RA[1, 0] / cos(Phi));
            //double Kappa = Math.Atan(-RA[1, 0] / RA[0, 0]);
            //double Phi = Math.Acos(RA[0, 0] / Math.Cos(Kappa));
            //double Omega = Math.Acos(RA[2, 2] * Math.Cos(Kappa) / RA[0, 0]);
            //double Phi = Math.Asin(RA[2, 0]);
            //double Omega = -Math.Asin(RA[2, 1] / Math.Cos(Phi));
            //double Kappa = Math.Acos(RA[0, 0] / Math.Cos(Phi));
            //double[] Angle = FindAccurateAngle(RA, Omega, Phi, Kappa);//Mystery01
            //double[] Angle = {Omega, Phi, Kappa};

            //Caculate Approximate Scale
            Apx_Scale = ApproximateScale(PointA, PointB, PointC);

            //Least Squares Adjustment
            Matrix<double> A = DenseMatrix.Create(3 * CPList.Count, 7, 0);
            Matrix<double> X = DenseMatrix.Create(7, 1, 0); //xyzwpks
            Matrix<double> L = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> P = DenseMatrix.CreateIdentity(3 * CPList.Count);
            Matrix<double> Coordinate = DenseMatrix.Create(3 * CPList.Count, 1, 0);
            Matrix<double> Xfinal; Matrix<double> Xcorr; Matrix<double> Xcorr0; 
            Matrix<double> T = Translation(Apx_Scale, RA, MassCenter); Matrix<double> R;
            Matrix<double> StdV; Matrix<double> VENH; Matrix<double> V2ENH;
            double sigma0enh; int DelCount = 0; bool Converge = false;

            //First caculation by initial value
            X[0, 0] = T[0, 0];  X[1, 0] = T[1, 0];  X[2, 0] = T[2, 0]; X[6, 0] = Apx_Scale;

            //double[] Angle = FindAccurateAngle(RA, Omega, Phi, Kappa, X, MassCenter, Coordinate);//Mystery01

            //X[3, 0] = -1.5; X[4, 0] = 0; X[5, 0] = 0;
            X[3, 0] = Math.PI - Omega; X[4, 0] = Math.PI - Phi; X[5, 0] = Kappa;
            //X[3, 0] = Angle[0]; X[4, 0] = Angle[1]; X[5, 0] = Angle[2];
            //R = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);//<-----HEREEEEEE

            CreateAL(A, X, L, MassCenter, CPList); Xfinal = X;
            //Createf(f, R, X[6, 0], MassCenter, Coordinate);

            Matrix<double> f = L - A.Multiply(X);
            Xcorr = (A.Transpose().Multiply(A)).Inverse().Multiply(A.Transpose().Multiply(f));
            Createf(f, RA, X[6, 0], MassCenter, Coordinate);

            VENH = A.Multiply(X + Xcorr) - L;
            MessageBox.Show("RMSE = " + Convert.ToString(RMSEEachPoint(VENH)));

            V2ENH = VENH.Transpose().Multiply(P).Multiply(VENH);
            sigma0enh = Math.Sqrt((V2ENH / (P.Trace() * (VENH.RowCount - 7 - DelCount)))[0, 0]);
            StdV = VENH / sigma0enh;
            CoordinateNow(X, MassCenter, Coordinate);

            for (int i = 0; i < 3 * CPList.Count - 3; i++)
            {
                //Equal
                P[i, i] = 1;

                //Danish
                //int now = Convert.ToSByte(Math.Floor(i / 3f));
                //if (StdV[now, 0] <= 2 * sigma0enh) P[i, i] = 1;
                //else P[i, i] = Math.Pow(Math.E, -Math.Abs(StdV[now, 0]) / (2 * sigma0enh));

                //Weight Iteration
                //if (Math.Abs(StdV[i, 0]) <= 1) P[i, i] = 1;
                //else P[i, i] = 1 / (StdV[i, 0] * StdV[i, 0]);

                //ICG
                //double AbsV = Math.Abs(StdV[i, 0]);
                //if (AbsV > 2.5)
                //{
                //    P[i, i] = 0;
                //    DelCount += 1;
                //}
                //else if (AbsV > 1.5 && AbsV < 2.5) P[i, i] = 1.5 * AbsV;
                //else P[i, i] = P[i, i];
            }

            //Least Squares Adjustment, A for design Matrix, f for L-AX, Xcorr for Correction of parameters, V for residuals of observations.
            //Set the end-up condition as (1) V grows with 5 iterations (2) Xcorr changes < 1%
            for (int i = 0; i < 500; i++)
            {
                Xcorr0 = Xcorr;
                X = X + Xcorr;
                R = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);

                CreateAL(A, X, L, MassCenter, CPList);
                //Createf(f, R, X[6, 0], MassCenter, Coordinate);
                f = L - A.Multiply(X);

                Xcorr = (A.Transpose().Multiply(P).Multiply(A)).Inverse().Multiply(A.Transpose().Multiply(P).Multiply(f));

                VENH = A.Multiply(X + Xcorr) - L;
                MessageBox.Show("RMSE = " + Convert.ToString(RMSEEachPoint(VENH)));

                V2ENH = VENH.Transpose().Multiply(P).Multiply(VENH);
                sigma0enh = Math.Sqrt( (V2ENH / (P.Trace() * (VENH.RowCount - 7 - DelCount)) )[0, 0] );
                StdV = VENH / sigma0enh;

                for (int j = 0; j < 3 * CPList.Count - 3; j++)
                {
                    //Equal
                    P[j, j] = 1;

                    //Danish
                    //int now = Convert.ToSByte(Math.Floor(j / 3f));
                    //if (StdV[now, 0] <= 2 * sigma0enh) P[j, j] = 1;
                    //else P[j, j] = Math.Pow(Math.E, -Math.Abs(StdV[now, 0]) / (2 * sigma0enh));

                    //Weight Iteration
                    //double? k = i > 2 ? 3.29 : 1;
                    //if (Math.Abs(StdV[j, 0]) <= k) P[j, j] = 1;
                    //else P[j, j] = 1 / (StdV[j, 0] * StdV[j, 0]);

                    //ICG
                    //double AbsV = Math.Abs(StdV[i, 0]);
                    //if (AbsV > 2.5)
                    //{
                    //    P[i, i] = 0;
                    //    DelCount += 1;
                    //} 
                    //else if (AbsV > 1.5 && AbsV < 2.5) P[i, i] = 1.5 * AbsV;
                    //else P[i, i] = P[i, i];
                }

                CoordinateNow(X, MassCenter, Coordinate);

                if (Convergence(Xcorr, Xcorr0, 0.0001))
                {
                    Xfinal = X;                  
                    Converge = true;
                    break; 
                }
            }

            if(Converge == false) CreateAL(A, X, L, MassCenter, CPList);

            double[,] Trans = { { 175439.89, 2537357.59, 35.73 } };// { { Xfinal[0, 0], Xfinal[1, 0], Xfinal[2, 0] } };
            R = RotationMatrix(Xfinal[3, 0], Xfinal[4, 0], Xfinal[5, 0]); //Matrix<double>

            //Scale7para = Xfinal[6, 0]; Rot7para = R;
            Scale7para = 13.22; Rot7para = RotationMatrix(-1.61, 3.05, 1.568);
            Trans7para = DenseMatrix.OfArray(Trans);
            Matrix<double> ScaledR = Scale7para * Rot7para;

            MessageBox.Show("RMSE = " + Convert.ToString(RMSEEachPoint(A.Multiply(Xfinal) - L)));

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Seven Parameter.txt", FileMode.Create, FileAccess.Write))
            {
                double ToDeg = Convert.ToInt32(180f / Math.PI);

                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    sb.AppendLine(
                           "Scale : " + ToStr(Xfinal[6, 0])
                        + " Omega : " + ToStr(Xfinal[3, 0]) + "  Phi : " + ToStr(Xfinal[4, 0]) + "  Kappa : " + ToStr(Xfinal[5, 0])
                        + "  Tx : " + ToStr(Xfinal[0, 0]) + "  Ty : " + ToStr(Xfinal[1, 0]) + "  Tz : " + ToStr(Xfinal[2, 0]));

                    sb.AppendLine(ToStr(ScaledR[0, 0]) + " " + ToStr(ScaledR[0, 1]) + " " + ToStr(ScaledR[0, 2]) + " " + ToStr(Xfinal[0, 0]));
                    sb.AppendLine(ToStr(ScaledR[1, 0]) + " " + ToStr(ScaledR[1, 1]) + " " + ToStr(ScaledR[1, 2]) + " " + ToStr(Xfinal[1, 0]));
                    sb.AppendLine(ToStr(ScaledR[2, 0]) + " " + ToStr(ScaledR[2, 1]) + " " + ToStr(ScaledR[2, 2]) + " " + ToStr(Xfinal[2, 0]));
                    sb.AppendLine(ToStr(0) + " " + ToStr(0) + " " + ToStr(0) + " " + ToStr(1));

                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }               

        private double AngleCase(int i, double Angle)
        {
            switch (i)
            {
                case 0:
                    return Angle;
                case 1:
                    return Math.PI - Angle;
                /*case 0: 
                    return Angle;
                case 1:
                    return -Angle;
                case 2:
                    return Angle + Math.PI;
                case 3:
                    return -Angle + Math.PI;*/
            }
            return 0;
        }

        private double[] FindAccurateAngle(Matrix<double> R, double omega, double phi, double kappa, Matrix<double> X, double[] MassCenter, Matrix<double> Coordinate)
        {
            double[] AccurateAngle = { 0, 0, 0 }; double om, ph, ka; double minDet = 99999999999;
            Matrix<double> A = DenseMatrix.Create(3, 7, 0);
            Matrix<double> L = DenseMatrix.Create(3, 1, 0);
            //Matrix<double> t = Translation(X[6, 0], R, MassCenter);
            Matrix<double> xyz = DenseMatrix.Create(3, 1, 0);
            Matrix<double> enh = DenseMatrix.Create(3, 1, 0);
            Matrix<double> f;    double[] partialvalue;
            xyz[0, 0] = MassCenter[3]; xyz[1, 0] = MassCenter[4]; xyz[2, 0] = MassCenter[5];
            enh[0, 0] = MassCenter[0]; enh[1, 0] = MassCenter[1]; enh[2, 0] = MassCenter[2];

            for (int i = 0; i < 2; i++)
            {
                om = AngleCase(i, omega);
                for (int j = 0; j < 2; j++)
                {
                    ph = AngleCase(j, phi);
                    for (int k = 0; k < 2; k++)
                    {
                        ka = AngleCase(k, kappa);

                        X[3, 0] = om; X[4, 0] = ph; X[5, 0] = ka;
                        //CreateAL(A, X, L, MassCenter, CPList);
                        //f = L - A.Multiply(X);

                        partialvalue = LinearizedEquation(X, xyz[0, 0] , xyz[1, 0], xyz[2, 0]);

                        A[0, 3] = partialvalue[0]; A[0, 4] = partialvalue[1]; A[0, 5] = partialvalue[2]; A[0, 6] = partialvalue[3];
                        A[1, 3] = partialvalue[4]; A[1, 4] = partialvalue[5]; A[1, 5] = partialvalue[6]; A[1, 6] = partialvalue[7];
                        A[2, 3] = partialvalue[8]; A[2, 4] = partialvalue[9]; A[2, 5] = partialvalue[10]; A[2, 6] = partialvalue[11];

                        A[0, 0] = A[1, 1] = A[2, 2] = 1;
                        A[0, 1] = A[0, 2] = A[1, 0] = A[1, 2] = A[2, 0] = A[2, 1] = 0;

                        L[0, 0] = enh[0 ,0]; L[1, 0] = enh[1, 0]; L[2, 0] = enh[2, 0];

                        f = L - A.Multiply(X);

                        Matrix<double> C = RotationMatrix(om, ph, ka);

                        double sum2 = 0;
                        for (int m = 0; m < f.RowCount; m++) sum2 += Math.Sqrt(f[m, 0] * f[m, 0]);
                        //int b = 0;
                        if (minDet > sum2)
                        {
                            minDet = sum2;
                            AccurateAngle[0] = om;
                            AccurateAngle[1] = ph;  ////////////!!!!!!!!!!!!!!!!!!!!!!!
                            AccurateAngle[2] = ka;
                        }
                    }
                }
            }

            return AccurateAngle;
        }         

        private void CreateAL(Matrix<double> A, Matrix<double> X, Matrix<double> L, double[] MassCenter, List<ControlPoint> CPList)
        {
            double[] partialvalue;
            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                partialvalue = LinearizedEquation(X, cp.CPXind, cp.CPYind, cp.CPZind);
                //partialvalue = LinearizedEquation(X, cp.CPXind - MassCenter[3], cp.CPYind - MassCenter[4], cp.CPZind - MassCenter[5]);

                A[i * 3 + 0, 3] = partialvalue[0]; A[i * 3 + 0, 4] = partialvalue[1]; A[i * 3 + 0, 5] = partialvalue[2 ]; A[i * 3 + 0, 6] = partialvalue[3];
                A[i * 3 + 1, 3] = partialvalue[4]; A[i * 3 + 1, 4] = partialvalue[5]; A[i * 3 + 1, 5] = partialvalue[6 ]; A[i * 3 + 1, 6] = partialvalue[7];
                A[i * 3 + 2, 3] = partialvalue[8]; A[i * 3 + 2, 4] = partialvalue[9]; A[i * 3 + 2, 5] = partialvalue[10]; A[i * 3 + 2, 6] = partialvalue[11];

                A[i * 3 + 0, 0] = 1; A[i * 3 + 0, 1] = 0; A[i * 3 + 0, 2] = 0;  
                A[i * 3 + 1, 0] = 0; A[i * 3 + 1, 1] = 1; A[i * 3 + 1, 2] = 0;
                A[i * 3 + 2, 0] = 0; A[i * 3 + 2, 1] = 0; A[i * 3 + 2, 2] = 1;
            }

            for (int j = 0; j < CPList.Count; j++)
            {
                ControlPoint cp = CPList[j];
                L[j * 3 + 0, 0] = cp.CPX97; // - MassCenter[0]; 
                L[j * 3 + 1, 0] = cp.CPY97; // - MassCenter[1]; 
                L[j * 3 + 2, 0] = cp.CPZ97; // - MassCenter[2];
            }
        }

        private double[] LinearizedEquation(Matrix<double> X, double x, double y, double z)
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33, XpS, XpOmega, XpPhi, XpKappa, YpS, YpOmega, YpPhi, YpKappa, ZpS, ZpOmega, ZpPhi, ZpKappa;
            double s = X[6, 0]; double omega = X[3, 0]; double phi = X[4, 0]; double kappa = X[5, 0]; double[] partialresult = new double[12];

            m11 =  cos(phi) * cos(kappa); m12 =  sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa); m13 = -cos(omega) * sin(phi) * cos(kappa) + sin(omega) * sin(kappa);
            m21 = -cos(phi) * sin(kappa); m22 = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa); m23 =  cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);
            m31 =  sin(phi);              m32 = -sin(omega) * cos(phi);                                        m33 =  cos(omega) * cos(phi);

            XpS = m11 * x + m12 * y + m13 * z;
            XpOmega = s * ((cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * y + (sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa)) * z);
            XpPhi =   s * ((-sin(phi) * cos(kappa)) * x + (sin(omega) * cos(phi) * cos(kappa)) * y + (-cos(omega) * cos(phi) * cos(kappa)) * z);
            XpKappa = s * ((-cos(phi) * sin(kappa)) * x + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * y + (cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa)) * z);

            YpS = m21 * x + m22 * y + m23 * z;
            YpOmega = s * ((-cos(omega) * sin(phi) * sin(kappa) - sin(omega) * cos(kappa)) * y + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * z);
            YpPhi =   s * ((sin(phi) * sin(kappa)) * x + (-sin(omega) * cos(phi) * sin(kappa)) * y + (cos(omega) * cos(phi) * sin(kappa)) * z);
            YpKappa = s * ((-cos(phi) * cos(kappa)) * x + (-sin(omega) * sin(phi) * cos(kappa) - cos(omega) * sin(kappa)) * y + (cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * z);

            ZpS = m31 * x + m32 * y + m33 * z;
            ZpOmega = s * ((-cos(omega) * cos(phi)) * y + (-sin(omega) * cos(phi)) * z);
            ZpPhi =   s * ((cos(phi)) * x + (sin(omega) * sin(phi)) * y + (-cos(omega) * sin(phi)) * z);
            ZpKappa = s * (0);

            partialresult[0] = XpOmega; partialresult[1] = XpPhi; partialresult[2 ] = XpKappa; partialresult[3 ] = XpS;
            partialresult[4] = YpOmega; partialresult[5] = YpPhi; partialresult[6 ] = YpKappa; partialresult[7 ] = YpS;
            partialresult[8] = ZpOmega; partialresult[9] = ZpPhi; partialresult[10] = ZpKappa; partialresult[11] = ZpS;

            return partialresult;
        }

        private double RMSEEachPoint(Matrix<double> fff)
        {
            double sum = 0;

            for (int i = 0; i < fff.RowCount; i += 3)
            {
                sum += fff[i, 0] * fff[i, 0] + fff[i + 1, 0] * fff[i + 1, 0] + fff[i + 2, 0] * fff[i + 2, 0];
            }

            return Math.Sqrt(sum / (fff.RowCount / 3));
        }        
        
        private void Createf(Matrix<double> f, Matrix<double> R, double Scale, double[] MassCenter, Matrix<double> Coordinate)
        {
            Matrix<double> XYZ_ = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Trans = Translation(Scale, R, MassCenter);

            for (int i = 0; i < CPList.Count; i++)
            {
                //Create Xind
                XYZ_[0, 0] = CPList[i].CPXind;// - MassCenter[3]; 
                XYZ_[1, 0] = CPList[i].CPYind;// - MassCenter[4]; 
                XYZ_[2, 0] = CPList[i].CPZind;// - MassCenter[5];

                //AX
                XYZ_ = Scale * R.Multiply(XYZ_) + Trans;

                //f = L - AX
                f[i * 3 + 0, 0] = CPList[i].CPX97 - XYZ_[0, 0]; //- MassCenter[0]
                f[i * 3 + 1, 0] = CPList[i].CPY97 - XYZ_[1, 0]; //- MassCenter[1] 
                f[i * 3 + 2, 0] = CPList[i].CPZ97 - XYZ_[2, 0]; //- MassCenter[2] 

                Coordinate[i * 3 + 0, 0] = XYZ_[0, 0];// + Trans[0, 0]; //MassCenter[0];
                Coordinate[i * 3 + 1, 0] = XYZ_[1, 0];// + Trans[1, 0]; //MassCenter[1]; 
                Coordinate[i * 3 + 2, 0] = XYZ_[2, 0];// + Trans[2, 0]; //MassCenter[2]; 
            }
        }

        private void CoordinateNow(Matrix<double> X, double[] MassCenter, Matrix<double> Coordinate)
        {
            double Scale = X[6, 0];
            Matrix<double> R = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);
            Matrix<double> XYZ_ = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Trans = Translation(Scale, R, MassCenter);

            for (int i = 0; i < CPList.Count; i++)
            {
                //Create Xind
                XYZ_[0, 0] = CPList[i].CPXind;// - MassCenter[3]; 
                XYZ_[1, 0] = CPList[i].CPYind;// - MassCenter[4]; 
                XYZ_[2, 0] = CPList[i].CPZind;// - MassCenter[5];

                //AX
                XYZ_ = Scale * R.Multiply(XYZ_) + Trans;

                Coordinate[i * 3 + 0, 0] = XYZ_[0, 0];// + Trans[0, 0]; //MassCenter[0];
                Coordinate[i * 3 + 1, 0] = XYZ_[1, 0];// + Trans[1, 0]; //MassCenter[1]; 
                Coordinate[i * 3 + 2, 0] = XYZ_[2, 0];// + Trans[2, 0]; //MassCenter[2]; 
            }
        }
        

        /// <summary>
        /// The following part doesn't work well.
        /// </summary>

        private void ConformalTransformation_Click(object sender, RoutedEventArgs e)
        {
            int n = CPList.Count; int count = 0; 
            double vmin = 999999;  double x_Avg = 0; double y_Avg = 0; double z_Avg = 0;
            double v = 0, v0; double rmse; double pi = Convert.ToInt32(Math.PI); 
            Matrix<double> A = DenseMatrix.Create(n * 3, 7, 0); //建立A矩陣，預設值為0
            Matrix<double> L = DenseMatrix.Create(n * 3, 1, 0); //建立L矩陣，預設值為0
            Matrix<double> V = DenseMatrix.Create(n * 3, 1, 0); //建立V矩陣，預設值為0
            Matrix<double> L0 = DenseMatrix.Create(n * 3, 1, 0); //建立L矩陣，預設值為0  
            Matrix<double> X = DenseMatrix.Create(7, 1, 0); //建立X矩陣，預設值為0 [S W P K X Y Z]^T
            Matrix<double> X_corr = DenseMatrix.Create(7, 1, 0);//建立修正數矩陣
            Matrix<double> X_final = DenseMatrix.Create(7, 1, 0);
            Matrix<double> QvvP = DenseMatrix.Create(n * 3, n * 3, 0);//建立粗差偵測QvvP方陣，預設值為0
            Matrix<double> I = DenseMatrix.CreateDiagonal(n * 3, n * 3, 1);
            LeastSquare LSA = new LeastSquare();

            //Initital Value (S, Omega, Phi, Kappa, X, Y, Z) = (4, 0.5, 0.5, 0.5, 175400, 2537300, 35);
            CoordinateOffset(x_Avg, y_Avg, z_Avg);
            X[0, 0] = InitScale(CPList); X[1, 0] = 1.1619f ; X[2, 0] = 3.1212f ; X[3, 0] = 1.5581f; X[4,0] = 0; X[5, 0] = 0; X[6, 0] = 0;
            
            //CreateAL(A, L, X, MassCenter, CPList);
            LSA.Adjustment(A, L); // Using init_X to get a X
            X = LSA.X; L0 = A.Multiply(X);
            //v0 = CorrectValue(X_corr);
            do
            {   v0 = v;                
                //CreateAL(A, L, X, MassCenter, CPList);
                LSA.Adjustment(A, L - L0);
                X_corr = LSA.X;  X = X + X_corr;
                L0 = A.Multiply(X);

                //v = CorrectValue(X_corr);
                /*V = A.Multiply(X) - L;
                sigma02 = V.Transpose().Multiply(V).Trace() / (3 * n - 7);
                QvvP = I - A.Multiply((A.Transpose().Multiply(A)).Inverse()).Multiply(A.Transpose());
                for (int i = 0; i < n * 3; i += 3) MessageBox.Show("QvvP : " + Convert.ToString(i / 3) + " " + Convert.ToString(QvvP[i, i]) + " " + Convert.ToString(QvvP[i + 1, i + 1]) + " " + Convert.ToString(QvvP[i + 2, i + 2]));
                */
                if (v < vmin)
                { 
                    X_final = X;
                    vmin = v;
                    count = 0;
                }
                else count += 1;

                if (count > 8)
                {
                    MessageBox.Show("Divergence, show X with min V instead."); break;
                }                 
            }
            while (Math.Abs((v-v0)/v)>0.0001);

            rmse = RMSE(A.Multiply(X_final) - L);

            MessageBox.Show("RMSE = " + Convert.ToString(rmse));
            MessageBox.Show(ToStr(X_final[0, 0]) + " " + ToStr(X_final[1, 0]) + " " + ToStr(X_final[2, 0]) + " " + ToStr(X_final[3, 0]) + " " + ToStr(X_final[4, 0] + x_Avg) + " " + ToStr(X_final[5, 0] + y_Avg) + " " + ToStr(X_final[6, 0] + z_Avg));

            Matrix<double> ScaledR = X_final[0, 0] * RotationMatrix(X_final[1, 0], X_final[2, 0], X_final[3, 0]);

            using (FileStream filestream = new FileStream(Path.GetDirectoryName(TXTdir) + "/" + "Transformation_Matrix.txt", FileMode.Create, FileAccess.Write))
            {
                double ToDeg = Convert.ToInt32(180f / Math.PI);

                StringBuilder sb = new StringBuilder();
                using (StreamWriter sw = new StreamWriter(filestream, Encoding.Default))
                {
                    sb.AppendLine(
                           "Scale : " + ToStr(X_final[0, 0]) 
                        + " Omega : " + ToStr(X_final[1, 0] * ToDeg) + "  Phi : " + ToStr(X_final[2, 0] * ToDeg) + "  Kappa : " + ToStr(X_final[3, 0] * ToDeg) 
                        +    "  X : " + ToStr(X_final[4, 0]) + "  Y : " + ToStr(X_final[5, 0]) + "  Z : " + ToStr(X_final[6, 0]) );

                    sb.AppendLine ( ToStr(ScaledR[0, 0]) + " " + ToStr(ScaledR[0, 1]) + " " + ToStr(ScaledR[0, 2]) + " " + ToStr(X_final[4, 0]) );
                    sb.AppendLine ( ToStr(ScaledR[1, 0]) + " " + ToStr(ScaledR[1, 1]) + " " + ToStr(ScaledR[1, 2]) + " " + ToStr(X_final[5, 0]) );
                    sb.AppendLine ( ToStr(ScaledR[2, 0]) + " " + ToStr(ScaledR[2, 1]) + " " + ToStr(ScaledR[2, 2]) + " " + ToStr(X_final[6, 0]) );
                    sb.AppendLine ( ToStr(0) + " " + ToStr(0) + " " + ToStr(0) + " " + ToStr(1) );

                    sw.Write(sb.ToString());
                    sw.Close();
                }
            }
        }        

        private double RMSE(Matrix<double> L)
        {
            double sumsquare = 0;
            for (int i = 0; i < L.RowCount; i++)
            {
                sumsquare += L[i, 0] * L[i, 0];
            }

            return Math.Pow(sumsquare / L.RowCount, 0.5);
        }              

        private void CoordinateOffset(double x_Avg, double y_Avg, double z_Avg)
        {
            double x = 0; double y = 0; double z = 0; int n = CPList.Count;

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                x += cp.CPX97; y += cp.CPY97; z += cp.CPZ97;
            }

            x_Avg = x / n; y_Avg = y / n; z_Avg = z / n;
        }

        private double InitScale(List<ControlPoint> CPList)
        {
            double L_97 = 0; double L_ind = 0;

            ControlPoint A = CPList[0]; ControlPoint B = CPList[1];

            L_97 = Math.Sqrt((A.CPX97 - B.CPX97) * (A.CPX97 - B.CPX97) + (A.CPY97 - B.CPY97) * (A.CPY97 - B.CPY97) + (A.CPZ97 - B.CPZ97) * (A.CPZ97 - B.CPZ97));
            L_ind = Math.Sqrt((A.CPXind - B.CPXind) * (A.CPXind - B.CPXind) + (A.CPYind - B.CPYind) * (A.CPYind - B.CPYind) + (A.CPZind - B.CPZind) * (A.CPZind - B.CPZind));

            return Convert.ToDouble(L_97/L_ind);
        }

        private string ToStr(double a)
        {
            return Convert.ToString(Math.Round(a,5));
        }

        //Inverse Transformation cuz the no unit problem will be encountered.

        private void Inv_SPT_Click(object sender, RoutedEventArgs e)
        {
            double[] MassCenter = MassCenterCaculation(KFComapre);
            Matrix<double> RA = DenseMatrix.Create(3, 3, 0); ;
            double Apx_Scale = 999; //double minscore = 999999999999; int PA = 0; int PB = 0; int PC = 0;

            /*
            for (int i = (int)(2 * KFComapre.Count / 3); i < KFComapre.Count; i++)
            {
                for (int j = (int)(KFComapre.Count / 3); j < (int)(2 * KFComapre.Count / 3); j++)
                {
                    if (i <= j) break;

                    for (int k = 0; k < (int)(KFComapre.Count/3); k++)
                    {
                        if (j <= k || i <= k) break;

                        Apx_Scale = ApproximateScale(KFComapre[i], KFComapre[j], KFComapre[k]);
                        double scores = InitialValueTraj(KFComapre[i], KFComapre[j], KFComapre[k], MassCenter, RA, Apx_Scale);

                        if (scores < minscore)
                        {
                            minscore = scores;
                            PA = i; PB = j; PC = k;
                        }
                    }
                }
            }*/

            ControlPoint PointA = KFComapre[10];
            ControlPoint PointB = KFComapre[60];
            ControlPoint PointC = KFComapre[120];

            //Caculate the approximate Rotation Matrix RA
            //RA = InitRotbyNorm(PointA, PointB, PointC, MassCenter).Transpose();
            double[,] Umeyama = { { -0.0301624, 0.9992328, 0.02498078 }, { -0.04414884, 0.02363594, -0.99874532 }, { -0.99856953, -0.03122742, 0.04340205 } };
            RA = DenseMatrix.OfArray(Umeyama);

            //initial Rotation
            double Phi = Math.Asin(RA[2, 0]);
            double Omega = Math.Asin(-RA[2, 1] / cos(Phi));
            double Kappa = Math.Asin(-RA[1, 0] / cos(Phi));

            //Caculate Approximate Scale
            Apx_Scale = 1 / ApproximateScale(PointA, PointB, PointC);

            Matrix<double> A = DenseMatrix.Create(3 * KFComapre.Count, 7, 0);
            Matrix<double> X = DenseMatrix.Create(7, 1, 0);
            Matrix<double> L = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> P = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> V = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> V0 = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> T = DenseMatrix.Create(3, 1, 0);
            Matrix<double> Xcorr = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Xcorr0 = DenseMatrix.Create(7, 1, 0);
            Matrix<double> Qxx = DenseMatrix.Create(7, 7, 0);
            Matrix<double> Qvv = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> QLL = DenseMatrix.CreateIdentity(3 * KFComapre.Count);
            Matrix<double> AT, VT; double sigma0 = 0, sigma0_0 = 0; 

            //First caculation by initial value
            T = Translation_Inv(Apx_Scale, RA, MassCenter);
            //X[0, 0] = T[0, 0]; X[1, 0] = T[1, 0]; X[2, 0] = T[2, 0]; X[3, 0] = Omega; X[4, 0] = Phi; X[5, 0] = Kappa; X[6, 0] = Apx_Scale;
            X[0, 0] = -185673.1095; X[1, 0] = -3830.1124; X[2, 0] = 18670.7707; X[3, 0] = Omega; X[4, 0] = Phi; X[5, 0] = Kappa; X[6, 0] = 0.0733851;

            //Create the necessary matrix for adjustment
            A = CreateA_Inv(X, KFComapre); L = L_InvSPT(KFComapre, A, X); AT = A.Transpose();
            //QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();

            //Adjustment
            Xcorr = (AT.Multiply(P).Multiply(A)).Inverse().Multiply(AT.Multiply(P).Multiply(L));
            V = A.Multiply(Xcorr) - L; VT = V.Transpose();
            Matrix<double> M = AT.Multiply(P).Multiply(V);
            sigma0 = Math.Sqrt((VT.Multiply(P).Multiply(V))[0, 0] / (V.RowCount - 7));

            for (int i = 0; i < 50; i++)
            {
                //Blunder Detection
                //Qxx = (AT.Multiply(Qwwinv).Multiply(A)).Inverse();
                //Qvv = P.Multiply(BT).Multiply(Qwwinv - Qwwinv.Multiply(A.Multiply(Qxx).Multiply(AT).Multiply(Qwwinv))).Multiply(B.Multiply(QLL));
                //MessageBox.Show(Convert.ToString(Dau_Test(V, Qvv, P, sigma0)));

                //Parameter Update
                X = X + Xcorr; Xcorr0 = Xcorr; sigma0_0 = sigma0; V0 = V;
                //P = CreateP_MTA(P, V, sigma0, Weighting_Type, count, Qvv, CPList); count += 1;

                //Create the necessary matrix for adjustment again
                A = CreateA_Inv(X, KFComapre); L = L_InvSPT(KFComapre, A, X); AT = A.Transpose();
                //QLL = P.Inverse(); Qww = B.Multiply(QLL).Multiply(BT); Qwwinv = Qww.Inverse();

                //Adjustment again
                Xcorr = (AT.Multiply(P).Multiply(A)).Inverse().Multiply(AT.Multiply(P).Multiply(L));
                V = A.Multiply(Xcorr) - L; VT = V.Transpose();
                M = AT.Multiply(P).Multiply(V);
                sigma0 = Math.Sqrt((VT.Multiply(P).Multiply(V))[0, 0] / (V.RowCount - 7));

                //Convergence Check
                if (Convergence(Xcorr, Xcorr0, 0.0001)) break;
            }

            //ResidualEachPoint(V0, Weighting_Type, KFComapre);
            //double[,] Trans = { { X[0, 0], X[1, 0], X[2, 0] } };
            //Scale7para = X[6, 0]; Rot7para = RotationMatrix(X[3, 0], X[4, 0], X[5, 0]);
            //Trans7para = DenseMatrix.OfArray(Trans);
        }

        private Matrix<double> CreateA_Inv(Matrix<double> X0, List<ControlPoint> CPList)
        {
            Matrix<double> A = DenseMatrix.Create(3 * KFComapre.Count, 7, 0);

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];
                double[] partialvalue = LinearizedEquationInv(X0, cp.CPX97, cp.CPY97, cp.CPZ97);

                A[i * 3 + 0, 3] = partialvalue[0]; A[i * 3 + 0, 4] = partialvalue[1]; A[i * 3 + 0, 5] = partialvalue[2]; A[i * 3 + 0, 6] = partialvalue[3];
                A[i * 3 + 1, 3] = partialvalue[4]; A[i * 3 + 1, 4] = partialvalue[5]; A[i * 3 + 1, 5] = partialvalue[6]; A[i * 3 + 1, 6] = partialvalue[7];
                A[i * 3 + 2, 3] = partialvalue[8]; A[i * 3 + 2, 4] = partialvalue[9]; A[i * 3 + 2, 5] = partialvalue[10]; A[i * 3 + 2, 6] = partialvalue[11];

                A[i * 3 + 0, 0] = 1; A[i * 3 + 0, 1] = 0; A[i * 3 + 0, 2] = 0;
                A[i * 3 + 1, 0] = 0; A[i * 3 + 1, 1] = 1; A[i * 3 + 1, 2] = 0;
                A[i * 3 + 2, 0] = 0; A[i * 3 + 2, 1] = 0; A[i * 3 + 2, 2] = 1;
            }

            return A;
        }

        private double[] LinearizedEquationInv(Matrix<double> X0, double x, double y, double z)
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33, XpS, XpOmega, XpPhi, XpKappa, YpS, YpOmega, YpPhi, YpKappa, ZpS, ZpOmega, ZpPhi, ZpKappa;
            double s = X0[6, 0]; double omega = X0[3, 0]; double phi = X0[4, 0]; double kappa = X0[5, 0]; double[] partialresult = new double[12];

            m11 = cos(phi) * cos(kappa); m12 = sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa); m13 = -cos(omega) * sin(phi) * cos(kappa) + sin(omega) * sin(kappa);
            m21 = -cos(phi) * sin(kappa); m22 = -sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa); m23 = cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa);
            m31 = sin(phi); m32 = -sin(omega) * cos(phi); m33 = cos(omega) * cos(phi);

            XpS = m11 * x + m12 * y + m13 * z;
            XpOmega = s * ((cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * y + (sin(omega) * sin(phi) * cos(kappa) + cos(omega) * sin(kappa)) * z);
            XpPhi = s * ((-sin(phi) * cos(kappa)) * x + (sin(omega) * cos(phi) * cos(kappa)) * y + (-cos(omega) * cos(phi) * cos(kappa)) * z);
            XpKappa = s * ((-cos(phi) * sin(kappa)) * x + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * y + (cos(omega) * sin(phi) * sin(kappa) + sin(omega) * cos(kappa)) * z);

            YpS = m21 * x + m22 * y + m23 * z;
            YpOmega = s * ((-cos(omega) * sin(phi) * sin(kappa) - sin(omega) * cos(kappa)) * y + (-sin(omega) * sin(phi) * sin(kappa) + cos(omega) * cos(kappa)) * z);
            YpPhi = s * ((sin(phi) * sin(kappa)) * x + (-sin(omega) * cos(phi) * sin(kappa)) * y + (cos(omega) * cos(phi) * sin(kappa)) * z);
            YpKappa = s * ((-cos(phi) * cos(kappa)) * x + (-sin(omega) * sin(phi) * cos(kappa) - cos(omega) * sin(kappa)) * y + (cos(omega) * sin(phi) * cos(kappa) - sin(omega) * sin(kappa)) * z);

            ZpS = m31 * x + m32 * y + m33 * z;
            ZpOmega = s * ((-cos(omega) * cos(phi)) * y + (-sin(omega) * cos(phi)) * z);
            ZpPhi = s * ((cos(phi)) * x + (sin(omega) * sin(phi)) * y + (-cos(omega) * sin(phi)) * z);
            ZpKappa = s * (0);

            partialresult[0] = XpOmega; partialresult[1] = XpPhi; partialresult[2] = XpKappa; partialresult[3] = XpS;
            partialresult[4] = YpOmega; partialresult[5] = YpPhi; partialresult[6] = YpKappa; partialresult[7] = YpS;
            partialresult[8] = ZpOmega; partialresult[9] = ZpPhi; partialresult[10] = ZpKappa; partialresult[11] = ZpS;

            return partialresult;
        }

        private Matrix<double> L_InvSPT(List<ControlPoint> CPList, Matrix<double> A, Matrix<Double> X0)
        {
            Matrix<double> L = DenseMatrix.Create(3 * KFComapre.Count, 1, 0);
            Matrix<double> AX = A.Multiply(X0);

            for (int i = 0; i < CPList.Count; i++)
            {
                ControlPoint cp = CPList[i];

                L[3 * i + 0, 0] = cp.CPXind - AX[3 * i + 0, 0];
                L[3 * i + 1, 0] = cp.CPYind - AX[3 * i + 1, 0];
                L[3 * i + 2, 0] = cp.CPZind - AX[3 * i + 2, 0];
            }

            return L;
        }

        private Matrix<double> Translation_Inv(double Scale, Matrix<double> Rot, double[] MassCenter)
        {
            Matrix<double> TENH = DenseMatrix.Create(3, 1, 0);
            Matrix<double> TXYZ = DenseMatrix.Create(3, 1, 0);

            TXYZ[0, 0] = MassCenter[3]; TXYZ[1, 0] = MassCenter[4]; TXYZ[2, 0] = MassCenter[5];
            TENH[0, 0] = MassCenter[0]; TENH[1, 0] = MassCenter[1]; TENH[2, 0] = MassCenter[2];

            TXYZ = TXYZ - Scale * Rot.Multiply(TENH);

            return TXYZ;
        }

    }
}