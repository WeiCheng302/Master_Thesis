using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using LOC.Photogrammetry;

namespace SLAM
{
    public class KeyFrame
    {
        private int kfid, frameid;
        private double xind, yind, zind, x97, y97, z97;
        private double[,] rot;
        private RotationMatrix rotm;
        private ExOrietation eo;

        public int KFID { get => kfid; set => kfid = value; }
        public int FrameID { get => frameid; set => frameid = value; }
        public double Xind { get => xind; set => xind = value; }
        public double Yind { get => yind; set => yind = value; }
        public double Zind { get => zind; set => zind = value; }
        public double X97 { get => x97; set => x97 = value; }
        public double Y97 { get => y97; set => y97 = value; }
        public double Z97 { get => z97; set => z97 = value; }
        public double[,] Rot { get => rot; set => rot = value; }
        public RotationMatrix RotM { get => rotm; set => rotm = value; }
        public ExOrietation EO { get => eo; set => eo = value; }

        public KeyFrame()
        {

        }

        public KeyFrame(int KFID, double Xind, double Yind, double Zind, double[,] Rot, int FrameID)
        {
            this.KFID = KFID;
            this.FrameID = FrameID;
            this.Xind = Xind;
            this.Yind = Yind;
            this.Zind = Zind;
            this.Rot = Rot;
        }

        public KeyFrame(int KFID, double Xind, double Yind, double Zind, RotationMatrix RotM, int FrameID)
        {
            this.KFID = KFID;
            this.FrameID = FrameID;
            this.Xind = Xind;
            this.Yind = Yind;
            this.Zind = Zind;
            this.RotM = RotM;
        }

        public KeyFrame(int KFID, ExOrietation EO, int FrameID)
        {
            this.KFID = KFID;
            this.FrameID = FrameID;
            this.EO = EO;
        }

        public KeyFrame(int KFID, int FrameID, double X, double Y, double Z, bool Absolute)
        {
            this.KFID = KFID;
            this.FrameID = FrameID;

            if (Absolute == true)
            {
                this.X97 = X;
                this.Y97 = Y;
                this.Z97 = Z;
            }
            else
            {
                this.Xind = X;
                this.Yind = Y;
                this.Zind = Z;
            }
        }
    }

    public class KeyPoint
    {
        private double kfx, kfy;
        private int fid;
        private ImagePoints.Coordinate imgxy;

        public int FID { get => fid; set => fid = value; }
        public double KFX { get => kfx; set => kfx = value; }
        public double KFY { get => kfy; set => kfy = value; }
        public ImagePoints.Coordinate IMGXY { get => imgxy; set => imgxy = value; }

        public KeyPoint()
        { 
        
        }
        
        /*public KeyPoint(int kfid, Double kfx, Double kfy)
        {
            this.KFID = kfid;
            this.KFX = kfx;
            this.KFY = kfy;
        }*/

        public KeyPoint(int fid, double KFx, double KFy)
        {
            this.FID = fid;
            this.IMGXY = new ImagePoints.Coordinate(KFx, KFy);
            //this.IMGXY.X = KFx;
            //this.IMGXY.Y = KFy;
        }

    }

    public class ControlPoint
    {
        private string cpid;
        private int cpcount;
        private double cpx97, cpy97, cpz97, cpxind, cpyind, cpzind;
        private List<KeyPoint> keypointlist;

        public string CPID { get => cpid; set => cpid = value;}
        public int CPCOUNT { get => cpcount; set => cpcount = value; }
        public double CPX97 { get => cpx97; set => cpx97 = value; }
        public double CPY97 { get => cpy97; set => cpy97 = value; }
        public double CPZ97 { get => cpz97; set => cpz97 = value; }
        public double CPXind { get => cpxind; set => cpxind = value; }
        public double CPYind { get => cpyind; set => cpyind = value; }
        public double CPZind { get => cpzind; set => cpzind = value; }
        public List<KeyPoint> KeyPointList { get => keypointlist; set => keypointlist = value; }
        

        public ControlPoint()
        { 
        
        }
        /*
        public ControlPoint(string cpid, double cpx97, double cpy97, double cpz97)
        {
            this.CPID = cpid;
            this.CPX97 = Convert.ToSingle(cpx97);
            this.CPY97 = Convert.ToSingle(cpy97);
            this.CPZ97 = Convert.ToSingle(cpz97);
        }
        */
        public ControlPoint(string cpid, double cpx97, double cpy97, double cpz97)
        {
            this.CPID = cpid;
            this.CPX97 = cpx97;
            this.CPY97 = cpy97;
            this.CPZ97 = cpz97;
        }

        public ControlPoint(string cpid, int cpcount, double cpx97, double cpy97, double cpz97)
        {
            this.CPID = cpid;
            this.CPCOUNT = cpcount;
            this.CPX97 = cpx97;
            this.CPY97 = cpy97;
            this.CPZ97 = cpz97;
        }

        public ControlPoint(string cpid, double cpx97, double cpy97, double cpz97, List<KeyPoint> keypointlist)
        {
            this.CPID = cpid;
            this.CPX97 = cpx97;
            this.CPY97 = cpy97;
            this.CPZ97 = cpz97;
            this.KeyPointList = keypointlist;
        }

        public ControlPoint(int KFID, int cpcount, double X, double Y, double Z, bool Absolute)
        {
            this.CPID = Convert.ToString(KFID);
            this.CPCOUNT = cpcount;

            if (Absolute == true)
            {
                this.CPX97 = X;
                this.CPY97 = Y;
                this.CPZ97 = Z;
            }
            else
            {
                this.CPXind = X;
                this.CPYind = Y;
                this.CPZind = Z;
            }
        }
    }
    
}
