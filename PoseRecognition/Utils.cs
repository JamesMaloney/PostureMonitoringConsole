using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Media.Media3D;

using Intel.RealSense;

using static Cubemos.SkeletonTracking.Api;

namespace PoseRecognition
{
    class Utils
    {
        readonly static string[] BodyParts =    {   "Nose", "Spine",
                                                    "Right Shoulder", "Right Elbow", "Right Wrist",
                                                    "Left Shoulder", "Left Elbow", "Left Wrist",
                                                    "Right Hip", "Right Knee", "Right Ankle",
                                                    "Left Hip", "Left Knee", "Left Ankle",
                                                    "Right Eye", "Left Eye",
                                                    "Left Ear", "Right Ear"  };

        public static Dictionary<string, Point3D?> Get3DCoordinates(SkeletonKeypoints skeleton, DepthFrame depthFrame, Intrinsics intrinsicsDepthImagerMaster)
        {
            Dictionary<string, Point3D?> skeleton3D = new Dictionary<string, Point3D?>();

            for (int i = 0; i < skeleton.listJoints.Count; i++)
            {
                string jointName = BodyParts[i];
                if (skeleton.listJoints[i].x <= 0 || skeleton.listJoints[i].y <= 0)
                {
                    skeleton3D.Add(jointName, null);
                    continue;
                }

                float[,] depthValues = GetDepthInKernel(depthFrame, (int)skeleton.listJoints[i].x, (int)skeleton.listJoints[i].y, nKernelSize: 5);
                float averageDepth = AverageValidDepthFromNeighbourhood(depthValues);

                if (averageDepth <= 0)
                {
                    skeleton3D.Add(jointName, null);
                    continue;
                }

                Point3D worldCoordinates = WorldCoordinate(averageDepth, (int)skeleton.listJoints[i].x, (int)skeleton.listJoints[i].y, intrinsicsDepthImagerMaster);

                // Change y value sign
                worldCoordinates.Y = -worldCoordinates.Y;

                skeleton3D.Add(jointName, worldCoordinates);
            }

            return skeleton3D;
        }

        // Convert the realsense frame to C# bitmap
        public static Bitmap FrameToBitmap(VideoFrame frame, PixelFormat format = PixelFormat.Format24bppRgb)
        {
            if (frame.Width == 0)
                return null;
            return new Bitmap(frame.Width, frame.Height, frame.Stride, format, frame.Data);
        }

        /// \brief Returns average depth value of a square region in the depth map only taking into account valid depth values
        /// \param depthKernel [in] square region of depth values
        public static float AverageValidDepthFromNeighbourhood(float[,] depthKernel)
        {
            float average = 0;
            int nValidDepths = 0;
            for (int row = 0; row < depthKernel.GetLength(0); row++)
            {
                for (int col = 0; col < depthKernel.GetLength(1); col++)
                {
                    float depth = depthKernel[row, col];
                    if (depth <= 0.0001)
                        continue;

                    average += depth;
                    nValidDepths++;
                }
            }

            if (nValidDepths == 0)
                return 0.0f;
            else
                return average / nValidDepths;
        }

        /// \brief Returns depth values for a square region with the side of the nKernelSize centered on the specified coordinate
        /// \param depthFrame [in] depth map containing float values of distances in mm for every image pixel
        /// \param nCol [in] x coordinate of the region center
        /// \param nRow [in] y coordinate of the region center
        /// \param nKernelSize [in] side length of the region, e.g. nKernelSize = 3 gives a square region 3x3 
        public static float[,] GetDepthInKernel(DepthFrame depthFrame, int nCol, int nRow, int nKernelSize)
        {
            if (nCol >= depthFrame.Width || nRow >= depthFrame.Height || nCol < 0 || nRow < 0)
                throw new IndexOutOfRangeException(string.Format("Requested coordinages x: {0}, y: {1} out of CM_Image Range: {2}*{3}", nCol, nRow, depthFrame.Width, depthFrame.Height));

            uint kernelSizeHalf = (uint)(nKernelSize / 2);

            uint unStartCol = Math.Max(0, (uint)(nCol - kernelSizeHalf));
            uint unEndCol = Math.Min((uint)depthFrame.Width - 1, (uint)(nCol + kernelSizeHalf));

            uint unStartRow = Math.Max(0, (uint)(nRow - kernelSizeHalf));
            uint unEndRow = Math.Min((uint)depthFrame.Height - 1, (uint)(nRow + kernelSizeHalf));


            float[,] depthNeigbourhood = new float[unEndRow - unStartRow + 1, unEndCol - unStartCol + 1];
            for (uint i = unStartCol; i <= unEndCol; i++)
            {
                for (uint j = unStartRow; j <= unEndRow; j++)
                {
                    float depth = depthFrame.GetDistance((int)i, (int)j);

                    depthNeigbourhood[j - unStartRow, i - unStartCol] = depth;
                }
            }

            return depthNeigbourhood;
        }

        /// \brief Calculates 3D world coordinates relative to the camera for a point in image coordinates (2D) and camera intrinsics
        /// \param depth [in] distance from the camera center for this image point
        /// \param pixelX [in] x image coordinate (column)
        /// \param pixelY [in] y image coordinate (row)
        /// \param focalLengthX [in] focal length
        /// \param focalLengthY [in] focal length
        /// \param centerX [in] x coordinate of the camera center
        /// \param centerY [in] y coordinate of the camera center
        public static Point3D WorldCoordinate(float depth, int pixelX, int pixelY, Intrinsics intrinsicsDepthImagerMaster)
        {
            // calculate x and y
            // x = (u - ppx) * z / f;
            // *Round has been added and can be removed*

            double X = Math.Round((pixelX - intrinsicsDepthImagerMaster.ppx) * depth / intrinsicsDepthImagerMaster.fx, 3);
            double Y = Math.Round((pixelY - intrinsicsDepthImagerMaster.ppy) * depth / intrinsicsDepthImagerMaster.fy, 3);
            double Z = Math.Round(depth, 3);

            // *Y is inverted for convenience*
            return new Point3D(X, -Y, Z);
        }
    }
}
