using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Windows.Media.Media3D;

using Intel.RealSense;

using Cubemos.SkeletonTracking;
using static Cubemos.SkeletonTracking.Api;

namespace PoseRecognition
{
    class Program
    {
        readonly (double, double) DefinedXRange = (3.20, 3.60);
        readonly (double, double) DefinedZRange = (-0.5, 0.5);

        readonly string CubemosFolder = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\Cubemos\\SkeletonTracking";
        readonly int FrameWidth = 1280;
        readonly int FrameHeight = 720;

        Pipeline Pipeline;
        PipelineProfile pp;
        Config Config;
        Context Context;

        public static void Main()
        {
            Program p = new Program();
            //p.AnglesCalculationTest();
            //p.ReadFromFile();
            //return;
            bool success = p.ReadRealtime();
            if (!success)
            {
                Console.ReadLine();
                Environment.Exit(1);
            }
        }

        //TODELETE
        public void AnglesCalculationTest()
        {
            Point3D a = new Point3D { X = 5, Y = 2.82, Z = 11 };
            Point3D b = new Point3D { X = 5, Y = 0, Z = 5 };
            Point3D c = new Point3D { X = 10, Y = 8.45, Z = 8 };
            var threedim = Angles.Calculate3DAngle(a, b, c);
            var frontal = Angles.Calculate2DFrontalAngle(a, b, c);
            var lateral = Angles.Calculate2DLateralAngle(a, b, c);
            var overhead = Angles.Calculate2DOverheadAngle(a, b, c);
            Console.WriteLine("3Dimens: " + threedim);
            Console.WriteLine("Frontal: " + frontal);
            Console.WriteLine("Lateral: " + lateral);
            Console.WriteLine("Overhead: " + overhead);
            Console.ReadKey();
        }

        private bool ReadRealtime()
        {
            // Initialize logging to output all messages with severity level INFO or higher to the console
            //Cubemos.Api.InitialiseLogging(Cubemos.LogLevel.CM_LL_ERROR, bWriteToConsole: true);
            Api skeletontrackingApi;

            // Create cubemos Skeleton tracking Api handle and specify the directory containing a cubemos_license.json file
            try
            {
                skeletontrackingApi = new Api(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\Cubemos\\SkeletonTracking\\license");
            }
            catch (Exception)
            {
                Console.WriteLine("SDK is not activated.");
                return false;
            }

            // Initialise cubemos DNN framework with the required model
            try
            {
                skeletontrackingApi.LoadModel(Cubemos.TargetComputeDevice.CM_CPU, CubemosFolder + "\\models\\fp32\\skeleton-tracking.cubemos");
            }
            catch (Exception)
            {
                Console.WriteLine("Model not found");
                return false;
            }

            // Initialise the intel realsense pipeline as an acquisition device
            Pipeline = new Pipeline();
            Config = new Config();
            Context = new Context();

            // create the alignment object to the color stream
            Align align = new Align(Intel.RealSense.Stream.Color);

            // Choose and initialise camera
            List<(string, string)> CameraSources = new List<(string, string)>();
            DeviceList availableDevices;
            int ChosenCamera = 0;

            while (true)
            {
                availableDevices = Context.QueryDevices(include_platform_camera: false);
                if (availableDevices.Count > 0)
                    break;
                Console.WriteLine("NO DEVICE DETECTED! Press Enter to recheck!");
                Console.ReadKey();
            }
            if (availableDevices.Count == 1)
                CameraSources.Add((availableDevices[0].Info[CameraInfo.Name], availableDevices[0].Info[CameraInfo.SerialNumber]));
            else
            {
                Console.Write("Write number to choose camera: ");
                for (int i = 0; i < availableDevices.Count; i++)
                {
                    Console.Write(i+1 + " " + availableDevices[i].Info[CameraInfo.Name] + " " + availableDevices[i].Info[CameraInfo.SerialNumber] + "; ");
                    CameraSources.Add((availableDevices[i].Info[CameraInfo.Name], availableDevices[i].Info[CameraInfo.SerialNumber]));
                }
                ChosenCamera = int.Parse(Console.ReadKey().KeyChar.ToString()) - 1;
                Console.WriteLine();
            }

            // Save camera info
            string cameraInfo = availableDevices[ChosenCamera].Info[CameraInfo.Name] + "," + availableDevices[ChosenCamera].Info[CameraInfo.SerialNumber];

            // Create and config the pipeline to stream color and depth frames.
            Config.EnableDevice(CameraSources[ChosenCamera].Item2);
            Config.EnableStream(Intel.RealSense.Stream.Color, FrameWidth, FrameHeight, Format.Bgr8, framerate: 30);
            Config.EnableStream(Intel.RealSense.Stream.Depth, FrameWidth, FrameHeight, framerate: 30);

            // Create and config the pipeline to stream color and depth frames.
            pp = Pipeline.Start(Config);
            Intrinsics intrinsicsDepthImagerMaster = pp.GetStream(Intel.RealSense.Stream.Depth).As<VideoStreamProfile>().GetIntrinsics();

            // Set the network input size to 128 for faster inference
            int networkHeight = 128;

            // Udp Socket for data exchange
            UdpClient udpClient = new UdpClient();
            udpClient.Connect("localhost", 65000);

            // Create output file headers
            File.AppendAllText("output.csv", "Camera Name,Camera Serial,Body Part,Timestamp,X,Y,Z\n");

            Console.WriteLine("Starting image acquisition and skeleton keypoints");
            while (true)
            {
                int pipelineID = 0;

                // We wait for the next available FrameSet and using it as a releaser object that would
                // track all newly allocated .NET frames, and ensure deterministic finalization at the end
                // of scope.
                using (var releaser = new FramesReleaser())
                {
                    using (var frames = Pipeline.WaitForFrames())
                    {
                        FrameSet frameSet = frames.ApplyFilter(align).DisposeWith(releaser).AsFrameSet().DisposeWith(releaser);

                        // Create DepthFrame to manage third dimension
                        DepthFrame depthFrame = frameSet.DepthFrame.DisposeWith(releaser);

                        // DepthFrame colorisation
                        VideoFrame colorFrame = frameSet.ColorFrame.DisposeWith(releaser);

                        // Preprocess the input image
                        Bitmap inputImage = Utils.FrameToBitmap(colorFrame);

                        // Get 2D joints
                        skeletontrackingApi.RunSkeletonTracking(ref inputImage, networkHeight, out List<SkeletonKeypoints> skeletons, pipelineID);

                        Console.WriteLine(skeletons.Count + " skeletons detected");

                        //KeyValuePair<string, Point3D?>? isOperativeInWorkingArea = null;

                        foreach (var skeleton in skeletons)
                        {
                            // Calculate 3D joints for entire skeleton
                            Dictionary<string, Point3D?> skeleton3D = Utils.Get3DCoordinates(skeleton, depthFrame, intrinsicsDepthImagerMaster);

                            // Save timestamp
                            string unixTimestamp = Convert.ToString((long)DateTime.Now.Subtract(new DateTime(1970, 1, 1)).TotalMilliseconds);

                            // Print joints
                            foreach (var joint in skeleton3D)
                            {
                                // Write to file and send
                                string output;
                                if (joint.Value == null)
                                    output = cameraInfo + "," + joint.Key + "," + unixTimestamp + ",,,\n";
                                else
                                    output = cameraInfo + "," + joint.Key + "," + unixTimestamp + "," + joint.Value.ToString() + "\n";
                                udpClient.Send(Encoding.UTF32.GetBytes(output), Encoding.UTF32.GetByteCount(output));

                                File.AppendAllText("output.csv", output);
                            }

                            // Body angles calculations
                            //double? elbow = Angles.Calculate3DAngle(skeleton3D["Right Shoulder"], skeleton3D["Right Elbow"], skeleton3D["Right Wrist"]);
                            //if (elbow == null)
                            //    Console.WriteLine("Elbow angle is not calculable");
                            //else
                            //    Console.WriteLine("Elbow angle is " + elbow);

                            // Working area checking
                            //isOperativeInWorkingArea = IsOperativeInWorkingArea(skeleton3D);
                            //if (isOperativeInWorkingArea != null)
                            //    break;
                        }

                        //if (isOperativeInWorkingArea != null)
                        //    Console.WriteLine(string.Format("{0} HAS BEEN DETECTED IN WORKING AREA AT {1}, {2}, {3}",
                        //                                    isOperativeInWorkingArea.Value.Key,
                        //                                    isOperativeInWorkingArea.Value.Value.Value.X,
                        //                                    isOperativeInWorkingArea.Value.Value.Value.Y,
                        //                                    isOperativeInWorkingArea.Value.Value.Value.Z));
                        //else
                        //    Console.WriteLine("WORKING AREA IS CLEAR");
                    }
                }
            }
        }

        // Checks if the operative is inside the working area (only X and Z coordinates are required)
        public KeyValuePair<string, Point3D?>? IsOperativeInWorkingArea(Dictionary<string, Point3D?> skeleton3D)
        {
            foreach (var joint in skeleton3D)
                if (joint.Value != null)
                    if ((joint.Value.Value.X > DefinedXRange.Item1 && joint.Value.Value.X < DefinedXRange.Item2) || (joint.Value.Value.Z > DefinedZRange.Item1 && joint.Value.Value.Z < DefinedZRange.Item2))
                        return joint;
            return null;
        }

        //TODELETE
        public void ReadFromFile()
        {
            // Read a text file line by line.
            string[] lines = File.ReadAllLines("../../../Prova1.txt");

            int record = 0;
            while (record < lines.Length)
            {
                Dictionary<string, Point3D?> skeleton3D = new Dictionary<string, Point3D?>();
                while (lines[record].StartsWith("Joint"))
                {
                    string[] values = lines[record].Split(new char[0], StringSplitOptions.RemoveEmptyEntries);
                    if (values[3] != "unavailable")
                    {
                        Point3D point = new Point3D
                        {
                            X = double.Parse(values[3].TrimStart('(').TrimEnd(',').Replace(',', '.')),
                            Y = double.Parse(values[4].TrimEnd(',').Replace(',', '.')),
                            Z = double.Parse(values[5].TrimEnd(')').Replace(',', '.'))
                        };

                        skeleton3D.Add(values[2].TrimEnd(':'), point);
                    }
                    record++;
                }

                if (skeleton3D.Count != 0)
                {
                    KeyValuePair<string, Point3D?>? isOperativeInWorkingArea = IsOperativeInWorkingArea(skeleton3D);
                    if (isOperativeInWorkingArea != null)
                        Console.WriteLine(string.Format("{0} HAS BEEN DETECTED IN WORKING AREA AT {1}, {2}, {3}", isOperativeInWorkingArea.Value.Key, isOperativeInWorkingArea.Value.Value.Value.X, isOperativeInWorkingArea.Value.Value.Value.Y, isOperativeInWorkingArea.Value.Value.Value.Z));
                    else
                        Console.WriteLine("WORKING AREA IS CLEAR");
                    Console.ReadKey();
                }
                record++;
            }
        }
    }
}
