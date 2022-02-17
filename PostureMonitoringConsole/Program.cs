using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

using Intel.RealSense;

using Cubemos.SkeletonTracking;
using static Cubemos.SkeletonTracking.Api;
using System.Diagnostics;

namespace PoseRecognition
{
    class Program
    {
        readonly (double, double) DefinedXRange = (3.20, 3.60);
        readonly (double, double) DefinedZRange = (-0.5, 0.5);

        readonly string CubemosFolder = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\Cubemos\\SkeletonTracking";
        readonly int FrameWidth = 1280;
        readonly int FrameHeight = 720;

        private readonly ReaderWriterLockSlim lock_ = new ReaderWriterLockSlim();

        public static void Main()
        {
            Program p = new Program();
            //p.AnglesCalculationTest();
            //p.ReadFromFile();
            //return;
            bool success = p.SetupAndFindCameras();
            if (!success)
            {
                Console.ReadLine();
                Environment.Exit(1);
            }
            else
            {
                while (true)
                {
                    if (Console.ReadKey(true).Key == ConsoleKey.Escape)
                        Environment.Exit(0);
                }
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

        private bool SetupAndFindCameras()
        {
            Context Context = new Context();

            // Print header to output file
            File.AppendAllText("mixed_output.csv", "Camera Name,Camera Serial,Body Part,Milliseconds,X,Y,Z\n");

            // Choose and initialise camera
            DeviceList availableDevices;

            while (true)
            {
                availableDevices = Context.QueryDevices(include_platform_camera: false);
                if (availableDevices.Count > 0)
                    break;
                Console.WriteLine("NO DEVICE DETECTED! Press Enter to recheck!");
                Console.ReadKey();
            }

            // Create a list with camera info and apis
            List<(Device, Api)> devices = new List<(Device, Api)>();
            foreach (Device camera in availableDevices)
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

                devices.Add((camera, skeletontrackingApi));
            }

            // Start one thread for each camera available
            foreach (var device in devices)
            {
                Task.Factory.StartNew(() => ReadRealtime(device.Item1, device.Item2));
                Console.WriteLine("Camera \"" + device.Item1.Info[CameraInfo.Name] + "\" with SN \"" + device.Item1.Info[CameraInfo.SerialNumber] + "\" detected");
            }
            return true;
            //if (availableDevices.Count == 1)
            //{
            //    cameraSources.Add((availableDevices[0].Info[CameraInfo.Name], availableDevices[0].Info[CameraInfo.SerialNumber]));
            //    Console.WriteLine("Single camera \"" + CameraInfo.Name + "\" with SN \"" + CameraInfo.SerialNumber + "\" detected");
            //}
            //else
            //{
            //    Console.WriteLine("Write number to choose camera: ");
            //    for (int i = 0; i < availableDevices.Count; i++)
            //    {
            //        Console.WriteLine(i+1 + " " + availableDevices[i].Info[CameraInfo.Name] + " " + availableDevices[i].Info[CameraInfo.SerialNumber] + "; ");
            //        cameraSources.Add((availableDevices[i].Info[CameraInfo.Name], availableDevices[i].Info[CameraInfo.SerialNumber]));
            //    }
            //    chosenCamera = int.Parse(Console.ReadKey().KeyChar.ToString()) - 1;
            //    Console.WriteLine();
            //}
        }

        public void ReadRealtime(Device camera, Api skeletontrackingApi)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            // Save camera info
            string cameraInfo = camera.Info[CameraInfo.Name] + "," + camera.Info[CameraInfo.SerialNumber];

            Config Config = new Config();

            // Create and config the pipeline to stream color and depth frames
            Config.EnableDevice(camera.Info[CameraInfo.SerialNumber]);
            Config.EnableStream(Intel.RealSense.Stream.Color, FrameWidth, FrameHeight, Format.Bgr8, framerate: 30);
            Config.EnableStream(Intel.RealSense.Stream.Depth, FrameWidth, FrameHeight, framerate: 30);

            // create the alignment object to the color stream
            Align align = new Align(Intel.RealSense.Stream.Color);

            // Initialise the intel realsense pipeline as an acquisition device
            Pipeline Pipeline = new Pipeline();

            // Create and config the pipeline to stream color and depth frames
            PipelineProfile pp = Pipeline.Start(Config);
            Intrinsics intrinsicsDepthImagerMaster = pp.GetStream(Intel.RealSense.Stream.Depth).As<VideoStreamProfile>().GetIntrinsics();

            // Set the network input size to 128 for faster inference
            int networkHeight = 128;

            // Create output file headers
            File.AppendAllText(camera.Info[CameraInfo.SerialNumber] + ".csv", "Body Part,Milliseconds,X,Y,Z\n");

            //Save frames as images in temp folder
            Directory.CreateDirectory(camera.Info[CameraInfo.SerialNumber] + "/");

            Console.WriteLine("Starting image acquisition and skeleton keypoints for \"" + camera.Info[CameraInfo.Name] + "\" with SN \"" + camera.Info[CameraInfo.SerialNumber] + "\"");
            while (true)
            {
                // We wait for the next available FrameSet and using it as a releaser object that would
                // track all newly allocated .NET frames, and ensure deterministic finalization at the end
                // of scope.
                using (FramesReleaser releaser = new FramesReleaser())
                {
                    using (FrameSet frames = Pipeline.WaitForFrames())
                    {
                        FrameSet frameSet = frames.ApplyFilter(align).DisposeWith(releaser).AsFrameSet().DisposeWith(releaser);

                        // Create DepthFrame to manage third dimension
                        DepthFrame depthFrame = frameSet.DepthFrame.DisposeWith(releaser);

                        // DepthFrame colorisation
                        VideoFrame colorFrame = frameSet.ColorFrame.DisposeWith(releaser);

                        // Preprocess the input image
                        Bitmap inputImage = Utils.FrameToBitmap(colorFrame);

                        // Save timestamp
                        long timestamp = stopwatch.ElapsedMilliseconds;

                        //Save image to temp folder
                        inputImage.Save(camera.Info[CameraInfo.SerialNumber] + "/" + timestamp + ".jpg");

                        // Get 2D joints
                        skeletontrackingApi.RunSkeletonTracking(ref inputImage, networkHeight, out List<SkeletonKeypoints> skeletons, 0);

                        Console.WriteLine(skeletons.Count + " skeletons detected");

                        //KeyValuePair<string, Point3D?>? isOperativeInWorkingArea = null;

                        foreach (SkeletonKeypoints skeleton in skeletons)
                        {
                            // Calculate 3D joints for entire skeleton
                            Dictionary<string, Point3D?> skeleton3D = Utils.Get3DCoordinates(skeleton, depthFrame, intrinsicsDepthImagerMaster);

                            // Print joints
                            foreach (var joint in skeleton3D)
                            {
                                // Write to file and send
                                string output;
                                string info = cameraInfo + ",";
                                // Switch left and right ears as they appear to be wrong
                                switch (joint.Key)
                                {
                                    case "Left Ear":
                                        if (joint.Value == null)
                                            output = "Right Ear," + timestamp + ",,,\n";
                                        else
                                            output = "Right Ear," + timestamp + "," + joint.Value.ToString() + "\n";
                                        break;
                                    case "Right Ear":
                                        if (joint.Value == null)
                                            output = "Left Ear," + timestamp + ",,,\n";
                                        else
                                            output = "Left Ear," + timestamp + "," + joint.Value.ToString() + "\n";
                                        break;
                                    default:
                                        if (joint.Value == null)
                                            output = joint.Key + "," + timestamp + ",,,\n";
                                        else
                                            output = joint.Key + "," + timestamp + "," + joint.Value.ToString() + "\n";
                                        break;
                                }
                                File.AppendAllText(camera.Info[CameraInfo.SerialNumber] + ".csv", output);
                                lock_.EnterWriteLock();
                                File.AppendAllText("mixed_output.csv", info + output);
                                lock_.ExitWriteLock();
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
