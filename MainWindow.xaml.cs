//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyIndexBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Runtime.InteropServices;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections;
    using System.Collections.Generic;
    /// <summary>
    /// Interaction logic for the MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private const int BytesPerPixel = 4;

        /// <summary>
        /// Collection of colors to be used to display the BodyIndexFrame data.
        /// </summary>
        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x00FF0000,
            0xFFFF4000,
            0x40FFFF00,
            0xFF40FF00,
            0xFF808000,
        };

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;
        private StreamWriter writetext = new StreamWriter("write.txt");
        /// <summary>
        /// Reader for body index frames
        /// </summary>
        private BodyIndexFrameReader bodyIndexFrameReader = null;
        private MultiSourceFrameReader depthAndBodyIndexReader = null;
        /// <summary>
        /// Description of the data contained in the body index frame
        /// </summary>
        private FrameDescription bodyIndexFrameDescription = null;
        private FrameDescription depthFrameDescription;
        private Body[] bodies = null;
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap bodyIndexBitmap = null;
        private WriteableBitmap depthBitmap;
        string dataSource;
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private uint[] bodyIndexPixels = null;
        private uint[] depthPixels = null;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        private double angle;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public void OnWindowClosing(object sender, CancelEventArgs e)
        {
            writetext.Close();
            // Handle closing logic, set e.Cancel as needed
        }
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.bodyIndexFrameReader = this.kinectSensor.BodyIndexFrameSource.OpenReader();

           
            // wire handler for frame arrival
            //this.bodyIndexFrameReader.FrameArrived += this.Reader_FrameArrived;
            Closing += OnWindowClosing;
            this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;
            this.depthFrameDescription = kinectSensor.DepthFrameSource.FrameDescription;
            
            // allocate space to put the pixels being converted
            this.bodyIndexPixels = new uint[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];
            this.depthPixels = new uint[depthFrameDescription.Width * depthFrameDescription.Height];
            // create the bitmap to display
            this.bodyIndexBitmap = new WriteableBitmap(this.bodyIndexFrameDescription.Width, this.bodyIndexFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            // set IsAvailableChanged event notifier    
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            this.depthAndBodyIndexReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            this.depthAndBodyIndexReader.MultiSourceFrameArrived += this.MultisorceReader_FrameArrived;
            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        private void MultisorceReader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            if (multiSourceFrame == null)
                return;
            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                    return;
                using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                {
                    if (depthFrame == null)
                        return;
                    using (BodyIndexFrame bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
                    {
                        if (bodyIndexFrame == null)
                            return;
                        var bodyIndexBuffer = bodyIndexFrame.BodyIndexFrameSource;
                        GetBackPosition(depthFrame, bodyIndexFrame, bodyFrame);
                        // verify data and write the color data to the display bitmap
                        //if (((this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height) == bodyIndexBuffer.Size) &&
                        //    (this.bodyIndexFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.bodyIndexFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                        //{
                        //    this.ProcessBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size);
                        //    bodyIndexFrameProcessed = true;
                        //}
                    }
                }
            }
        }

        private void calculateRotationAngle(CameraSpacePoint leftShoulder, CameraSpacePoint rightShoulder, out double tan, out double cos)
        {
            tan = 0;
            cos = 1;
        
            var zDiff = rightShoulder.Z - leftShoulder.Z;
            var xDiff = rightShoulder.X - leftShoulder.X;
            tan = -zDiff / xDiff;
            cos = xDiff / (Math.Sqrt(xDiff * xDiff + zDiff * zDiff));
            if (Math.Abs(this.angle) > 1e-5)
            {
                var alpha = this.angle / 360f * 2 * Math.PI;
                tan = Math.Tan(alpha);
                cos = Math.Cos(alpha);
            }
        }

        private static Point depthSpacePointToPoint(DepthSpacePoint depthSpacePoint)
        {
            return new Point(depthSpacePoint.X, depthSpacePoint.Y);
        }
        double maxHeightM = Double.MinValue, minHeightM = Double.MaxValue;
        double maxDepthM = Double.MinValue, minDepthM = Double.MaxValue;
        private unsafe void GetBackPosition(DepthFrame depthFrame, BodyIndexFrame bodyIndexFrame, BodyFrame bodyFrame)
        {
            writetext.WriteLine("GetBackPosition:");
            if (this.bodies == null)
                this.bodies = new Body[6];
            bodyFrame.GetAndRefreshBodyData(this.bodies);
            Body activeBody = null;
            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    if (activeBody == null)
                        activeBody = body;
                    else
                        StatusText = "More than 1 body found";
                }
            }
            if (activeBody == null)
                return;
            writetext.WriteLine("Body found");
            var joints = activeBody.Joints;
            var backTracked =
                joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked &&
                joints[JointType.SpineBase].TrackingState == TrackingState.Tracked &&
                joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked &&
                joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                joints[JointType.Head].TrackingState == TrackingState.Tracked;
            if (!backTracked)
                return;
            writetext.WriteLine("Joints are tracked");
            var leftShoulder = joints[JointType.ShoulderLeft].Position;
            var rightShoulder = joints[JointType.ShoulderRight].Position;
            var spineBase = joints[JointType.SpineBase].Position;
            var spineShoulder = joints[JointType.SpineShoulder].Position;
            var neck = joints[JointType.Head].Position;

            calculateRotationAngle(leftShoulder, rightShoulder, out double tan, out double cos);
            var backPoints2D = new DepthSpacePoint[5];
            kinectSensor.CoordinateMapper.MapCameraPointsToDepthSpace(new CameraSpacePoint[5]
            { leftShoulder, rightShoulder, spineBase, spineShoulder, neck }, backPoints2D);
            var leftShoulder2D = depthSpacePointToPoint(backPoints2D[0]);
            var rightShoulder2D = depthSpacePointToPoint(backPoints2D[1]);
            var spineBase2D = depthSpacePointToPoint(backPoints2D[2]);
            var spineShoulder2D = depthSpacePointToPoint(backPoints2D[3]);
            var neck2D = depthSpacePointToPoint(backPoints2D[4]);

            var spineLine = Line2D.makeLine(spineBase2D, spineShoulder2D);
            var leftShoulderLine = spineLine.makeParalelLine(leftShoulder2D);
            var rightShoulderLine = spineLine.makeParalelLine(rightShoulder2D);
            var hipLine = spineLine.makePerpendicularLine(new Point(spineBase2D.X, spineBase2D.Y - 0.15*(spineShoulder2D.Y - spineBase2D.Y)));
            var neckLine = spineLine.makePerpendicularLine(neck2D);

            using (var depthBuffer = depthFrame.LockImageBuffer())
            {
                var width = depthFrameDescription.Width;
                var height = depthFrameDescription.Height;
           

                if (!(((width * height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel))))
                    return;
                using (var bodyIndexBuffer = bodyIndexFrame.LockImageBuffer())
                {
                    List<int> rightPoints = new List<int>(height);
                    for (int i = 0; i < height; i++)
                        rightPoints.Add(0);

                    Array.Clear(depthPixels, 0, depthPixels.Length);
                    var bodyIndexFrameDataSize = bodyIndexBuffer.Size;
                    var depthFrameDataSize = depthBuffer.Size;
                    if (bodyIndexFrameDataSize*2 != depthFrameDataSize)
                        return;
                    var bodyIndexFrameData = bodyIndexBuffer.UnderlyingBuffer;
                   
                    byte* frameData = (byte*)bodyIndexFrameData;
                    ushort* depthFrameData = (ushort*)depthBuffer.UnderlyingBuffer;
                    // convert body index to a visual representation
                    string a = "";

                    var depthSpacePoint = new DepthSpacePoint();
                    var point3D = new CameraSpacePoint();
                    
                    int[] diffs = new int[] { -1, 0, 1 };

                    for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
                    {
                        // the BodyColor array has been sized to match
                        // BodyFrameSource.BodyCount

                        if (frameData[i] < BodyColor.Length)
                        {
                            depthSpacePoint.X = i % width;
                            depthSpacePoint.Y = i / width;
                            point3D = kinectSensor.CoordinateMapper.MapDepthPointToCameraSpace(depthSpacePoint, depthFrameData[i]);
                            var x = i % width;
                            var y = i / width;
                            int neighbours = 0;
                            for (int xdiff = 0; xdiff < 3; xdiff++)
                                for (int ydiff = 0; ydiff < 3; ydiff++)
                                {
                                    var newX = x + xdiff;
                                    var newY = y + ydiff;
                                    var pos = newY * width + newX;
                                    if (pos > 0 && pos < (int)bodyIndexFrameDataSize && frameData[pos] == frameData[i])
                                        neighbours++;
                                }
                            //if (neighbours <= 2)
                            //    continue;
                            if (point3D.Y > maxHeightM)
                                maxHeightM = point3D.Y;
                            if (point3D.Y < minHeightM)
                                minHeightM = point3D.Y;

                            if (point3D.Z > maxDepthM)
                                maxDepthM = point3D.Z;
                            if (point3D.Z < minDepthM)
                                minDepthM = point3D.Z;

                            //StatusText = $"heigth: {minHeightM} ... {maxHeightM}, depth: {minDepthM} ... {maxDepthM}";

                        }
                    }
                    writetext.WriteLine($"heigth: {minHeightM} ... {maxHeightM}, depth: {minDepthM} ... {maxDepthM}");
                    //minHeightM = Math.Floor(minHeightM);
                    //maxHeightM = Math.Ceiling(maxHeightM);
                    //minHeightM = -1.3;
                    //maxHeightM = 1.3;
                    //minDepthM = 0;
                    bool pointIsOnBack;
                    int sX, sY;
                    sX = width - 1 - (int)(height / (maxHeightM - minHeightM) * (leftShoulder.Z - minDepthM));
                    sY = height - 1 - (int)((leftShoulder.Y - minHeightM) / (maxHeightM - minHeightM) * height + minHeightM);
                    writetext.WriteLine($"Left shoulder: {sX}, {sY}");
                    sX = (int)((sX - depthSpacePoint.X * tan) * cos);
                    writetext.WriteLine($"Left shoulder: {sX}, {sY}");
                    sX = width - 1 - (int)(height / (maxHeightM - minHeightM) * (rightShoulder.Z - minDepthM));
                    sY = height - 1 - (int)((rightShoulder.Y - minHeightM) / (maxHeightM - minHeightM) * height + minHeightM);
                    writetext.WriteLine($"Right shoulder: {sX}, {sY}");

                    sX = (int)((sX - depthSpacePoint.X * tan) * cos);
                    writetext.WriteLine($"Right shoulder: {sX}, {sY}");
                    Point point = new Point();
                    Line2D.PointPosition spineHipPosition = hipLine.GetPointPosition(spineShoulder2D);
                    Line2D.PointPosition spineNeckPosition = neckLine.GetPointPosition(spineShoulder2D);
                    Line2D.PointPosition leftShoulderRelativePosition, rightShoulderRelativePosition, hipRelativePosition, neckRelativePosition;
                    //List<Point> backPoints = new List<Point>();
                    int lastPointY = -1;
                    writetext.WriteLine("############## DATA TRANSFER BEGIN ##############");
                    for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
                    {
                        // the BodyColor array has been sized to match
                        // BodyFrameSource.BodyCount
                       
                        if (frameData[i] < BodyColor.Length)
                        {
                            point.X = i % width;
                            point.Y = i / width;
                            leftShoulderRelativePosition = leftShoulderLine.GetPointPosition(point);
                            rightShoulderRelativePosition = rightShoulderLine.GetPointPosition(point);
                            hipRelativePosition = hipLine.GetPointPosition(point);
                            neckRelativePosition = neckLine.GetPointPosition(point);

                            pointIsOnBack =
                                (leftShoulderRelativePosition == Line2D.PointPosition.Right || leftShoulderRelativePosition == Line2D.PointPosition.OnLine) &&
                                (rightShoulderRelativePosition == Line2D.PointPosition.Left || rightShoulderRelativePosition == Line2D.PointPosition.OnLine) &&
                                (hipRelativePosition == spineHipPosition || hipRelativePosition == Line2D.PointPosition.OnLine) &&
                                (neckRelativePosition == spineNeckPosition || neckRelativePosition == Line2D.PointPosition.OnLine);
                            //this.bodyIndexPixels[i] = BodyColor[frameData[i]];

                            // this pixel is part of a player,
                            // display the appropriate color
                            if (pointIsOnBack)
                                this.bodyIndexPixels[i] = 0xFFFFFFFF;
                            else
                                this.bodyIndexPixels[i] = BodyColor[frameData[i]];
                            //if (!pointIsOnBack)
                            //    continue;
                            if (lastPointY != -1 && lastPointY != i / width)
                                writetext.WriteLine("-100, -100");
                            lastPointY = i / width;

                            depthSpacePoint.X = i % width;
                            depthSpacePoint.Y = i / width;

                            point3D = kinectSensor.CoordinateMapper.MapDepthPointToCameraSpace(depthSpacePoint, depthFrameData[i]);
                            writetext.WriteLine($"{point3D.Z}, {point3D.Y}");
                            //int newX = (int) ((double)(depthFrame.DepthMaxReliableDistance - 2*depthFrameData[i])/(depthFrame.DepthMaxReliableDistance) * width);
                            int newX = width - 1 - (int)(height / (maxHeightM - minHeightM) * (point3D.Z - minDepthM));
                            var oldX = newX;
                            //var sin = Math.Sin(this.angle);
                            newX = (int)((newX - depthSpacePoint.X * tan) * cos);
                            //if (oldX < newX)
                            //    sin = 0;
                            //newX = (int)(newX / cos + (depthSpacePoint.X - newX * tan) * sin);
                            int newY = height - 1 - (int)((point3D.Y - minHeightM) / (maxHeightM - minHeightM) * (height-1));
                            this.depthPixels[newY * width + newX] = pointIsOnBack ? 0xFFFFFFFF : BodyColor[frameData[i]];
                          
                            CameraSpacePoint cameraPoint = kinectSensor.CoordinateMapper.MapDepthPointToCameraSpace(depthSpacePoint, depthFrameData[i]);
                            //Trace.WriteLine($"{cameraPoint.X} {cameraPoint.Y} {cameraPoint.Z}");
                            //a += cameraPoint.X;
                            //a += ' '; a += cameraPoint.Y;
                            //a += ' '; a += cameraPoint.Z;
                            //a += '\n';
                                                         //depthPixels[i]
                        }
                        else
                        {
                            // this pixel is not part of a player
                            // display black
                            this.bodyIndexPixels[i] = 0x00000000;
                        }
                    }
                    for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
                    {
                        var newX = i % width;
                        var newY = i / width;
                 
                        if (this.depthPixels[newY * width + newX] != 0x00000000)
                        {
                            int currentRightPoint = rightPoints[newY];
                            int pointNeighbours = 0;
                            for (int xDiffI = 0; xDiffI < 3; xDiffI++)
                            {
                                for (int yDiffI = 0; yDiffI < 3; yDiffI++)
                                {
                                    var adjustX = newX + diffs[xDiffI];
                                    var adjustY = newY + diffs[yDiffI];
                                    if (adjustX < 0 || adjustX >= width)
                                        continue;
                                    if (adjustY < 0 || adjustY >= height)
                                        continue;
                                    if (this.depthPixels[adjustY * width + adjustX] != 0x00000000)
                                    {
                                        pointNeighbours++;
                                    }
                                }
                            }
                            if (newX > currentRightPoint && pointNeighbours >= 8)
                                rightPoints[newY] = newX;
                        }
                    }
                    writetext.WriteLine("############## DATA TRANSFER END ##############");
                    for (int y = 0; y < depthFrameDescription.Height; y++)
                        for (int i = 0; i < 1; i++)
                            this.depthPixels[y * depthFrameDescription.Width + rightPoints[y] + i] = 0xFFFFFFFF;
                    //Trace.WriteLine(a);
                    this.RenderBodyIndexPixels();
                    this.depthBitmap.WritePixels(
                    new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                    this.depthPixels,
                    this.depthBitmap.PixelWidth * (int)BytesPerPixel,
                    0);
                }
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.bodyIndexBitmap;
            }
        }
    
        public ImageSource BackImageSource
        {
            get
            {
                return this.depthBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (this.bodyIndexFrameReader != null)
            {
                // remove the event handler
                this.bodyIndexFrameReader.FrameArrived -= this.Reader_FrameArrived;

                // BodyIndexFrameReder is IDisposable
                this.bodyIndexFrameReader.Dispose();
                this.bodyIndexFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.bodyIndexBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.bodyIndexBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-BodyIndex-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
                }
                catch (IOException)
                {
                    this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
                }
            }
        }

        /// <summary>
        /// Handles the body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyIndexFrameArrivedEventArgs e)
        {
            bool bodyIndexFrameProcessed = false;

            using (BodyIndexFrame bodyIndexFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyIndexFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer bodyIndexBuffer = bodyIndexFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height) == bodyIndexBuffer.Size) &&
                            (this.bodyIndexFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.bodyIndexFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                        {
                            this.ProcessBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size);
                            bodyIndexFrameProcessed = true;
                        }
                    }
                }
            }

            if (bodyIndexFrameProcessed)
            {
                this.RenderBodyIndexPixels();
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the BodyIndexFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the bodyIndexFrameData pointer.
        /// </summary>
        /// <param name="bodyIndexFrameData">Pointer to the BodyIndexFrame image data</param>
        /// <param name="bodyIndexFrameDataSize">Size of the BodyIndexFrame image data</param>
        private unsafe void ProcessBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize)
        {
            byte* frameData = (byte*)bodyIndexFrameData;

            // convert body index to a visual representation
            for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            {
                // the BodyColor array has been sized to match
                // BodyFrameSource.BodyCount
                if (frameData[i] < BodyColor.Length)
                {
                    // this pixel is part of a player,
                    // display the appropriate color
                    this.bodyIndexPixels[i] = BodyColor[frameData[i]];
                }
                else
                {
                    // this pixel is not part of a player
                    // display black
                    this.bodyIndexPixels[i] = 0x00000000;
                }
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderBodyIndexPixels()
        {
            this.bodyIndexBitmap.WritePixels(
                new Int32Rect(0, 0, this.bodyIndexBitmap.PixelWidth, this.bodyIndexBitmap.PixelHeight),
                this.bodyIndexPixels,
                this.bodyIndexBitmap.PixelWidth * (int)BytesPerPixel,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void TextBox_KeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == System.Windows.Input.Key.Enter)
            {
                var text = ((TextBox)sender).Text;
                if (Double.TryParse(text, out double angle))
                    this.angle = angle;
                else
                    StatusText = "Please input an integer";
            }
        }
    }
}
