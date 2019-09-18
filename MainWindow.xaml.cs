

namespace KinectBackCurveDetector
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
    using System.Timers;

    /// <summary>
    /// Interaction logic for the MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        private const bool debugData = false;
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
        private WriteableBitmap spineBitmap;
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private uint[] bodyIndexPixels = null;
        private uint[] spinePixels = null;
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
            this.spinePixels = new uint[depthFrameDescription.Width * depthFrameDescription.Height];
            // create the bitmap to display
            this.bodyIndexBitmap = new WriteableBitmap(this.bodyIndexFrameDescription.Width, this.bodyIndexFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.spineBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            // set IsAvailableChanged event notifier    
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;


            // Create reader for depth and body data
            // To calculate back position
            this.depthAndBodyIndexReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);
            
            this.depthAndBodyIndexReader.MultiSourceFrameArrived += this.MultisorceReader_FrameArrived;
            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? "Kinect is running"
                                                            : "Kinect is not available";

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        private Body getActiveBody(BodyFrame bodyFrame)
        {
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
                    {
                        StatusText = "More than 1 body found";
                        return null;
                    }
                }
            }
            return activeBody;
        }

        private bool aquireFrames(MultiSourceFrameArrivedEventArgs e, out BodyFrame bodyFrame, out DepthFrame depthFrame, out BodyIndexFrame bodyIndexFrame)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            bodyFrame = null;
            depthFrame = null;
            bodyIndexFrame = null;

            if (multiSourceFrame == null)
                return false;

            bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();
            if (bodyFrame == null)
                return false;

            depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
            if (depthFrame == null)
            {
                bodyFrame.Dispose();
                return false;

            }

            bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
            if (bodyIndexFrame == null)
            {
                bodyFrame.Dispose();
                depthFrame.Dispose();
                return false;
            }

            return true;
            
        }
        private bool aquireBodyDataAndBuffers(
            BodyFrame bodyFrame, DepthFrame depthFrame, BodyIndexFrame bodyIndexFrame, 
            out Body body, out KinectBuffer depthBuffer, out KinectBuffer bodyIndexBuffer)
        {
            depthBuffer = null;
            bodyIndexBuffer = null;

            body = getActiveBody(bodyFrame);
            if (body == null)
                return false;

            depthBuffer = depthFrame.LockImageBuffer();
            var width = depthFrameDescription.Width;
            var height = depthFrameDescription.Height;
            if (depthBuffer == null ||
                (width * height) != (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel))
                return false;

            bodyIndexBuffer = bodyIndexFrame.LockImageBuffer();
            if (bodyIndexBuffer == null || bodyIndexBuffer.Size * 2 != depthBuffer.Size)
            {
                depthBuffer.Dispose();
                return false;
            }
            return true;

        }
        private void MultisorceReader_FrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // acquire all frames and then process them
            if (!aquireFrames(e, out var bodyFrame, out var depthFrame, out var bodyIndexFrame))
                return;

            if (!aquireBodyDataAndBuffers(bodyFrame, depthFrame, bodyIndexFrame, out var body, out var depthBuffer, out var bodyIndexBuffer))
            {
                bodyFrame.Dispose();
                depthFrame.Dispose();
                bodyIndexFrame.Dispose();
                return;
            }
            timer.Restart();

            // calculate and show the back position on the screen
            var spinePoints = GetSpinePoints(body, depthBuffer, bodyIndexBuffer);
           

        
            drawSpinePoints(spinePoints);
            RenderBodyIndexPixels();

            timer.Stop();
            writetext.WriteLine($"{1000.0 / timer.ElapsedMilliseconds} fps, {timer.ElapsedMilliseconds} ms");

            // dispose of data
            depthBuffer.Dispose();
            bodyIndexBuffer.Dispose();

            bodyFrame.Dispose();
            depthFrame.Dispose();
            bodyIndexFrame.Dispose();
        }

        Comparison<Point> pointCompare = delegate (Point p1, Point p2)
        {
            return p1.X.CompareTo(p2.X);
        };
        Stopwatch timer = new Stopwatch();
        private void drawSpinePoints(List<Point> spinePoints)
        {
            if (spinePoints == null)
                return;
            Array.Clear(spinePixels, 0, spinePixels.Length);

            var width = depthFrameDescription.Width;
            var height = depthFrameDescription.Height;

            // calculate max and mix points 
            // of the spine
            int maxHeightPoint = 0, minHeightPoint = 0;
            int maxZPoint = 0, minZPoint = 0;
            for (int i = 0; i < spinePoints.Count; i++)
            {
                if (spinePoints[minHeightPoint].Y > spinePoints[i].Y)
                    minHeightPoint = i;
                if (spinePoints[maxHeightPoint].Y < spinePoints[i].Y)
                    maxHeightPoint = i;

                if (spinePoints[minZPoint].X > spinePoints[i].X)
                    minZPoint = i;
                if (spinePoints[maxZPoint].X < spinePoints[i].X)
                    maxZPoint = i;

            }
            double roundFactor = 0.3;

            // round them, otherwise the picture is to shaky
            // because max and min value are slightly different from frame to frame
            double maxHeight = BackTrackerHelper.upperBound(spinePoints[maxHeightPoint].Y, roundFactor);
            double minHeight = BackTrackerHelper.lowerBound(spinePoints[minHeightPoint].Y, roundFactor);
            double maxZ = BackTrackerHelper.upperBound(spinePoints[maxZPoint].X, roundFactor);
            double minZ = BackTrackerHelper.lowerBound(spinePoints[minZPoint].X, roundFactor);
            double conversionRateY = height / (maxHeight - minHeight);
            double conversionRateZ = width / (maxZ - minZ);
            double conversionRate = Math.Min(conversionRateY, conversionRateZ);

            // convert coordinates of a spine to pixels on the screen
            for (int i = 0; i < spinePoints.Count; i++)
            {
                int pictureY = height - 1 - (int)(conversionRate * (spinePoints[i].Y - minHeight));
                int pictureX = (int)(conversionRate * (spinePoints[i].X - minZ));
                this.spinePixels[pictureY * width + pictureX] = BodyColor[0];
            }
            
            this.spineBitmap.WritePixels(
                new Int32Rect(0, 0, this.spineBitmap.PixelWidth, this.spineBitmap.PixelHeight),
                    this.spinePixels,
                    this.spineBitmap.PixelWidth * (int)BytesPerPixel,
                    0);
            

        }
        private unsafe List<Point> GetSpinePoints(Body activeBody,KinectBuffer depthBuffer, KinectBuffer bodyIndexBuffer)
        {
            var joints = activeBody.Joints;
            if (!BackTrackerHelper.isBodyTracked(joints))
                return null;

            // calculateRotationAngle(leftShoulder, rightShoulder, out double tan, out double cos);

            var width = depthFrameDescription.Width;
            var height = depthFrameDescription.Height;

            var bodyIndexFrameData = bodyIndexBuffer.UnderlyingBuffer;
            byte* frameData = (byte*)bodyIndexFrameData;
            ushort* depthFrameData = (ushort*)depthBuffer.UnderlyingBuffer;
            
            // preparation work
            List<Point> spinePoints = new List<Point>(height);
            BackArea backArea = new BackArea(kinectSensor, joints);
            int lastPointY = -1;
            Point point = new Point();
            var depthSpacePoint = new DepthSpacePoint();
            var point3D = new CameraSpacePoint();

            if (debugData)
                writetext.WriteLine("############## FRAME START ##############");
            List<Point> rowPoints = new List<Point>();
            var size = bodyIndexBuffer.Size;
            for (int i = 0; i < (int)size; ++i)
            {
                // the BodyColor array has been sized to match
                // BodyFrameSource.BodyCount


                // check if point belong to a person
                if (frameData[i] < BodyColor.Length)
                {
                    point.X = i % width;
                    point.Y = i / width;
                    depthSpacePoint.X = i % width;
                    depthSpacePoint.Y = i / width;

                    // check if point is on the back
                    var pointIsOnBack = backArea.isPointInSpineArea(point);
                    if (pointIsOnBack)
                        this.bodyIndexPixels[i] = 0xFFFFFFFF;
                    else
                        this.bodyIndexPixels[i] = BodyColor[frameData[i]];
                    if (!pointIsOnBack)
                        continue;

                    // fill the current row of points (pixels that have the same Y)
                    point3D = kinectSensor.CoordinateMapper.MapDepthPointToCameraSpace(depthSpacePoint, depthFrameData[i]);
                    rowPoints.Add(new Point(point3D.Z, point3D.Y));
                    if (debugData)
                        writetext.WriteLine($"{point3D.Z}, {point3D.Y}");
                    if (i / width != lastPointY && rowPoints.Count != 0)
                    {
                        // if the current pixel doesn't belong to the same row
                        // process the current row and calculate the spine point from it
                        rowPoints.Sort(pointCompare);
                        spinePoints.Add(BackTrackerHelper.calculateSpinePoint(rowPoints));
                        rowPoints.Clear();
                        if (debugData)
                            writetext.WriteLine("-100, -100");
                    }
                    lastPointY = i / width;
                }
                else
                {
                    this.bodyIndexPixels[i] = 0x00000000;
                }
            }

            if (debugData)
                writetext.WriteLine("************** FRAME END **************");

            // process the last line of pixels
            if (rowPoints.Count != 0)
            {
                rowPoints.Sort(pointCompare);
                spinePoints.Add(BackTrackerHelper.calculateSpinePoint(rowPoints));
                rowPoints.Clear();
            }
            return spinePoints;
           
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
                return this.spineBitmap;
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
            if (this.depthAndBodyIndexReader != null)
            {
                this.depthAndBodyIndexReader.MultiSourceFrameArrived -= this.MultisorceReader_FrameArrived;
                this.depthAndBodyIndexReader.Dispose();
                this.depthAndBodyIndexReader = null;
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
            this.StatusText = this.kinectSensor.IsAvailable ? "Kinect is connected"
                                                         : "Kinect is not available";
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
