//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using OpenCvSharp;
    using static TimingScanner.ScannerUtils;
    using static TimingScanner.BendClassifier;
    using Microsoft.VisualBasic.FileIO;
    using System.Threading.Tasks;
    using System.Text;
    using System.Collections.Generic;


    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged  // !!! izmjenjeno sa Window na System.Windows.Window da ne dolazi do preklapanja sa window iz OpenCvSharp biblioteke
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        //List<ushort> depthFrameData = new List<ushort>();  //!!! dodato
        List<List<ushort>> depthFrameDataList = new List<List<ushort>>();  //!!! dodato
        //ushort[] depthFrameData = null;  //!!! dodato
        Mat testImage = new Mat();  //!!! dodato
        int cntFrm = 0;     //!!! dodato
        int cntFrmCsv = 0;      //!!! dodato
        int bufferSize = 20;        //!!! dodato

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            //this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;  !!! zakomentarisano zbog citanja iz csv fajla

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            //this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, 300, 96.0, 96.0, PixelFormats.Gray8, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            this.InitializeComponent();


            //samo za citanje iz csv fajla:  !!! ... dodato 
            // {

            // dobri
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\dordje_nivelisan_pravilan_luk\dordje.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\isecak1\dordje.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\isecak_male_nozice\isecak_male_nozice.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\sirovaHorizontalnaElipsa\sirovaHorizontalnaElipsa.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\nezgodanPolukrug\NEYGODAN.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\novo\horizontalna_elipsa_1.csv", this.depthPixels);
            Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\novo\horizontalna_elipsa_1_rot1.csv", this.depthPixels);

            // nisu dobri
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\HE\dordje.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\ve\dordje.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\klasa4\klasa4.csv", this.depthPixels);
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\klasa5\klasa5.csv", this.depthPixels);

            // ne valjaju nista
            //Task t = ReadDepthFrameDataFromCSVasync(@"C:\Users\Djordje\Documents\TIMING_skener\Primjeri lukova\isecak2\dordje.csv", this.depthPixels);

            // }


            // initialize the components (controls) of the window
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
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
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
            if (this.depthBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

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
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;
            this.cntFrm += 1; //dodato !!!

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }

        public async Task ReadDepthFrameDataFromCSVasync(string path, byte[] depthPixels)   // dodato !!!
        {
            //StringBuilder strB = new StringBuilder(); /////

            TextFieldParser parser = new TextFieldParser(path);

            //Console.WriteLine(parser.GetType().Name);
            parser.TextFieldType = FieldType.Delimited;
            parser.SetDelimiters(",");
            ushort[] depthFrmData = null;

            //int cnt = 0;    /////
            while (!parser.EndOfData)
            {
                this.cntFrmCsv++;      /////
                //Processing row
                string[] fields = parser.ReadFields();

                depthFrmData = Array.ConvertAll(fields, s => ushort.Parse(s));

                ProcessDepthFrameDataFromCsv(depthFrmData, depthPixels, 0, ushort.MaxValue);
                RenderDepthPixels();
                //strB.Append(this.ClassifyBend() + "\n"); /////

                if (this.cntFrmCsv <= 5)
                {
                    classifyButton.IsEnabled = false;
                    //continue;
                }
                else
                {
                    if (depthFrameDataList.Count < bufferSize)
                    {
                        classifyButton.IsEnabled = false;
                        this.depthFrameDataList.Add(depthFrmData.ToList<ushort>());
                    }
                    else
                    {
                        classifyButton.IsEnabled = true;
                        this.depthFrameDataList.RemoveAt(0);
                        this.depthFrameDataList.Add(depthFrmData.ToList<ushort>());
                    }
                }
                await Task.Delay(1);

                //if (cnt == 50) /////
                //{              /////
                //    break;     /////
                //}              /////
            }
            //File.WriteAllText(@"C:\Users\Djordje\Documents\TIMING_skener\MyNewTextFile3.txt", strB.ToString());  /////
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            ushort[] depthFrmData = new ushort[(int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel)]; // dodato !!!

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];
                depthFrmData[i] = depth;      //dodato !!!

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }

            // dodato ... {
            if (this.cntFrm > 5)
            {
                if (depthFrameDataList.Count < bufferSize)
                {
                    classifyButton.IsEnabled = false;
                    this.depthFrameDataList.Add(depthFrmData.ToList<ushort>());
                }
                else
                {
                    classifyButton.IsEnabled = true;
                    this.depthFrameDataList.RemoveAt(0);
                    this.depthFrameDataList.Add(depthFrmData.ToList<ushort>());
                }
            }
            else
            {
                classifyButton.IsEnabled = false;
            }
            //this.depthFrameData = depthFrmData;  // dodato !!!
            // ... }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
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


        /// <summary>
        /// Resetuje bafer frejmova za klasifikaciju
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            this.depthFrameDataList.Clear();
            this.cntFrm = 0;
            this.cntFrmCsv = 0;

        }


        /// <summary>
        /// Poziva funkciju za klasifikaciju redom frejmova iz bafera
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ClassifyButton_Click(object sender, RoutedEventArgs e)
        {
            var watchAll = new System.Diagnostics.Stopwatch();
            watchAll.Start();

            int h = this.depthBitmap.PixelHeight;
            int w = this.depthBitmap.PixelWidth;
            ushort maxDepth = Convert.ToUInt16(MaxDepthInput.Text);

            List<List<ushort>> depthFrameDataListForCheck = this.depthFrameDataList;
            List<string> resultArray = new List<string>();


            //PARALELIZACIJA
            int numParts = 4;
            if (bufferSize % numParts != 0)  // ako bufferSize nije djeljivo sa numParts, numParts se podesava na bufferSize
            {
                numParts = bufferSize * 1;
            }
            List<List<string>> tempResultArray = new List<List<string>>();

            Parallel.For(0, numParts, i => tempResultArray.Add(ClassifyBend(depthFrameDataListForCheck.GetRange(i * (bufferSize / numParts), bufferSize / numParts), h, w, maxDepth)));

            for (int i = 0; i < tempResultArray.Count; i++)
            {
                resultArray.AddRange(tempResultArray[i]);
            }


            //BEZ PARALELIZACIJE:
            //resultArray = ClassifyBend(depthFrameDataListForCheck, h, w, maxDepth);

            watchAll.Stop();
            Console.WriteLine($"Execution Time TOTAL: {watchAll.ElapsedMilliseconds} ms");
            //MessageBox.Show(this.ClassifyBend());


            TimingScanner.ResultWindow resultWindow = new TimingScanner.ResultWindow(resultArray.ToArray());
            resultWindow.Show();
        }


        /// <summary>
        /// Za svaki frejm vrsi procesiranje slike, a zatim klasifikaciju profila sa procesiranjem pripremljene slike
        /// </summary>
        /// <param name="depthFrameDataListForCheck">lista frejmova za klasifikaciju</param>
        private List<string> ClassifyBend(List<List<ushort>> depthFrameDataListForCheck, int h, int w, ushort maxDepth)
        {
            //var watchAll = new System.Diagnostics.Stopwatch();
            //watchAll.Start();

            List<string> resultArr = new List<string>();
            byte[] depthPixelsClassify = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            Mat testImage = new Mat();

            //for (int i = 0; i < bufferSize; i++)
            for (int i = 0; i < depthFrameDataListForCheck.Count; i++)
            {
                //Depth scale
                //var watch = new System.Diagnostics.Stopwatch();
                //watch.Start();
                //ProcessDepthFrameDataFromCsv(depthFrameDataListForCheck[i].ToArray(), depthPixelsClassify, 0, Convert.ToUInt16(MaxDepthInput.Text));
                //this.testImage = Byte1DToMat(this.depthBitmap.PixelHeight, this.depthBitmap.PixelWidth, depthPixelsClassify);
                ProcessDepthFrameDataFromCsv(depthFrameDataListForCheck[i].ToArray(), depthPixelsClassify, 0, maxDepth);
                testImage = Byte1DToMat(h, w, depthPixelsClassify);
                //watch.Stop();
                //Console.WriteLine($"Execution Time Depth Scale: {watch.ElapsedMilliseconds} ms");
                //ShowImage("Depth Scaled Image", testImage);

                //Black-White image
                //watch = new System.Diagnostics.Stopwatch();
                //watch.Start();
                testImage = ToBlackWhiteImage(testImage);
                //watch.Stop();
                //Console.WriteLine($"Execution Time Black White: {watch.ElapsedMilliseconds} ms");
                //ShowImage("Black-White Image", testImage);


                //Erode - Dilate
                //watch = new System.Diagnostics.Stopwatch();
                //watch.Start();
                testImage = RemoveNoise(testImage);
                //watch.Stop();
                //Console.WriteLine($"Execution Time Erode Dilate: {watch.ElapsedMilliseconds} ms");
                //ShowImage("Erode-Dilate Image", testImage);

                //Extract Largest
                //watch = new System.Diagnostics.Stopwatch();
                //watch.Start();
                testImage = ExtractLargestContour(testImage, false);
                //watch.Stop();
                //Console.WriteLine($"Execution Time Extract Largest: {watch.ElapsedMilliseconds} ms");
                //ShowImage("Extract Largest Contour", testImage);

                //Correct Rotation
                //watch = new System.Diagnostics.Stopwatch();
                //watch.Start();
                testImage = CorrectRotationFinal(testImage);
                //watch.Stop();
                //Console.WriteLine($"Execution Time CorrectRotation: {watch.ElapsedMilliseconds} ms");
                //ShowImage("Rotation Corrected", testImage);

                //Classification Result
                string result = Classify(testImage);
                resultArr.Add(result);

                //break;

            }

            //watchAll.Stop();
            //Console.WriteLine($"Execution Time TOTAL: {watchAll.ElapsedMilliseconds} ms");

            return resultArr.Cast<string>().ToList();
        }

    }
}
