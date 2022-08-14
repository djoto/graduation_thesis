using Microsoft.VisualBasic.FileIO;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.DepthBasics.TimingScanner
{
    using static TimingScanner.BendClassifier;
    static class ScannerUtils
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Prikazuje sliku u originalnim dimenzijama
        /// </summary>
        /// <param name="imageWindowTitle">naslov na prozoru u kome se slika otvara</param>
        /// <param name="mat">matrica piksela tipa Mat</param>
        public static void ShowImage(string imageWindowTitle, Mat img)
        {

            Cv2.ImShow(imageWindowTitle, img);
        }

        /// <summary>
        /// Mapira niz 16-bitnih integera (short[]) u niz 8-bitnih vrijednosti (byte[]) uz mogucnost skaliranja po dubini
        /// </summary>
        /// <param name="depthFrameData">niz 16-bitnih int vrijednosti dubina piksela</param>
        /// <param name="depthPixels">niz u kome ce biti 8-bitne vrijednost dubina piksela</param>
        /// <param name="minDepth">minimalna vrijednost dubine piksela koja ce se uzeti u obzir</param>
        /// <param name="maxDepth">maksimalna vrijednost dubine piksela koja ce se uzeti u obzir</param>
        public static unsafe void ProcessDepthFrameDataFromCsv(ushort[] depthFrameData, byte[] depthPixels, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value

            // convert depth to a visual representation
            for (int i = 0; i < depthFrameData.Length; ++i)
            {
                // Get the depth for this pixel
                ushort depth = depthFrameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        /// <summary>
        /// Vraca matricu slike tipa Mat (1 channel!) od 1D niza 8-bitnih integera (byte[])
        /// </summary>
        /// <param name="h">broj redova matrice - visina slike</param>
        /// <param name="w">broj kolona matrice - sirina slike</param>
        /// <param name="byte1D">1D niz 16-bitnih int vrijednosti dubina piksela</param>
        public static Mat Byte1DToMat(int h, int w, byte[] byte1D)
        {
            Mat imageMat = new Mat(h, w, MatType.CV_8UC1);
            imageMat.SetArray(0, 0, byte1D);
            return imageMat;
        }

        /// <summary>
        /// Vraca matricu slike tipa Mat (1 channel!) od 2D niza 8-bitnih integera (byte[,])
        /// </summary>
        /// <param name="byte2D">2D niz 16-bitnih int vrijednosti dubina piksela</param>
        public static Mat Byte2DToMat(byte[,] byte2D)
        {
            Mat imageMat = new Mat(byte2D.GetLength(0), byte2D.GetLength(1), MatType.CV_8UC1);
            imageMat.SetArray(0, 0, byte2D);
            return imageMat;
        }

        /// <summary>
        /// Vraca 1D niz 8-bitnih integera (byte[]) od matrice slike tipa Mat (1 channel!)
        /// </summary>
        /// <param name="mat">matrica piksela tipa Mat (1 channel!)</param>
        public static byte[] MatToByte1D(Mat mat)
        {
            byte[] b = new byte[mat.Cols * mat.Rows];
            mat.GetArray(0, 0, b);
            return b;
        }

        /// <summary>
        /// Vraca 2D niz 8-bitnih integera (byte[,]) od matrice slike tipa Mat (1 channel!)
        /// </summary>
        /// <param name="mat">matrica piksela tipa Mat (1 channel!)</param>
        public static byte[,] MatToByte2D(Mat mat)
        {
            byte[,] b = new byte[mat.Rows, mat.Cols];
            mat.GetArray(0, 0, b);
            return b;
        }

        /// <summary>
        /// Vraca matricu slike tipa Mat (1 channel!) sa otklonjenim sumom
        /// </summary>
        /// <param name="srcImg">slika - matrica piksela tipa Mat (1 channel!)</param>
        public static Mat RemoveNoise(Mat srcImg)
        {
            Mat dstImg = new Mat();
            srcImg.CopyTo(dstImg);

            Mat elementErode1 = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(3, 3), new Point(1, 1));
            Mat elementDilate1 = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(7, 7), new Point(3, 3));
            Mat closingElement = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(7, 7), new Point(3, 3));
            Mat elementErode2 = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(7, 7), new Point(3, 3));
            Mat elementDilate2 = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(3, 3), new Point(1, 1));

            Cv2.Erode(dstImg, dstImg, elementErode1);
            Cv2.Dilate(dstImg, dstImg, elementDilate1);
            Cv2.MorphologyEx(dstImg, dstImg, MorphTypes.Close, closingElement);
            Cv2.Erode(dstImg, dstImg, elementErode2);
            Cv2.Dilate(dstImg, dstImg, elementDilate2);

            //Cv2.GaussianBlur(dstImg, dstImg, new Size(3, 3), 0);
            //dstImg = ToBlackWhiteImage(dstImg);
            //ShowImage("gaus", ToBlackWhiteImage(dstImg));

            return dstImg;
        }

        /// <summary>
        /// Sluzi za izdvajanje najvece konture sa slike (najveca kontura je po pravilu kontura savijenog profila!)
        /// Vraca odsjecenu sliku sa najvecom konturom koja se nalazi u srcImg.
        /// </summary>
        /// <param name="srcImg">slika - matrica piksela tipa Mat (1 channel!)</param>
        public static Mat ExtractLargestContour(Mat srcImg, bool onlyFind)
        {
            Point[][] contours = null;
            HierarchyIndex[] hierarchy = null;
            Mat dstImg = new Mat();
            srcImg.CopyTo(dstImg);
            Cv2.FindContours(dstImg, out contours, out hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxNone);

            Rect maxRect = new Rect();
            int maxContourIndex = -1;
            for (int i = 0; i < contours.Length; i++)
            {
                Rect rct = Cv2.BoundingRect(contours[i]);
                if (i == 0)
                {
                    maxRect = rct;
                    maxContourIndex = 0;
                }

                if ((rct.Height * rct.Width) > (maxRect.Height * maxRect.Width))
                {
                    maxRect = rct;
                    maxContourIndex = i * 1;
                }

            }
            Mat newImg = new Mat(dstImg.Size(), MatType.CV_8UC1, 0); //generise potpuno crnu sliku dimenzije kao dstImg
            //Cv2.ImShow("prava", dstImg);
            Cv2.DrawContours(newImg, contours, maxContourIndex, 255, -1);
            newImg.CopyTo(dstImg);

            //podesavamo pravougaonik da ne preklapa ivice profila
            maxRect.X = maxRect.X - 1;
            maxRect.Y = maxRect.Y - 1;
            maxRect.Height = maxRect.Height + 2;
            maxRect.Width = maxRect.Width + 2;

            if (maxRect.Y == -1 || maxRect.X == -1)
            {
                return null;
            }


            if (onlyFind)
            {
                return dstImg;
            }

            // odsijecanje slike na dimenzije pravougaonika oko konture 
            dstImg = dstImg.RowRange(maxRect.Y + 1, maxRect.Y + maxRect.Height - 1);
            dstImg = dstImg.ColRange(maxRect.X + 1, maxRect.X + maxRect.Width - 1);

            return dstImg;
        }

        /// <summary>
        /// Sluzi za prebacivanje slike u crno-bijelu tako sto ce sve sto nije bilo crno u srcImg biti bijelo
        /// </summary>
        /// <param name="srcImg">slika - matrica piksela tipa Mat (1 channel!)</param>
        public static Mat ToBlackWhiteImage(Mat srcImg)
        {
            Cv2.Threshold(srcImg, srcImg, 0, 255, ThresholdTypes.Binary);
            return srcImg;
        }

        /// <summary>
        /// Sluzi za skaliranje slike tako sto odsijeca crni okvir oko konture, tako da ostaje samo slika cije ivice dodiruju ivice konture.
        /// Koristi se kada na slici imamo samo bijelu konturu profila na crnoj pozadini.
        /// Dimenzije slike koju funkcija vraca se mijenjaju u odnosu na srcImg. 
        /// </summary>
        /// <param name="srcImg">slika - matrica piksela tipa Mat (1 channel!)</param>
        public static Mat ScaleImageToOnlyContour(Mat srcImg)
        {
            int fromTop = 0;
            int fromLeft = 0;
            int fromRight = 0;
            int fromBottom = 0;

            bool top = false;
            bool left = false;
            bool right = false;
            bool bottom = false;

            byte[,] pixels2D = MatToByte2D(srcImg);

            // za odozgo:
            for (int i = 0; i < pixels2D.GetLength(0); i++)
            {
                for (int j = 0; j < pixels2D.GetLength(1); j++)
                {
                    if (pixels2D[i, j] != 0)
                    {
                        fromTop = i;
                        top = true;
                        break;
                    }
                }
                if (top)
                {
                    break;
                }
            }

            // za lijevo:
            for (int j = 0; j < pixels2D.GetLength(1); j++)
            {
                for (int i = 0; i < pixels2D.GetLength(0); i++)
                {
                    if (pixels2D[i, j] != 0)
                    {
                        fromLeft = j;
                        left = true;
                        break;
                    }
                }
                if (left)
                {
                    break;
                }
            }

            // za odozdo
            for (int i = pixels2D.GetLength(0) - 1; i >= 0; i--)
            {
                for (int j = 0; j < pixels2D.GetLength(1); j++)
                {
                    if (pixels2D[i, j] != 0)
                    {
                        fromBottom = i;
                        bottom = true;
                        break;
                    }
                }
                if (bottom)
                {
                    break;
                }
            }

            // za desno:
            for (int j = pixels2D.GetLength(1) - 1; j >= 0; j--)
            {
                for (int i = 0; i < pixels2D.GetLength(0); i++)
                {
                    if (pixels2D[i, j] != 0)
                    {
                        fromRight = j;
                        right = true;
                        break;
                    }
                }
                if (right)
                {
                    break;
                }
            }

            int scaledHeight = fromBottom - fromTop + 1;
            int scaledWidth = fromRight - fromLeft + 1;

            byte[,] scaledPixels2D = new byte[scaledHeight, scaledWidth];
            for (int i = 0; i < scaledHeight; i++)
            {
                for (int j = 0; j < scaledWidth; j++)
                {
                    scaledPixels2D[i, j] = pixels2D[i + fromTop, j + fromLeft];
                }
            }

            return Byte2DToMat(scaledPixels2D);
        }

        /// <summary>
        /// Vraca niz Y koordinata spoljasnje ivice profila koji je okrenut kracima ka gore 
        /// Duzina izlaznog niza jednaka je sirini slike (uzimaju se koordinate po visini za svaki piksel po X).
        /// </summary>
        /// <param name="scaledImg">skalirana slika do ivica profila i pravilno rotirana kracima ka gore</param>
        public static int[] GetArrayY(Mat scaledImg)
        {
            byte[,] byteImg2D = MatToByte2D(scaledImg);
            int[] array = new int[scaledImg.Cols];

            for (int j = 0; j < scaledImg.Cols; j++)
            {
                for (int i = (scaledImg.Rows - 1); i >= 0; i--)
                {
                    if (byteImg2D[i, j] != 0)
                    {
                        array[j] = byteImg2D.GetLength(0) - i;
                        break;
                    }
                }
            }

            return array;
        }

        /// <summary>
        /// Vraca sliku promenjenih dimenzija, s tim da se kao argument proslijedjuje samo sirina.
        /// Visina se podesava preko propocije tako da odnos visine i sirine ulazne slike bude jedank odnosu visine i sirine izlazne slike 
        /// </summary>
        /// <param name="srcImg">matrica piksela slike tipa Mat</param>
        /// <param name="sizeX">sirina nove slike na izlazu</param>
        public static Mat ScaledResize(Mat srcImg, int sizeX)
        {
            float scaledY = (srcImg.Rows * sizeX) / srcImg.Cols;
            int sizeY = Convert.ToInt32(scaledY);
            Mat retImg = srcImg.Resize(new Size(sizeX, sizeY));
            return retImg;
        }


        /// <summary>
        /// Vraca gornju polovinu slike. Dimenzije izlazne slike su h=srcImg.h/2, w=srcImg.w
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        public static Mat UpperHalf(Mat srcImg)
        {
            return srcImg.RowRange(0, srcImg.Size(0) / 2);
        }

        /// <summary>
        /// Vraca donju polovinu slike. Dimenzije izlazne slike su h=srcImg.h/2, w=srcImg.w
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        public static Mat LowerHalf(Mat srcImg)
        {
            return srcImg.RowRange(srcImg.Size(0) / 2, srcImg.Size(0));
        }

        /// <summary>
        /// Vraca lijevu polovinu slike. Dimenzije izlazne slike su h=srcImg.h, w=srcImg.w/2
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        public static Mat LeftHalf(Mat srcImg)
        {
            return srcImg.ColRange(0, srcImg.Size(1) / 2);
        }

        /// <summary>
        /// Vraca desnu polovinu slike. Dimenzije izlazne slike su h=srcImg.h, w=srcImg.w/2
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        public static Mat RightHalf(Mat srcImg)
        {
            return srcImg.ColRange(srcImg.Size(1) / 2, srcImg.Size(1));
        }


        /// <summary>
        /// Rotira sliku za odredjeni ugao suprotno kazaljci na satu za pozitivan ugao odnosno u pravcu kretanja kazaljke na satu za negativan ulazni ugao.
        /// Ne gube se konture sa slike prilikom rotiranja jer se dimenzije slike koju funkcija vraca mijenjaju
        /// </summary>
        /// <param name="src">matrica slike tipa Mat</param>
        /// <param name="angle">ugao za koji rotiramo sliku</param>
        public static Mat MatRotate(Mat src, float angle)
        {
            Mat dst = new Mat();
            Point2f center = new Point2f(src.Cols / 2, src.Rows / 2);
            Mat rot = Cv2.GetRotationMatrix2D(center, angle, 1);
            Size2f s2f = new Size2f(src.Size().Width, src.Size().Height);
            Rect box = new RotatedRect(new Point2f(0, 0), s2f, angle).BoundingRect();
            double xx = rot.At<double>(0, 2) + box.Width / 2 - src.Cols / 2;
            double zz = rot.At<double>(1, 2) + box.Height / 2 - src.Rows / 2;
            rot.Set(0, 2, xx);
            rot.Set(1, 2, zz);
            Cv2.WarpAffine(src, dst, rot, box.Size);

            return ToBlackWhiteImage(dst);
        }



        /// <summary>
        /// Vraca sliku novih dimenzija
        /// </summary>
        /// <param name="srcImg">matrica slike tipa Mat</param>
        /// <param name="h">nova visina</param>
        /// <param name="w">nova sirina</param>
        public static Mat ResizeImage(Mat srcImg, int h, int w)
        {
            return srcImg.Resize(new Size(h, w));
        }


        /// <summary>
        /// Prebacuje niz vrijednosti tipa int u niz vrijednosti tipa float
        /// </summary>
        /// <param name="arr">ulazni niz int[]</param>
        public static float[] IntToFloatArray(int[] arr)
        {
            float[] floatArr = new float[arr.Length];
            for (int i = 0; i < arr.Length; i++)
            {
                floatArr[i] = (float)arr[i];
            }
            return floatArr;
        }


        /// <summary>
        /// Implementira average pooling tehniku za uglacavanje vrijednosti odbiraka
        /// </summary>
        /// <param name="srcArr">ulazni niz float[]</param>
        /// <param name="filterSize">duzina filtera (mora biti neparan broj! - ako nije, funkcija vraca ulazni niz)</param>
        /// <param name="numIter">broj iteracija kroz koji ce se obavljati uglacavanje sa definisanim filterom (default 1)</param>
        public static float[] MovingAverageSmooth1D(float[] srcArr, int filterSize, int numIter = 1)
        {
            float[] dstArr = new float[srcArr.Length];
            srcArr.CopyTo(dstArr, 0);
            if (filterSize % 2 != 0)
            {
                for (int n = 0; n < numIter; n++)
                {
                    for (int i = 0; i < dstArr.Length; i++)
                    {
                        if ((i < filterSize / 2) || (i > (dstArr.Length - filterSize / 2 - 1)))
                        {
                            continue;
                        }
                        else
                        {
                            float sum = 0.0f;
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sum += dstArr[i - k];
                            }
                            sum += dstArr[i];
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sum += dstArr[i + k];
                            }
                            dstArr[i] = sum / filterSize;
                        }
                    }
                }
                return dstArr;
            }
            else
            {
                return dstArr;
            }
        }


        /// <summary>
        /// Sluzi da izdvoji tacke koje cine srednju liniju konture
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        /// <param name="startEndContourPoints">pocetna i krajnja tacka linije</param>
        /// <param name="numIter">broj iteracija kroz koji ce se obavljati uglacavanje sa definisanim filterom (default 1)</param>
        public static Point[] GetContourPoints(Mat srcImg, out Point[] startEndContourPoints)
        {
            List<Point> startEndPoints = new List<Point>();
            List<Point> contourPoints = new List<Point>();
            byte[,] byteImg = MatToByte2D(srcImg);

            for (int i = 0; i < srcImg.Rows; i++)
            {
                for (int j = 0; j < srcImg.Cols; j++)
                {
                    //if(srcImg.Get<int>(i, j) > 0)
                    if (byteImg[i, j] != 0)
                    {
                        contourPoints.Add(new Point(j, i));

                        int cntWhiteNeighbors = 0;
                        if (i > 0)
                        {
                            if (j > 0)
                            {
                                if (byteImg[i, j] == byteImg[i - 1, j - 1])
                                {
                                    cntWhiteNeighbors += 1;
                                }
                            }
                            if (byteImg[i, j] == byteImg[i - 1, j])
                            {
                                cntWhiteNeighbors += 1;
                            }
                            if (j < srcImg.Cols - 1)
                            {
                                if (byteImg[i, j] == byteImg[i - 1, j + 1])
                                {
                                    cntWhiteNeighbors += 1;
                                }
                            }
                        }
                        if (j > 0)
                        {
                            if (byteImg[i, j] == byteImg[i, j - 1])
                            {
                                cntWhiteNeighbors += 1;
                            }
                        }
                        if (j < srcImg.Cols - 1)
                        {
                            if (byteImg[i, j] == byteImg[i, j + 1])
                            {
                                cntWhiteNeighbors += 1;
                            }
                        }
                        if (i < srcImg.Rows - 1)
                        {
                            if (j > 0)
                            {
                                if (byteImg[i, j] == byteImg[i + 1, j - 1])
                                {
                                    cntWhiteNeighbors += 1;
                                }
                            }
                            if (byteImg[i, j] == byteImg[i + 1, j])
                            {
                                cntWhiteNeighbors += 1;
                            }
                            if (j < srcImg.Cols - 1)
                            {
                                if (byteImg[i, j] == byteImg[i + 1, j + 1])
                                {
                                    cntWhiteNeighbors += 1;
                                }
                            }
                        }
                        if (cntWhiteNeighbors == 1)
                        {
                            startEndPoints.Add(new Point(j, i));
                        }
                    }
                }
            }
            startEndContourPoints = startEndPoints.ToArray();

            return contourPoints.ToArray();
        }

        /// <summary>
        /// Sluzi za sortiranje tacaka srednje linije profila tako da budu poredane od pocetne do zavrsne tacke
        /// </summary>
        /// <param name="cPoints">niz ispreturanih svih tacaka linije</param>
        /// <param name="startEndContourPoints">niz koji ima 2 clana: startEndContourPoints[0] - pocetna tacka, startEndContourPoints[1] - zavrsna tacka</param>
        public static Point[] GetSortedContourPoints(Point[] cPoints, Point[] startEndContourPoints)
        {

            List<Point> contourPoints = cPoints.Cast<Point>().ToList();  //cast-ovanje iz niza u listu
            List<Point> sortedPoints = new List<Point>();  // lista u kojoj ce biti sortirane tacke
            sortedPoints.Add(startEndContourPoints[0]); //dodajemo pocetnu tacku na pocetak niza sortiranih tacaka


            List<Point> contourPointsD = new List<Point>(contourPoints);  //lista koja ce na pocetku biti ista kao contourPoints, a posle se izbacuju iz nje oni 
                                                                          //elementi koji se 

            for (int n = 1; n < contourPoints.Count; n++)  // krecemo od 1 jer smo vec dodali 1 element u listu sortiranih
            {
                double minDistance = 10000; //neki veliki broj - 10000 je vise nego dovoljno u ovom slucaju
                int idxMinDist = 0;
                for (int i = 0; i < contourPointsD.Count; i++) //prolazimo kroz sve one tacke koji nisu u listi sortiranih
                {
                    if (sortedPoints.Last().DistanceTo(contourPointsD[i]) < minDistance)  //provjeravamo koja od preostalih tacaka ima najmanju udaljenost od posledenje 
                                                                                          // tacke u listi sortiranih
                    {
                        minDistance = sortedPoints.Last().DistanceTo(contourPointsD[i]);
                        idxMinDist = i;
                    }
                }
                sortedPoints.Add(contourPointsD[idxMinDist]);  // u sortirane dodajemo onu tacku sa najmanjom udaljenoscu
                contourPointsD.RemoveAt(idxMinDist); //iz privremene liste uklanjamo onu tacku koju smo dodali u sortirane
            }

            return sortedPoints.ToArray(); //cast u obican niz
        }


        public static Point2f[] PointArrToPoint2fArr(Point[] arr)
        {
            Point2f[] retArr = new Point2f[arr.Length];
            for (int i = 0; i < arr.Length; i++)
            {
                retArr[i] = new Point2f((float)arr[i].X, (float)arr[i].Y);
            }
            return retArr;
        }

        /// <summary>
        /// Usrednjavanje tacaka (x,y)
        /// </summary>
        /// <param name="srcArr">ulazni niz Point2f[]</param>
        /// <param name="filterSize">duzina filtera (mora biti neparan broj! - ako nije, funkcija vraca ulazni niz)</param>
        /// <param name="numIter">broj iteracija kroz koji ce se obavljati usrednjavanje sa definisanim filterom (default 1)</param>
        public static Point2f[] MovingAverageSmooth1DPointArr(Point2f[] srcArr, int filterSize, int numIter = 1)
        {
            Point2f[] dstArr = new Point2f[srcArr.Length];
            srcArr.CopyTo(dstArr, 0);

            if (filterSize % 2 != 0)
            {
                for (int n = 0; n < numIter; n++)
                {
                    for (int i = 0; i < dstArr.Length; i++)
                    {
                        if (i == 0 || i == (dstArr.Length - 1))
                        {
                            continue;
                        }
                        if (i < filterSize / 2)
                        {
                            int toLeft = i;
                            float sumX = 0.0f;
                            float sumY = 0.0f;
                            for (int k = 1; k <= toLeft; k++)
                            {
                                sumX += dstArr[i - k].X;
                                sumY += dstArr[i - k].Y;
                            }
                            sumX += dstArr[i].X;
                            sumY += dstArr[i].Y;
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sumX += dstArr[i + k].X;
                                sumY += dstArr[i + k].Y;
                            }
                            dstArr[i] = new Point2f(sumX / (float)(filterSize / 2 + toLeft + 1.0f), sumY / (float)(filterSize / 2 + toLeft + 1));
                        }
                        else if (i > (dstArr.Length - filterSize / 2 - 1))
                        {
                            int toRight = dstArr.Length - i - 1;
                            float sumX = 0.0f;
                            float sumY = 0.0f;
                            for (int k = 1; k <= toRight; k++)
                            {
                                sumX += dstArr[i + k].X;
                                sumY += dstArr[i + k].Y;
                            }
                            sumX += dstArr[i].X;
                            sumY += dstArr[i].Y;
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sumX += dstArr[i - k].X;
                                sumY += dstArr[i - k].Y;
                            }
                            dstArr[i] = new Point2f(sumX / (float)(filterSize / 2 + toRight + 1), sumY / (float)(filterSize / 2 + toRight + 1));
                        }
                        else
                        {
                            float sumX = 0.0f;
                            float sumY = 0.0f;
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sumX += dstArr[i - k].X;
                                sumY += dstArr[i - k].Y;
                            }
                            sumX += dstArr[i].X;
                            sumY += dstArr[i].Y;
                            for (int k = 1; k <= filterSize / 2; k++)
                            {
                                sumX += dstArr[i + k].X;
                                sumY += dstArr[i + k].Y;
                            }
                            dstArr[i] = new Point2f(sumX / (float)filterSize, sumY / (float)filterSize);
                        }
                    }
                }
                return dstArr;
            }
            else
            {
                return dstArr;
            }
        }


        /// <summary>
        /// Uklanja ravan dio profila sa slike i vraca sliku na kojoj je luk bez ravnog dijela
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        /// <param name="angleBetweenLines">izlazni parametar - ugao izmedju pravih koje cine ravne dijelove konture profila</param>
        public static Mat FindFlatParts(Mat srcImg, out float angleBetweenLines)
        {
            var watch = new System.Diagnostics.Stopwatch();

            Mat tempImg = new Mat();
            srcImg.CopyTo(tempImg);

            Cv2.CopyMakeBorder(tempImg, tempImg, 30, 30, 30, 30, BorderTypes.Constant, 0);

            Mat elementDilate = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(7, 7), new Point(3, 3));
            Cv2.Dilate(tempImg, tempImg, elementDilate, null, 2);

            Mat blackHatImg = new Mat();
            Mat sumMat = new Mat();

            Cv2.MorphologyEx(tempImg, blackHatImg, MorphTypes.BlackHat, Cv2.GetStructuringElement(MorphShapes.Rect, new Size(15, 15), new Point(7, 7)));
            sumMat = blackHatImg + tempImg;
            sumMat.CopyTo(tempImg);

            Cv2.CopyMakeBorder(tempImg, tempImg, 1, 1, 1, 1, BorderTypes.Constant, 0);

            //Cv2.ImShow("dilate to flat", tempImg);

            OpenCvSharp.XImgProc.CvXImgProc.Thinning(tempImg, tempImg, OpenCvSharp.XImgProc.ThinningTypes.ZHANGSUEN);

            tempImg = ToBlackWhiteImage(tempImg);
            watch.Start();
            Point[] startEndContourPoints;
            Point[] contourPoints = GetContourPoints(tempImg, out startEndContourPoints);
            Point[] mainContourPoints = GetSortedContourPoints(contourPoints, startEndContourPoints);
            watch.Stop();
            //Console.WriteLine($"Execution Time - get points + sort: {watch.ElapsedMilliseconds} ms");

            Point2f[] mainContourPointsAvgPool = PointArrToPoint2fArr(mainContourPoints);
            
            mainContourPointsAvgPool = MovingAverageSmooth1DPointArr(mainContourPointsAvgPool, 5, 1);
            mainContourPointsAvgPool = MovingAverageSmooth1DPointArr(mainContourPointsAvgPool, 3, 1);

            // Nalazimo pribliznu duzinu da bismo posle podesavali toleranciju u zavisnosti od priblizne duzine ravnog dijela
            float tol = 4.0f;

            int startIdx = 2; //ako je startIdx = m, krece od m plus prvog elementa u nizu
            int currIdx = (mainContourPoints.Length - 1) / 2;

            Point2f startPoint = mainContourPointsAvgPool[startIdx];
            float x1 = startPoint.X;
            float y1 = startPoint.Y;

            bool flatFound = false;

            bool foundPartOfFlat = false;
            int cnt = 0;
            while (!flatFound)
            {
                cnt += 1;
                Point2f currPoint = mainContourPointsAvgPool[currIdx];
                float x2 = currPoint.X;
                float y2 = currPoint.Y;

                //Cv2.Line(tempImg, new Point(startPoint.X, startPoint.Y), new Point(currPoint.X, currPoint.Y), 100);

                // prava je u obliku a*x + b*y + c = 0
                float a = y1 - y2;
                float b = x2 - x1;
                float c = (x1 - x2) * y1 + (y2 - y1) * x1;


                bool allSatisfied = false;
                int tempMaxBad = 0;

                for (int i = startIdx; i <= currIdx; i++)
                {

                    if ((Math.Abs(a * mainContourPointsAvgPool[i].X + b * mainContourPointsAvgPool[i].Y + c) / Math.Sqrt(Math.Pow(a, 2) + Math.Pow(b, 2))) > tol)
                    {
                        tempMaxBad += 1;
                    }
                    else
                    {
                        tempMaxBad = 0;
                    }
                    if (tempMaxBad >= 5)
                    {
                        break;
                    }

                }
                if (tempMaxBad < 5)
                {
                    allSatisfied = true;
                    foundPartOfFlat = true;
                }
                else
                {
                    if (foundPartOfFlat)
                    {
                        flatFound = true;
                        continue;
                    }
                }

                if (allSatisfied)
                {
                    currIdx += 1;
                }
                else
                {
                    currIdx /= 2;
                }
            }


            int idxToScale = currIdx * 1;


            tol = 0.017f * idxToScale;

            startIdx = 5; //ako je startIdx = m, krece od m plus prvog elementa u nizu
            currIdx = (mainContourPoints.Length - 1) / 2;

            startPoint = mainContourPointsAvgPool[startIdx];
            x1 = startPoint.X;
            y1 = startPoint.Y;

            flatFound = false;

            foundPartOfFlat = false;
            cnt = 0;
            while (!flatFound)
            {
                //break;
                cnt += 1;
                Point2f currPoint = mainContourPointsAvgPool[currIdx];
                float x2 = currPoint.X;
                float y2 = currPoint.Y;

                //Cv2.Line(tempImg, new Point(startPoint.X, startPoint.Y), new Point(currPoint.X, currPoint.Y), 100);

                // prava je u obliku a*x + b*y + c = 0
                float a = y1 - y2;
                float b = x2 - x1;
                float c = (x1 - x2) * y1 + (y2 - y1) * x1;


                bool allSatisfied = false;
                int tempMaxBad = 0;

                for (int i = startIdx; i <= currIdx; i++)
                {
                    if ((Math.Abs(a * mainContourPointsAvgPool[i].X + b * mainContourPointsAvgPool[i].Y + c) / Math.Sqrt(Math.Pow(a, 2) + Math.Pow(b, 2))) > tol)
                    {
                        tempMaxBad += 1;
                    }
                    else
                    {
                        tempMaxBad = 0;
                    }
                    if (tempMaxBad >= (0.06f * idxToScale))
                    {
                        break;
                    }

                }
                if (tempMaxBad < (0.06f * idxToScale))
                {
                    allSatisfied = true;
                    foundPartOfFlat = true;
                }
                else
                {
                    if (foundPartOfFlat)
                    {
                        flatFound = true;
                        continue;
                    }
                }

                if (allSatisfied)
                {
                    currIdx += 1;
                }
                else
                {
                    currIdx /= 2;
                }
            }

            int startX1 = (int)startPoint.X;
            int startY1 = (int)startPoint.Y;
            int endFlatX1 = (int)mainContourPoints[currIdx - (int)Math.Floor(0.08f * idxToScale)].X;
            int endFlatY1 = (int)mainContourPoints[currIdx - (int)Math.Floor(0.08f * idxToScale)].Y;

            Point2f endPoint = mainContourPointsAvgPool[mainContourPointsAvgPool.Length - startIdx - 1];
            int startX2 = (int)endPoint.X;
            int startY2 = (int)endPoint.Y;
            int endFlatX2 = (int)mainContourPoints[mainContourPoints.Length - (currIdx - (int)Math.Floor(0.08f * idxToScale)) - 1].X;
            int endFlatY2 = (int)mainContourPoints[mainContourPoints.Length - (currIdx - (int)Math.Floor(0.08f * idxToScale)) - 1].Y;


            Cv2.Circle(tempImg, startX1, startY1, 2, 200, -1);
            Cv2.Circle(tempImg, endFlatX1, endFlatY1, 2, 100, -1);

            //Cv2.ImShow("linija i tacka for flat", tempImg);

            float k11 = ((float)startY1 - (float)endFlatY1) / ((float)startX1 - (float)endFlatX1);
            float k22 = ((float)startY2 - (float)endFlatY2) / ((float)startX2 - (float)endFlatX2);

            angleBetweenLines = (float)Math.Atan((k22 - k11) / (1 + k11 * k22)) * 180.0f / (float)Math.PI;

            //Console.WriteLine("Ugao izmedju pravih (ukupan ugao isjecka) = " + angleBetweenLines.ToString() + "\n\n");

            Mat retImg = new Mat();
            if (Math.Abs(endFlatX1 - startY1) < 5.0)
            {
                retImg = ScaleImageToOnlyContour(srcImg.RowRange((int)mainContourPoints[currIdx - (int)Math.Floor(0.08f * idxToScale)].Y - 31, srcImg.Rows));

            }
            else
            {
                x1 = (float)startX1;
                y1 = (float)startY1;
                float x2 = (float)endFlatX1;
                float y2 = (float)endFlatY1;

                float k1 = (y2 - y1) / (x2 - x1);
                float n1 = -k1 * x1 + y1;

                float k2 = -1.0f / k1; //uslov normalnosti
                float n2 = -k2 * x2 + y2;

                int x0 = (int)-(n2 / k2);
                int y0 = (int)Math.Abs(n2);

                Point point0L;
                Point point1L;
                Point point2L;

                Point point0R;
                Point point1R;
                Point point2R;
                if (x1 < tempImg.Cols / 2)
                {
                    point0L = new Point(0, 0);
                    point1L = new Point(x0, 0);
                    point2L = new Point(0, y0);

                    point0R = new Point(tempImg.Cols, 0);
                    point1R = new Point(tempImg.Cols - x0, 0);
                    point2R = new Point(tempImg.Cols, y0);
                }
                else
                {
                    point0L = new Point(0, 0);
                    point1L = new Point(tempImg.Cols - x0, 0);
                    point2L = new Point(0, y0);

                    point0R = new Point(tempImg.Cols, 0);
                    point1R = new Point(x0, 0);
                    point2R = new Point(tempImg.Cols, y0);
                }

                Point[][] pollys = new Point[2][];
                Point[] pollyPointsL = new Point[] { point0L, point1L, point2L };
                Point[] pollyPointsR = new Point[] { point0R, point1R, point2R };
                pollys[0] = pollyPointsL;
                pollys[1] = pollyPointsR;

                Cv2.FillPoly(tempImg, pollys, 100);

                //Cv2.ImShow("srcImg odsjecen ravan dio trougao", tempImg);

                Cv2.CopyMakeBorder(srcImg, retImg, 31, 31, 31, 31, BorderTypes.Constant, 0);
                Cv2.FillPoly(retImg, pollys, 0);
                //Cv2.ImShow("srcImg odsjecen ravan dio trougao 2", retImg);
                retImg = ScaleImageToOnlyContour(retImg);

            }
            
            return retImg;
        }


        /// <summary>
        /// Prepoznaje orijentaciju profila na ulaynoj slici i vraca sliku na kojoj je profil okrenut kracima ka gore
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        /// <param name="angle">izlazni parametar - ugao za koji se rotira slika</param>
        public static Mat CorrectRotation(Mat srcImg, out float angle)
        {
            Mat tempImg = new Mat();
            srcImg.CopyTo(tempImg);

            Cv2.CopyMakeBorder(tempImg, tempImg, 40, 40, 40, 40, BorderTypes.Constant, 0);

            Mat elementDilate = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(3, 3), new Point(1, 1));
            Cv2.Dilate(tempImg, tempImg, elementDilate, null, 7);

            //ShowImage("slk dil", tempImg);

            Cv2.CopyMakeBorder(tempImg, tempImg, 1, 1, 1, 1, BorderTypes.Constant, 0);

            OpenCvSharp.XImgProc.CvXImgProc.Thinning(tempImg, tempImg, 0);

            tempImg = ToBlackWhiteImage(tempImg);
            Point[] startEndContourPoints;
            Point[] contourPoints = GetContourPoints(tempImg, out startEndContourPoints);

            float x1 = startEndContourPoints[0].X;
            float y1 = startEndContourPoints[0].Y;

            float x2 = startEndContourPoints[1].X;
            float y2 = startEndContourPoints[1].Y;
            
            if (x1 == x2)
            {
                angle = 90.0f;
            }
            else if (y1 == y2)
            {
                angle = 0.0f;
            }
            else
            {
                angle = (float)Math.Atan((y1 - y2) / (x1 - x2)) * 180.0f / (float)Math.PI;
            }

            srcImg.CopyTo(tempImg);
            tempImg = ScaleImageToOnlyContour(MatRotate(tempImg, angle));

            return tempImg;

        }

        /// <summary>
        /// Okrece profil kracima ka gore, u slucaju da je okrenut kracima ka dole 
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        /// <param name="angle">izlazni parametar - ugao za koji se vrsi rotacija</param>
        public static Mat TurnUpwards(Mat srcImg, out float angle)
        {
            angle = 0.0f;

            Mat tmpImg = new Mat();
            srcImg.CopyTo(tmpImg);

            Cv2.CopyMakeBorder(tmpImg, tmpImg, 10, 10, 10, 10, BorderTypes.Constant, 0);
            OpenCvSharp.XImgProc.CvXImgProc.Thinning(tmpImg, tmpImg, 0);

            Mat upperHalf = UpperHalf(tmpImg);
            Mat lowerHalf = LowerHalf(tmpImg);

            Point[][] contours = null;
            HierarchyIndex[] hierarchy = null;

            Cv2.FindContours(upperHalf, out contours, out hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxNone);
            int upperHalfNumberOfContours = contours.Length;

            Cv2.FindContours(lowerHalf, out contours, out hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxNone);
            int lowerHalfNumberOfContours = contours.Length;

            if (lowerHalfNumberOfContours > upperHalfNumberOfContours)
            {
                angle += 180.0f;
                return ScaleImageToOnlyContour(MatRotate(srcImg, 180));
            }
            else
            {
                return srcImg;
            }
        }


        /// <summary>
        /// Spaja funkcionalnosti CorrectRotation i TurnUpwards funkcija i daje konacan rezultat
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        public static Mat CorrectRotationFinal(Mat srcImg)
        {
            float angle1;
            float angle2;
            float angleUp;
            Mat tempImg1 = CorrectRotation(srcImg, out angle1);
            Mat tempImgUp = TurnUpwards(tempImg1, out angleUp);

            return tempImgUp;

        }



    }
}
