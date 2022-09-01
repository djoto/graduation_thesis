using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.DepthBasics.TimingScanner
{
    using static TimingScanner.ScannerUtils;
    static class BendClassifier
    {
        /// <summary>
        /// Vrsi izdvajanje odbiraka spoljasnje ivice profila sa donje strane i radi njihovo usrednjavanje
        /// </summary>
        /// <param name="srcImg">ulazna slika sa profilom okrenutim kracima ka gore</param>
        public static float[] GetAverageSamplesArray(Mat srcImg)
        {
            int[] arr = GetArrayY(srcImg);

            //string strI = "";
            //for(int i = 0; i<arr.Length; i++)
            //{
            //    strI += arr[i].ToString() + " & ";
            //}
            //System.Windows.MessageBox.Show(strI);
            //File.WriteAllText("C:\\Users\\Djordje\\Desktop\\WriteTextI.txt", strI);

            float[] floatArr = IntToFloatArray(arr);

            float[] firstArray = floatArr.Take(floatArr.Length / 2).ToArray();
            float[] secondArray = floatArr.Skip(floatArr.Length / 2).ToArray();
            firstArray = MovingAverageSmooth1D(firstArray, 5);
            firstArray = MovingAverageSmooth1D(firstArray, 3);
            secondArray = MovingAverageSmooth1D(secondArray, 5);
            secondArray = MovingAverageSmooth1D(secondArray, 3);
            firstArray.CopyTo(floatArr, 0);
            secondArray.CopyTo(floatArr, firstArray.Length);

            //string strF = "";
            //for (int i = 0; i < floatArr.Length; i++)
            //{
            //    strF += floatArr[i].ToString() + " & ";
            //}
            //System.Windows.MessageBox.Show(strF);
            //File.WriteAllText("C:\\Users\\Djordje\\Desktop\\WriteTextF.txt", strF);

            return floatArr;
        }

        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada odbircima odredjenog dijela kruznice
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="r">poluprecnik kruznice kojoj se provjerava pripadnost ulaznih odbiraka</param>
        /// <param name="errFromCircle">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsSection(float[] samplesArray, float r, float errFromCircle = 3.0f, int allowedBadSequence = 10)
        {
            int fullLength = samplesArray.Length;
            int halfLength = samplesArray.Length / 2;

            int cntBad = 0;
            int maxBadSequence = 0;
            for (int i = 0; i < halfLength; i++)
            {
                //if (Math.Abs(Math.Pow(r - floatArr[i], 2) + Math.Pow(halfLength - i + 1, 2) - Math.Pow(r, 2)) > (errFromCircle * Math.Pow(r, 2)))
                if (Math.Abs(Math.Sqrt(Math.Pow(r - samplesArray[i], 2) + Math.Pow(halfLength - i + 1, 2)) - r) > errFromCircle)
                {
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad * 1;
                }
            }
            for (int i = 0; i < (fullLength - halfLength); i++)
            {
                //if (Math.Abs(Math.Pow(r - floatArr[halfLength + i], 2) + Math.Pow(i + 1, 2) - Math.Pow(r, 2)) > (errFromCircle * Math.Pow(r, 2)))
                if (Math.Abs(Math.Sqrt(Math.Pow(r - samplesArray[halfLength + i], 2) + Math.Pow(i + 1, 2)) - r) > errFromCircle)
                {
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad;
                }
            }
            //System.Windows.MessageBox.Show("sssss = " + s);
            if (maxBadSequence <= allowedBadSequence)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada odbircima n luka
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="srcImg">ulazna slika</param>
        public static bool IsArcN(Mat srcImg, float[] samplesArray)
        {
            bool isB4;

            float[] samplesArrayHalf = samplesArray.Take(samplesArray.Length / 2).ToArray();

            int c = 0;

            while (true)
            {
                if (samplesArrayHalf[samplesArrayHalf.Length - 1 - c] > 8.0f)
                {
                    isB4 = false;
                    break;
                }
                else
                {
                    Mat tempImgL = srcImg.ColRange(0, srcImg.Cols / 2 - c);
                    Mat tempImgR = srcImg.ColRange(srcImg.Cols / 2 + c, srcImg.Cols);
                    Mat tempImg = new Mat();
                    Cv2.HConcat(tempImgL, tempImgR, tempImg);
                    //ShowImage("za n luk", tempImg);
                    tempImg = ScaledResize(tempImg, srcImg.Cols);

                    float[] samplesArrayTemp = GetAverageSamplesArray(tempImg);

                    if (IsRegularArc(tempImg, samplesArrayTemp, 3.3f, 15))
                    {
                        isB4 = true;
                        break;
                    }
                    c += 1;
                }
            }
            if (isB4)
            {
                if (!(c >= (samplesArrayHalf.Length - c) / 4))
                {
                    isB4 = false;
                }
            }
            return isB4;
        }


        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada horizontalnoj ili vertikalnoj elipsi
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        public static bool[] CheckHorizontalVerticalEllipse(float[] samplesArray)
        {
            bool[] retArr = new bool[2] { false, false };

            float percentOfRadius = 0.75f;
            float r = samplesArray.Length / 2;
            float d = (float)Math.Sqrt(Math.Pow(r, 2) - Math.Pow(percentOfRadius * r, 2));
            float hk = r - d;
            int idxToCheck = (int)Math.Floor((1.0f - percentOfRadius) * r);
            float h = samplesArray[idxToCheck];

            //Console.WriteLine("hk = " + hk.ToString() + ", h = " + h.ToString());

            if (h < hk)
            {
                retArr[1] = false;  // nije vertikalna elipsa

                float a = samplesArray.Length / 2.0f;

                float percentOfRad = 0.75f;
                int idx = (int)Math.Floor((1.0f - percentOfRad) * a);

                int cnt = 0;
                float b = 0.0f;
                for (int i = idx; i < samplesArray.Length / 2 - idx; i++)
                {
                    cnt++;

                    float x = a - (float)i;
                    float n = samplesArray[i];

                    float b1 = (2.0f * (float)Math.Pow(a, 2) * n + (float)Math.Sqrt(4 * (float)Math.Pow(a, 4) * (float)Math.Pow(n, 2) - 4 * (float)Math.Pow(x, 2) * (float)Math.Pow(a, 2) * (float)Math.Pow(n, 2))) / (2.0f * (float)Math.Pow(x, 2));
                    float b2 = (2.0f * (float)Math.Pow(a, 2) * n - (float)Math.Sqrt(4 * (float)Math.Pow(a, 4) * (float)Math.Pow(n, 2) - 4 * (float)Math.Pow(x, 2) * (float)Math.Pow(a, 2) * (float)Math.Pow(n, 2))) / (2.0f * (float)Math.Pow(x, 2));
                    b += b1;
                    //Console.WriteLine("a = " + a.ToString());
                    //Console.WriteLine("b1 = " + b1.ToString());
                    //Console.WriteLine("b2 = " + b2.ToString() + "\n");
                }
                //Console.WriteLine("sr vr b = " + (b / (float)cnt).ToString());
                b = b / (float)cnt;
                if (a <= b)
                {
                    retArr[0] = false;
                }
                else
                {
                    retArr[0] = IsHorizontalEllipse(samplesArray, a, b, 0.14f, 20);
                }
                //Console.WriteLine("rt 0 = " + retArr[0].ToString());
            }
            else if (h > hk)
            {

                retArr[0] = false; // nije horizontalna elipsa

                if (samplesArray[0] < samplesArray.Length / 2)
                {
                    retArr[1] = false;  // vertikalna elipsa
                }
                else
                {
                    float b = samplesArray.Length / 2.0f;
                    float percentOfRad = 0.75f;
                    int idx = (int)Math.Floor((1.0f - percentOfRad) * b);

                    int cnt = 0;
                    float a = 0.0f;
                    for (int i = idx; i < samplesArray.Length / 2 - idx; i++)
                    {
                        cnt++;

                        float x = b - (float)i;
                        float m = samplesArray[i];

                        float a1 = (2.0f * (float)Math.Pow(b, 2) * m + (float)Math.Sqrt(4 * (float)Math.Pow(b, 4) * (float)Math.Pow(m, 2) - 4 * (float)Math.Pow(x, 2) * (float)Math.Pow(b, 2) * (float)Math.Pow(m, 2))) / (2.0f * (float)Math.Pow(x, 2));
                        float a2 = (2.0f * (float)Math.Pow(b, 2) * m - (float)Math.Sqrt(4 * (float)Math.Pow(b, 4) * (float)Math.Pow(m, 2) - 4 * (float)Math.Pow(x, 2) * (float)Math.Pow(b, 2) * (float)Math.Pow(m, 2))) / (2.0f * (float)Math.Pow(x, 2));
                        a += a1;
                        //Console.WriteLine("b = " + b.ToString());
                        //Console.WriteLine("a1 = " + a1.ToString());
                        //Console.WriteLine("a2 = " + a2.ToString() + "\n");
                    }
                    //Console.WriteLine("sr vr a = " + (a / (float)cnt).ToString());
                    a = a / (float)cnt;
                    if (a <= b)
                    {
                        retArr[1] = false;
                    }
                    else
                    {
                        retArr[1] = IsVerticalEllipse(samplesArray, a, b, 0.14f, 20);
                    }

                }
                //Console.WriteLine("rt 0 = " + retArr[0].ToString());
            }

            return retArr;
        }


        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada horizontalnoj elipsi
        /// </summary> 
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="a">duzina vece poluose elipse</param>
        /// <param name="b">duzina manje poluose elipse</param>
        /// <param name="tol">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsHorizontalEllipse(float[] samplesArray, float a, float b, float tol, int allowedBadSequence)
        {
            float[] firstArray = samplesArray.Take(samplesArray.Length / 2).ToArray();
            float[] secondArray = samplesArray.Skip(samplesArray.Length / 2).ToArray();

            int cntBad = 0;
            int maxBadSequence = 0;

            //Console.WriteLine("tolerancija = " + (tol * Math.Pow(a, 2) * Math.Pow(b, 2)).ToString() + "\n\n");

            for (int i = 0; i < firstArray.Length; i++)
            {
                float x = -a + (float)i;
                float y = -b + samplesArray[i];
                if (Math.Abs(Math.Pow(b, 2) * Math.Pow(x, 2) + Math.Pow(a, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)) > (tol * Math.Pow(a, 2) * Math.Pow(b, 2)))
                {
                    //Console.WriteLine("s = " + Math.Abs(Math.Pow(b, 2) * Math.Pow(x, 2) + Math.Pow(a, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)).ToString());
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad * 1;
                }
            }
            //Console.WriteLine("\n\n");
            for (int i = 0; i < secondArray.Length; i++)
            {
                float x = (float)i;
                float y = -b + samplesArray[firstArray.Length + i];
                if (Math.Abs(Math.Pow(b, 2) * Math.Pow(x, 2) + Math.Pow(a, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)) > (tol * Math.Pow(a, 2) * Math.Pow(b, 2)))
                {
                    //Console.WriteLine("s = " + Math.Abs(Math.Pow(b, 2) * Math.Pow(x, 2) + Math.Pow(a, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)).ToString());
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad * 1;
                }
            }
            //Console.WriteLine("max bad = " + maxBadSequence.ToString());
            if (maxBadSequence <= allowedBadSequence)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada vertikalnoj elipsi
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="a">duzina vece poluose elipse</param>
        /// <param name="b">duzina manje poluose elipse</param>
        /// <param name="tol">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsVerticalEllipse(float[] samplesArray, float a, float b, float tol, int allowedBadSequence)
        {
            float[] firstArray = samplesArray.Take(samplesArray.Length / 2).ToArray();
            float[] secondArray = samplesArray.Skip(samplesArray.Length / 2).ToArray();

            int cntBad = 0;
            int maxBadSequence = 0;

            //Console.WriteLine("tolerancija = " + (tol * Math.Pow(a, 2) * Math.Pow(b, 2)).ToString() + "\n\n");

            for (int i = 0; i < firstArray.Length; i++)
            {
                float x = -b + (float)i;
                float y = -a + samplesArray[i];
                if (Math.Abs(Math.Pow(a, 2) * Math.Pow(x, 2) + Math.Pow(b, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)) > (tol * Math.Pow(a, 2) * Math.Pow(b, 2)))
                {
                    //Console.WriteLine("sv = " + Math.Abs(Math.Pow(a, 2) * Math.Pow(x, 2) + Math.Pow(b, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)).ToString());
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad * 1;
                }
            }
            //Console.WriteLine("\n\n");
            for (int i = 0; i < secondArray.Length; i++)
            {
                float x = (float)i;
                float y = -a + samplesArray[firstArray.Length + i];
                if (Math.Abs(Math.Pow(a, 2) * Math.Pow(x, 2) + Math.Pow(b, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)) > (tol * Math.Pow(a, 2) * Math.Pow(b, 2)))
                {
                    //Console.WriteLine("sv = " + Math.Abs(Math.Pow(a, 2) * Math.Pow(x, 2) + Math.Pow(b, 2) * Math.Pow(y, 2) - Math.Pow(a, 2) * Math.Pow(b, 2)).ToString());
                    cntBad += 1;
                }
                else
                {
                    cntBad = 0;
                }
                if (cntBad > maxBadSequence)
                {
                    maxBadSequence = cntBad * 1;
                }
            }
            //Console.WriteLine("max bad = " + maxBadSequence.ToString());
            if (maxBadSequence <= allowedBadSequence)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada polukrugu - pravilnom luku
        /// </summary>
        /// <param name="srcImg">ulazna slika</param>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="errFromCircle">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsRegularArc(Mat srcImg, float[] samplesArray, float errFromCircle = 3.5f, int allowedBadSequence = 15)
        {
            float r = 0.0f;

            if ((srcImg.Cols / 2 - srcImg.Rows) > 5)
            {
                return false;
            }
            else
            {
                if (srcImg.Rows > (srcImg.Cols / 2))
                {
                    float[] samplesArrScaled = GetAverageSamplesArray(srcImg.RowRange(srcImg.Rows - srcImg.Cols / 2, srcImg.Rows));
                    r = (float)samplesArray.Length / 2.0f;
                    return IsSection(samplesArrScaled, r, errFromCircle, allowedBadSequence);
                }
                r = (float)srcImg.Cols / 2.0f;
                return IsSection(samplesArray, r, errFromCircle, allowedBadSequence);
            }
        }


        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada L luku (zaklapa 90 stepeni)
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="errFromCircle">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsArcL(float[] samplesArray, float errFromCircle = 3.3f, int allowedBadSequence = 10)
        {
            float h = (samplesArray[0] + samplesArray[samplesArray.Length - 1]) / 2.0f;
            float r = 2.0f * h + h * (float)Math.Sqrt(2);

            return IsSection(samplesArray, r, errFromCircle, allowedBadSequence);
        }



        /// <summary>
        /// Provjerava da li ulazni niz usrednjenih odbiraka pripada kruznom isjecku koji zaklapa manje od 180 stepeni, pritom razlicito od 90 stepeni
        /// </summary>
        /// <param name="samplesArray">ulazni niz usrednjenih odbiraka</param>
        /// <param name="errFromCircle">granicno odstupanje jednog odbirka</param>
        /// <param name="allowedBadSequence">maksimalna dozvoljena sekvenca odbiraka koji odstupaju vise od granicne vrijednosti</param>
        public static bool IsAnySection(float[] samplesArray, float errFromCircle = 3.3f, int allowedBadSequence = 10)
        {
            float a = samplesArray.Length / 2;
            float b = (samplesArray[0] + samplesArray[samplesArray.Length - 1]) / 2.0f;
            float r = (float)(Math.Pow(a, 2) + Math.Pow(b, 2)) / (2 * b);

            return IsSection(samplesArray, r, errFromCircle, allowedBadSequence);
        }


        /// <summary>
        /// Vraca oznaku klase profila (0, 1, 2, 3, 4, 5, 6, 2.1, 3.1) u koju je klasifikovan profil sa ulazne slike
        /// </summary>
        /// <param name="srcImg">ulazna slika na kojoj je kontura profila za klasifikaciju okrenuta kracima ka gore</param>
        public static string Classify(Mat srcImg)
        {
            Mat beforeResizeImg = new Mat();
            srcImg.CopyTo(beforeResizeImg);

            srcImg = ScaledResize(srcImg, 300);
            //ShowImage("skalirane dimenzije", srcImg);

            string messageResult;

            float[] samplesArray = GetAverageSamplesArray(srcImg);

            if (IsRegularArc(srcImg, samplesArray, 3.5f, 15))
            {
                //messageResult = "Pravilan luk (180 stepeni)";
                messageResult = "1";
            }
            else if (IsArcL(samplesArray, 3.3f, 15))
            {
                //messageResult = "L arc (90 stepeni) (br)";
                messageResult = "2";
            }
            else if (IsAnySection(samplesArray, 3.3f, 15))
            {
                //messageResult = "isjecak (br)";
                messageResult = "3";
            }
            else if (IsArcN(srcImg, samplesArray))
            {
                //messageResult = "H luk - klasa4";
                messageResult = "4";
            }
            else
            {
                bool[] arr_H_V = CheckHorizontalVerticalEllipse(samplesArray);
                if (arr_H_V[0])
                {
                    //messageResult = "Horizontalna elipsa";
                    messageResult = "5";
                }
                else if (arr_H_V[1])
                {
                    //messageResult = "Vertikalna elipsa";
                    messageResult = "6";
                }
                else
                {
                    float angleBetweenLines;
                    srcImg = FindFlatParts(beforeResizeImg, out angleBetweenLines);  //otklanja ravne dijelove

                    srcImg = ScaledResize(srcImg, 300);
                    //ShowImage("skalirane dimenzije 2", srcImg);

                    samplesArray = GetAverageSamplesArray(srcImg);

                    if (IsArcL(samplesArray, 3.3f, 15))
                    {
                        //messageResult = "L luk (90 stepeni) (sr) - ukupan izmjeren ugao ~ " + Math.Abs(angleBetweenLines).ToString() + " stepeni";
                        messageResult = "2.1";
                    }
                    else if (IsAnySection(samplesArray, 3.3f, 15))
                    {
                        //messageResult = "isjecak (sr) - ukupan izmjeren ugao ~ " + Math.Abs(angleBetweenLines).ToString() + " stepeni";
                        messageResult = "3.1";
                    }
                    else
                    {
                        //messageResult = "Nepoznati oblik";
                        messageResult = "0";
                    }
                }
            }
            return messageResult;
        }




    }
}
