using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Microsoft.Samples.Kinect.DepthBasics.TimingScanner
{
    /// <summary>
    /// Interaction logic for ResultWindow.xaml
    /// </summary>
    public partial class ResultWindow : Window
    {
        //string strDetails = "Probabilities:\n\n";
        string strDetails = "VJEROVATNOĆE:\n\n";
        public ResultWindow(string[] resultArray)
        {

            int[] cntArr = new int[7] { 0, 0, 0, 0, 0, 0, 0 };

            for (int i = 0; i < resultArray.Length; i++)
            {
                if (resultArray[i] == "1")
                {
                    cntArr[1] += 1;
                }
                else if (resultArray[i] == "2" || resultArray[i] == "2.1")
                {
                    cntArr[2] += 1;
                }
                else if (resultArray[i] == "3" || resultArray[i] == "3.1")
                {
                    cntArr[3] += 1;
                }
                else if (resultArray[i] == "4")
                {
                    cntArr[4] += 1;
                }
                else if (resultArray[i] == "5")
                {
                    cntArr[5] += 1;
                }
                else if (resultArray[i] == "6")
                {
                    cntArr[6] += 1;
                }
                else
                {
                    cntArr[0] += 1;
                }

            }

            float[] percentResult = new float[7] { (float)cntArr[0] / (float)resultArray.Length * 100, (float)cntArr[1] / (float)resultArray.Length * 100, (float)cntArr[2] / (float)resultArray.Length * 100, (float)cntArr[3] / (float)resultArray.Length * 100, (float)cntArr[4] / (float)resultArray.Length * 100, (float)cntArr[5] / (float)resultArray.Length * 100, (float)cntArr[6] / (float)resultArray.Length * 100 };

            //strDetails += "Regular arc (180 degrees) - " + percentResult[1].ToString() + "%\n";
            //strDetails += "L arc (90 degrees) - " + percentResult[2].ToString() + "%\n";
            //strDetails += "Arc section - " + percentResult[3].ToString() + "%\n";
            //strDetails += "Arc n - " + percentResult[4].ToString() + "%\n";
            //strDetails += "Horizontal ellipse - " + percentResult[5].ToString() + "%\n";
            //strDetails += "Vertical ellipse - " + percentResult[6].ToString() + "%\n";
            //strDetails += "Unknown shape - " + percentResult[0].ToString() + "%\n";

            strDetails += "Pravilan luk (180 stepeni) - " + percentResult[1].ToString() + "%\n";
            strDetails += "L luk (90 stepeni) - " + percentResult[2].ToString() + "%\n";
            strDetails += "Kružni isječak - " + percentResult[3].ToString() + "%\n";
            strDetails += "n luk - " + percentResult[4].ToString() + "%\n";
            strDetails += "Horizontalna elipsa - " + percentResult[5].ToString() + "%\n";
            strDetails += "Vertikalna elipsa - " + percentResult[6].ToString() + "%\n";
            strDetails += "Nepoznat oblik - " + percentResult[0].ToString() + "%\n";

            float maxPercent = percentResult[0];
            int idxMax = 0;
            for (int i = 1; i < percentResult.Length; i++)
            {
                if (percentResult[i] > maxPercent)
                {
                    maxPercent = percentResult[i];
                    idxMax = i;
                }
            }

            InitializeComponent();

            BitmapImage bitmap = new BitmapImage();

            if (idxMax == 0)
            {
                ResultText.Content = "Nepoznat oblik";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\unknown_type_img.jpeg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if(idxMax == 1)
            {
                ResultText.Content = "Pravilan luk (180 stepeni)";

                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type1_regular_arc.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if(idxMax == 2)
            {
                ResultText.Content = "L luk (90 stepeni)";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type2_L_arc.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if (idxMax == 3)
            {
                ResultText.Content = "Kruzni isjecak";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type3_section.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if (idxMax == 4)
            {
                ResultText.Content = "n luk";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type4_n_arc.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if (idxMax == 5)
            {
                ResultText.Content = "Horizontalna elipsa";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type5_horizontal_ellipse.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }
            else if (idxMax == 6)
            {
                ResultText.Content = "Vertikalna elipsa";
                bitmap.BeginInit();
                bitmap.UriSource = new Uri(@"ArcsTypesImages\type6_vertical_ellipse.jpg", UriKind.Relative);
                bitmap.EndInit();
                ResultImage.Source = bitmap;
            }

            //InitializeComponent();
        }


        private void DetailsButton_Click(object sender, RoutedEventArgs e)
        {
            MessageBox.Show(strDetails, "Detalji klasifikacije", MessageBoxButton.OK, MessageBoxImage.Information);
        }
    }
}
