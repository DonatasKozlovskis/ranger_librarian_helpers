#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>
#include <iostream>
#include <string>
#include <boost/regex.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    string imgFileName = "/home/st13nod/src/ranger_book/images/c1.jpg";

    // Load image and check if suceeded
    Mat img = imread(imgFileName);
    if (img.empty()) {
        cout << "Cannot open source image!" << endl;
        return -1;
    }

    // image pre-processing
    imshow("", img);
    Mat img_gray;
    cvtColor(img, img_gray, CV_BGR2GRAY);

    // Tesseract API
    tesseract::TessBaseAPI tess;
    tess.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tess.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
    tess.SetVariable("tessedit_char_whitelist", "0123456789.()ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    // pass the image
    tess.SetImage((uchar*)img_gray.data, img_gray.cols, img_gray.rows, 1, img_gray.cols);

    // Get the text
    const char* out = tess.GetUTF8Text();
    string ss = string(out);

    // printa all obtained text
    cout << ss << endl;


    boost::regex expr("(\\b[0-9]{3}\\b)");
    boost::smatch what;
    if (boost::regex_search(ss, what, expr))
    {
        string s = what[1];
        cout << "classifier: " << endl;
        cout << s << '\n';

    }

    waitKey(0);
    return 0;
}
