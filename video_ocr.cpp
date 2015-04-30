//OPENCV & OCR
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <boost/circular_buffer.hpp>            // circular queue
#include <boost/regex.hpp>                      // regex
#include <boost/algorithm/string/trim_all.hpp>  // trim all

//custom libraries
#include "include/utils.h"

using namespace cv;
using namespace std;

// camera parameters
static const int IMG_WIDTH = 640;           //all formats: "luvcview -d /dev/video0 -L"
static const int IMG_HEIGHT = 480;
static const int CAM_FPS = 15;              // only 30, 25, 20, 15,
//WINDOW SQUARE
static const int LABEL_SQUARE_W = 200;
static const int LABEL_SQUARE_H = 200;
//WINDOW RECT
static const int LABEL_RECT_W = 400;        // 4:1
static const int LABEL_RECT_H = 100;
// OCR PARAMS
static const int OCR_FRAME_SKIP = 2;        // parameter to process each xx frame with OCR
static const int QUEUE_MAX_LENGTH = 10;     // how many historical values to keep in queue
static const double QUEUE_ACCEPT_RATE = 0.7;// last repeated element acceptance rate


int main(int argc, char** argv)
{

    // create a circular buffers (queues) of capacity QUEUE_MAX_LENGTH
    boost::circular_buffer<string> detectedCallNumbersRoiSquare(QUEUE_MAX_LENGTH);
    boost::circular_buffer<string> detectedAuthorsRoiSquare(QUEUE_MAX_LENGTH);

    boost::circular_buffer<string> detectedCallNumbersRoiRect(QUEUE_MAX_LENGTH);
    boost::circular_buffer<string> detectedAuthorsRoiRect(QUEUE_MAX_LENGTH);

    //Open the default camera
    VideoCapture cameraCap(0);

    // check if we succeeded to open
    if (!cameraCap.isOpened()) {
        return -1;
    }
    // set camera parameter
    cameraCap.set(CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
    cameraCap.set(CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cameraCap.set(CV_CAP_PROP_FPS, CAM_FPS);

    // create windows
    namedWindow("userView",WINDOW_AUTOSIZE);
    namedWindow("roiSquare",WINDOW_AUTOSIZE);
    namedWindow("roiRect",WINDOW_AUTOSIZE);

    // create two active regions in the image( square and long rectangle for OCR processing)
    //Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
    Rect labelRegionSquare = Rect( (IMG_WIDTH-LABEL_SQUARE_W)/2, (IMG_HEIGHT-LABEL_SQUARE_H)/2, LABEL_SQUARE_W, LABEL_SQUARE_H);
    Rect labelRegionRect = Rect( (IMG_WIDTH-LABEL_RECT_W)/2, (IMG_HEIGHT-LABEL_RECT_H)/2, LABEL_RECT_W, LABEL_RECT_H);

    // init Tesseract API
    tesseract::TessBaseAPI tessAPI;
    tessAPI.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tessAPI.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
    tessAPI.SetVariable("tessedit_char_whitelist", "0123456789.()ABCDEFGHIJKLMNOPQRSTUVWXYZ");


    unsigned long int frameCounter = 0;
    for(;;)
    {
        // grab frame from camera
        Mat frameOrig;
        cameraCap >> frameOrig;
        // increase counter
        frameCounter += 1;
        frameCounter %= MAX_UINT32;
        //flip (mirror effect) for better user expierence
        Mat frame;
        flip(frameOrig, frame, 1);     // can't flip in-place (leads to segfault)


        // process with OCR only each XX frame
        if (frameCounter % OCR_FRAME_SKIP == 0) {

            // copy ROIs of image and unflip for processing
            Mat imRoiSquare;
            Mat imRoiRect;

            flip( frame(labelRegionSquare), imRoiSquare, 1);
            flip( frame(labelRegionRect), imRoiRect, 1);

            //process with ocr and fill circular queues
            getNumberAuthorFromRoi(imRoiSquare, tessAPI, detectedCallNumbersRoiSquare, detectedAuthorsRoiSquare);
            getNumberAuthorFromRoi(imRoiRect, tessAPI, detectedCallNumbersRoiRect, detectedAuthorsRoiRect);

            bool bookDetectedRoiSquare = checkNumberAuthor(detectedCallNumbersRoiSquare,
                                                           detectedAuthorsRoiSquare,
                                                           QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE);
            bool bookDetectedRoiRect = checkNumberAuthor(detectedCallNumbersRoiRect,
                                                           detectedAuthorsRoiRect,
                                                           QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE);


            if (bookDetectedRoiSquare || bookDetectedRoiRect) {
                string lastNumber;
                string lastAuthor;

                if (!bookDetectedRoiSquare && bookDetectedRoiRect) {
                    lastNumber =  detectedCallNumbersRoiRect.back();
                    lastAuthor =  detectedAuthorsRoiRect.back();
                } else {
                    lastNumber =  detectedCallNumbersRoiSquare.back();
                    lastAuthor =  detectedAuthorsRoiSquare.back();
                }
                cout << "number: " << lastNumber << endl;
                cout << "author: " << lastAuthor << endl;
                break;
            }
            //display ROIs
            imshow("roiSquare", imRoiSquare);
            imshow("roiRect", imRoiRect);
        } // end if process OCR only each XX frame


        //PREPARE USER OUTPUT IMAGE

        // convert original to grayscale
        Mat userViewFrame;
        cvtColor(frame, userViewFrame, COLOR_BGR2GRAY);
        // blur
        blur(userViewFrame, userViewFrame, Size(20, 20));
        // convert back to colour image variable
        cvtColor(userViewFrame, userViewFrame, COLOR_GRAY2BGR);

        //copy ROIS to on top of blurred gray image
        frame(labelRegionSquare).copyTo( userViewFrame(labelRegionSquare));
        frame(labelRegionRect).copyTo( userViewFrame(labelRegionRect));

        // draw the boxes on original frame
        rectangle(userViewFrame, labelRegionSquare, Scalar(0,0,255), 2);
        rectangle(userViewFrame, labelRegionRect, Scalar(0,0,255), 2);

        //display userView
        imshow("userView", userViewFrame);



        int c = waitKey(1) & 255;
        if (c == 'q' || c == 'Q' || c =='c')
            break;
    } // end for

    //destroy GUI windows
    destroyAllWindows();
    return 0;
}

