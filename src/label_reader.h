#ifndef LABELREADER_H
#define LABELREADER_H

//OPENCV & OCR
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>

//other libs
#include <iostream>
#include <string>
#include <algorithm>
#include <boost/circular_buffer.hpp>            // circular queue
#include <boost/regex.hpp>                      // regex
#include <boost/algorithm/string/trim_all.hpp>  // trim all

using std::string;

//WINDOW BLUR
static const int BLUR_READ = 8;
static const int BLUR_SKIP = 5; // > 0

class LabelReader
{
private:

  // circular buffers (queues) of fixed lenght
  boost::circular_buffer<string> detectedCallNumbersRoiSquare;
  boost::circular_buffer<string> detectedAuthorsRoiSquare;

  boost::circular_buffer<string> detectedCallNumbersRoiRect;
  boost::circular_buffer<string> detectedAuthorsRoiRect;

  //default output strings
  string detectedCallNumber = "";
  string detectedAuthor = "";

  // boolean read frame
  bool read_label = false;

  // frame counter
  unsigned int frameCounter = 0;

  // user window blur
  int blur_kernel = BLUR_SKIP;

  // Tesseract API
  tesseract::TessBaseAPI tessAPI;
  boost::regex tessRegexpr;


  // two active regions in the image( square and long rectangle for OCR processing)
  cv::Rect labelRegionSquare;
  cv::Rect labelRegionRect;


  //ocr params
  int ocrFrameSkip;
  int queueLenght;
  double queueAcceptRate;
  //camera image paramts
  int imgWidth, imgHeight;

  void getNumberAuthorFromRoi(cv::Mat &imRoi,
                              boost::circular_buffer<string> &detectedCallNumbersRoi,
                              boost::circular_buffer<string> &detectedAuthorsRoi);

  bool checkNumberAuthor(boost::circular_buffer<string> &detectedCallNumbersRoi,
                         boost::circular_buffer<string> &detectedAuthorsRoi);
  
  void defineLabelRegion(int img_width, int img_height);
public:
  // constructors
  LabelReader(int ocr_frame_skip,
              int queue_lenght,
              double queue_accept_rate,
              int img_width = 0,
              int img_height = 0);
  // destructor
  ~LabelReader();

  //void imageCallback(const sensor_msgs::ImageConstPtr& msg);


  // method to return class object to init state
  void reset();

  // get read_label
  bool readLabel();
  // set read_label
  void readLabel(bool read);

  // process frame
  bool processFrame(const cv::Mat &inputFrame);

  // prepare user image
  void prepareUserImage(cv::Mat &inputFrame);

  //getters
  string getCallNumber();
  string getAuthor();
};


#endif // LABELREADER_H
