#ifndef UTILS_H
#define UTILS_H

//OPENCV & OCR
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>

#include <string>
#include <boost/circular_buffer.hpp>            // circular queue
#include <boost/regex.hpp>                      // regex
#include <boost/algorithm/string/trim_all.hpp>  // trim all
#include <algorithm>                            // replace


using namespace std;

/**@brief Wrap an angle to lie in \f$ (-\pi, \pi) \f$.

  Should be used
  @param imRoi
  @return nothing.*/
void getNumberAuthorFromRoi(cv::Mat imRoi, tesseract::TessBaseAPI &tessAPI,
                            boost::circular_buffer<string> &detectedCallNumbersRoi,
                            boost::circular_buffer<string> &detectedAuthorsRoi);

bool checkNumberAuthor(boost::circular_buffer<string> &detectedCallNumbersRoi,
                            boost::circular_buffer<string> &detectedAuthorsRoi,
                            int maxLength, double acceptRate);
#endif // UTILS_H
