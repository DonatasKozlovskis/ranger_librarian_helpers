#ifndef UTILS_H
#define UTILS_H

//OPENCV & OCR
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>

#include <string>

using std::string;

// Struct for books
struct Book {
    string author;
    string callNumber;
    double weight;
};

// navigator ACTIONS
enum NavigatorAction
{
    NAVIGATOR_STOP,
    NAVIGATOR_MOVE,
    NAVIGATOR_FINISH
};

// navigator ACTIONS
enum DepthLowCommand
{
    DEPTH_LOW_SLOW,
    DEPTH_LOW_STOP,
    DEPTH_LOW_STUCK
};


#endif // UTILS_H
