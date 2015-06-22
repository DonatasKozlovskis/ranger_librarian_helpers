#include "label_reader.h"


LabelReader::LabelReader(
        int ocr_frame_skip,
        int queue_lenght,
        double queue_accept_rate,
        int img_width,
        int img_height)
  :
    ocrFrameSkip(ocr_frame_skip),
    queueAcceptRate(queue_accept_rate),
    queueLenght(queue_lenght),
    imgWidth(img_width),
    imgHeight(img_height),

    detectedCallNumbersRoiSquare(queue_lenght),
    detectedAuthorsRoiSquare(queue_lenght),
    detectedCallNumbersRoiRect(queue_lenght),
    detectedAuthorsRoiRect(queue_lenght)

{
    // init Tesseract API
    tessAPI.Init(NULL, "eng", tesseract::OEM_DEFAULT);
    tessAPI.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
    tessAPI.SetVariable("tessedit_char_whitelist", "0123456789.()ABCDEFGHIJKLMNOPQRSTUVWXYZ");

    // set regular expression
    tessRegexpr = "\\b([0-9]{3})\\b.*(\\b[A-Z]{2,3})";

    defineLabelRegion(imgWidth, imgHeight);

}

LabelReader::~LabelReader()
{
}


void LabelReader::reset() {
    detectedAuthor = "";
    detectedCallNumber = "";
    detectedCallNumbersRoiSquare.clear();
    detectedAuthorsRoiSquare.clear();
    detectedCallNumbersRoiRect.clear();
    detectedAuthorsRoiRect.clear();

    readLabel(false);
}

bool LabelReader::readLabel() {
    return read_label;
}
void LabelReader::readLabel(bool read) {
    read_label = read;
    if (read) {
        blur_kernel = BLUR_READ;
    } else {
        blur_kernel = BLUR_SKIP;
    }
}

string LabelReader::getCallNumber() {
    return detectedCallNumber;
}

string LabelReader::getAuthor() {
    return detectedAuthor;
}

void LabelReader::defineLabelRegion(int img_width, int img_height) {
    imgWidth = img_width;
    imgHeight = img_height;
    // create two active regions in the image( square and long rectangle for OCR processing)
    int label_square_wh = std::min(imgWidth, imgHeight)/7*3;
    int label_rect_h = std::min(imgWidth, imgHeight)/7*1.5;
    int label_rect_w = std::min(imgWidth, label_rect_h*4); // 1:4

    // SQUARE
    labelRegionSquare = cv::Rect( (imgWidth-label_square_wh)/2, (imgHeight-label_square_wh)/2, label_square_wh, label_square_wh);
    // RECT
    labelRegionRect = cv::Rect( (imgWidth-label_rect_w)/2, (imgHeight-label_rect_h)/2, label_rect_w, label_rect_h);

}

bool LabelReader::processFrame(const cv::Mat &inputFrame) {

    if (inputFrame.cols != imgWidth || inputFrame.rows != imgHeight) {
        defineLabelRegion(inputFrame.cols, inputFrame.rows);
    }

    bool readSuccess = false;

    // increase counter
    frameCounter += 1;
    frameCounter %= MAX_UINT32;

    // process with OCR only each XX frame
    if ( read_label && ((frameCounter % ocrFrameSkip) == 0)) {

        // copy ROIs for image processing
        cv::Mat imRoiSquare = inputFrame(labelRegionSquare).clone();
        cv::Mat imRoiRect = inputFrame(labelRegionRect).clone();

        //process with ocr and fill circular queues
        getNumberAuthorFromRoi(imRoiSquare, detectedCallNumbersRoiSquare, detectedAuthorsRoiSquare);
        getNumberAuthorFromRoi(imRoiRect, detectedCallNumbersRoiRect, detectedAuthorsRoiRect);

        bool bookDetectedRoiSquare = checkNumberAuthor(detectedCallNumbersRoiSquare,
                                                       detectedAuthorsRoiSquare);
        bool bookDetectedRoiRect = checkNumberAuthor(detectedCallNumbersRoiRect,
                                                       detectedAuthorsRoiRect);


        if (bookDetectedRoiSquare || bookDetectedRoiRect) {
            if (!bookDetectedRoiSquare && bookDetectedRoiRect) {
                detectedCallNumber =  detectedCallNumbersRoiRect.back();
                detectedAuthor =  detectedAuthorsRoiRect.back();
            } else {
                detectedCallNumber =  detectedCallNumbersRoiSquare.back();
                detectedAuthor =  detectedAuthorsRoiSquare.back();
            }
            readSuccess = true;
        }

    } // end if process OCR only each XX frame

    return readSuccess;
}

/// PREPARE USER OUTPUT IMAGE
void LabelReader::prepareUserImage(cv::Mat &inputFrame) {

    if (inputFrame.cols != imgWidth || inputFrame.rows != imgHeight) {
        defineLabelRegion(inputFrame.cols, inputFrame.rows);
    }

    // flip (mirror effect) for better user expierence
    cv::Mat frameFlipped;
    flip(inputFrame, frameFlipped, 1);     // can't flip in-place (leads to segfault)

    // overwrite original to flipped grayscale
    cv::cvtColor(frameFlipped, inputFrame, cv::COLOR_BGR2GRAY);
    // blur
     cv::blur(inputFrame, inputFrame, cv::Size(blur_kernel, blur_kernel));
    // convert back to colour image variable
     cv::cvtColor(inputFrame, inputFrame, cv::COLOR_GRAY2BGR);

    //copy color ROIs to on top of blurred gray image
    frameFlipped(labelRegionSquare).copyTo( inputFrame(labelRegionSquare));
    frameFlipped(labelRegionRect).copyTo( inputFrame(labelRegionRect));

    // draw the reading boxes on original frame
     cv::rectangle(inputFrame, labelRegionSquare, cv::Scalar(0,0,255), 2);
     cv::rectangle(inputFrame, labelRegionRect, cv::Scalar(0,0,255), 2);

}


///
void LabelReader::getNumberAuthorFromRoi(cv::Mat &imRoi,
                                         boost::circular_buffer<string> &detectedCallNumbersRoi,
                                         boost::circular_buffer<string> &detectedAuthorsRoi) {

    if (imRoi.channels() == 3) {
        // convert to image grayscale
        cv::cvtColor(imRoi, imRoi, cv::COLOR_BGR2GRAY);
    }

    // these techniques didn't improve quality of OCR:
    // blur, equalizeHist, threshold
//    GaussianBlur(imRoi, imRoi, cv::Size(5,5), 0 );
//    adaptiveThreshold(imRoi, adaptive, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 15, 5);
//    threshold(imRoi, otsu,  128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);


    // pass the image to tesseract API for OCR
    tessAPI.SetImage((uchar*)imRoi.data, imRoi.cols, imRoi.rows, 1, imRoi.cols);

    // Get the text output from API
    const char* outOCR = tessAPI.GetUTF8Text();
    string ocrOutString = string(outOCR);

    //replace line breaks to space and remove multiple spaces
    replace(ocrOutString.begin(), ocrOutString.end(), '\n', ' ');
    boost::trim_all(ocrOutString);

    boost::smatch regexResult;
    if (boost::regex_search(ocrOutString, regexResult, tessRegexpr)) {
        // regexResult[0] keeps whole match
        string callNumber = regexResult[1]; // first capture
        string author = regexResult[2]; // second capture

        // add result to queues
        detectedCallNumbersRoi.push_back(callNumber);
        detectedAuthorsRoi.push_back(author);
    }
}

bool LabelReader::checkNumberAuthor(boost::circular_buffer<string> &detectedCallNumbersRoi,
                                    boost::circular_buffer<string> &detectedAuthorsRoi)
{

    // if max queue size reached, check if the last added element
    // is repeated at least queueAcceptRate*queueLenght times
    if (detectedCallNumbersRoi.size() == queueLenght) {
        uint countNumber = 0;
        uint countAuthor = 0;

        string lastNumber =  detectedCallNumbersRoi.back();
        string lastAuthor =  detectedAuthorsRoi.back();

        // count lastNumber occurencies
        for(auto number: detectedCallNumbersRoi) {
            if (number==lastNumber) {
                countNumber +=1;
            }
        }
        // count lastAuthor occurencies
        for(auto author: detectedAuthorsRoi) {
            if (author==lastAuthor) {
                countAuthor +=1;
            }
        }
        if (countNumber>=(1.0*queueAcceptRate*queueLenght) && countAuthor>=(1.0*queueAcceptRate*queueLenght)){
            return true;
        }
    }

    return false;
}
