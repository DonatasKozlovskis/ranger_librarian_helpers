#include "utils.h"

///
void getNumberAuthorFromRoi(cv::Mat imRoi, tesseract::TessBaseAPI& tessAPI,
                            boost::circular_buffer<string>& detectedCallNumbersRoi,
                            boost::circular_buffer<string>& detectedAuthorsRoi)
{
    if (imRoi.channels() == 3) {
        // convert to image grayscale
        cv::cvtColor(imRoi, imRoi, cv::COLOR_BGR2GRAY);
    }

    //these techniques didn't improve quality of OCR:
    //blur, equalizeHist, threshold

    // pass the image to tesseract API for OCR
    tessAPI.SetImage((uchar*)imRoi.data, imRoi.cols, imRoi.rows, 1, imRoi.cols);

    // Get the text output from API
    const char* outOCR = tessAPI.GetUTF8Text();
    string ocrOutString = string(outOCR);

    //replace line breaks to space and remove multiple spaces
    replace(ocrOutString.begin(), ocrOutString.end(), '\n', ' ');
    boost::trim_all(ocrOutString);

    // create regular expression for three separate numbers and 2-3 letters
    boost::regex expr("\\b([0-9]{3})\\b.*(\\b[A-Z]{2,3})");

    boost::smatch regexResult;
    if (boost::regex_search(ocrOutString, regexResult, expr)) {
        // regexResult[0] keeps whole match
        string callNumber = regexResult[1]; // first capture
        string author = regexResult[2]; // second capture

        // add result to queues
        detectedCallNumbersRoi.push_back(callNumber);
        detectedAuthorsRoi.push_back(author);
    }
}

bool checkNumberAuthor(boost::circular_buffer<string> &detectedCallNumbersRoi,
                            boost::circular_buffer<string> &detectedAuthorsRoi,
                            int maxLength, double acceptRate)
{

    // if max queue size reached, check if the last added element
    // is repeated at least QUEUE_ACCEPT_RATE*QUEUE_MAX_LENGTH times
    if (detectedCallNumbersRoi.size() == maxLength) {
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
        if (countNumber>=(1.0*acceptRate*maxLength) && countAuthor>=(1.0*acceptRate*maxLength)){
            return true;
        }
    }
    return false;
}
