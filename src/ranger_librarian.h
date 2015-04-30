#ifndef RANGERLIBRARIAN_H
#define RANGERLIBRARIAN_H

// ROS
#include <ros/ros.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>

//CV bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// other
#include "label_reader.h"

// camera parameters
static const int IMG_WIDTH = 640;           //all formats: "luvcview -d /dev/video0 -L"
static const int IMG_HEIGHT = 480;

// OCR PARAMS
static const int OCR_FRAME_SKIP = 5;        // parameter to process each xx frame with OCR
static const int QUEUE_MAX_LENGTH = 10;     // how many historical values to keep in queue
static const double QUEUE_ACCEPT_RATE = 0.7;// last repeated element acceptance rate

// node rate
static const int NODE_RATE = 10;

using std::string;
using namespace cv;

// Struct for books
struct Book {
    string author;
    string callNumber;
    double weight;
};

/// CLASS
class RangerLibrarian
{


public:
    // Constructor and desctructor
    RangerLibrarian();
    ~RangerLibrarian();

    // Callbacks
    void rgb_callback(const sensor_msgs::ImageConstPtr& msg);
    void depth_callback(const sensor_msgs::ImageConstPtr& msg);
    void scale_callback(const std_msgs::Float64& msg);

    int run();

private:

    // Node
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // Node loop rate
    ros::Rate nh_rate_;

    // Publishers
    ros::Publisher pub_user_rgb_;

    // Subscribers
    image_transport::Subscriber sub_rgb_;
    image_transport::Subscriber sub_depth_;
    ros::Subscriber sub_scale_;


    // NODE PARAMETERS

    // depth image control parameters;
    int depth_low_time_read;           // time (s) for depth val = 0 to start label read
    int depth_low_time_stuck;          // time (s) for depth val = 0 to reset navigato
    double depth_below_threshold;   // depth threshold to to assume depth val = 0

    // book add parameters
    double weight_sensitivity;      // assume weight change when abs(diff) > sensitivity
    int weight_wait_time_book;      // time (s) to wait for adding a book
    int weight_max_allowed;         // maximum weight allowed



    //CLASS Member variables

    // Label reader object
    LabelReader lr_;

    // control boolean variables
    bool depth_read_label_;
    bool read_success_;
    bool robot_stuck_;


    int depth_below_counter_;
    vector<Book> book_list_;

    double weight_change_;
    double weight_last_;

    // book object
    string last_label_author_;
    string last_label_number_;
    double last_label_weight_;
    double last_label_time_;


    // circular buffers (queue) of fixed lenght
    boost::circular_buffer<double> weight_measures;
    boost::circular_buffer<double> depth_measures;


};


#endif // RANGERLIBRARIAN_H
