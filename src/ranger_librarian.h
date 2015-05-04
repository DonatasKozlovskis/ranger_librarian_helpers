#ifndef RANGERLIBRARIAN_H
#define RANGERLIBRARIAN_H

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ranger_librarian/WeightFiltered.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ranger_librarian/LabelReadingAction.h"

//CV bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// other
#include "label_reader.h"
#include "include/utils.h"
#include <boost/thread.hpp>
#include <cmath>

using std::string;
using namespace cv;

/**@brief Flag for printing debug messages.*/
bool const DEBUG = false;

/**@brief Default rosparam server values.*/
static const string RGB_IMAGE_TOPIC = "/usb_cam/image_raw";
static const string SCALE_TOPIC = "/scale";
static const string SCALE_FILTERED_TOPIC = "/scale_filtered";

/**@brief Default OCR parameters.*/
static const int OCR_FRAME_SKIP = 5;        // parameter to process each xx frame with OCR
static const int QUEUE_MAX_LENGTH = 10;     // how many historical values to keep in queue
static const double QUEUE_ACCEPT_RATE = 0.7;// last repeated element acceptance rate

// node rate
static const int NODE_RATE = 31;


/// CLASS
class RangerLibrarian
{
public:
    // Constructor and desctructor
    RangerLibrarian();
    ~RangerLibrarian();

    // Callbacks
    void rgb_callback(const sensor_msgs::ImageConstPtr& msg);

    void depth_low_duration_callback(const std_msgs::Float64& msg);
    void depth_low_action_callback(const std_msgs::String& msg);
    void scale_callback(const std_msgs::Float64& msg);
    void scale_filtered_callback(const ranger_librarian::WeightFiltered& msg);

    // methods
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

    ros::Subscriber sub_depth_low_duration_;
    ros::Subscriber sub_depth_low_action_;
    ros::Subscriber sub_scale_;
    ros::Subscriber sub_scale_filtered_;


    // NODE PARAMETERS
    // depth image control parameters;

    //  scale callback
    int weight_max_allowed_;         // maximum weight allowed

    // read label parameters
    int time_depth_low_read_;            // time (s) for depth val = 0 to start label read
    int time_wait_read_label_;           // time (s) to wait for reading a book label
    int time_wait_add_book_;             // time (s) to wait for adding a book


    //CLASS Members

    // Label reader object
    LabelReader lr_;
    actionlib::SimpleActionClient<ranger_librarian::LabelReadingAction> ac;


    // pointer to obtained cv image
    cv_bridge::CvImagePtr cv_ptr;

    // control boolean variables
    bool read_label_;
    bool read_label_success_;
    bool weight_max_reached_;

    // book struct object
    Book last_book_add_;
    ros::Time last_book_add_time_;

    NavigatorAction action_last_;
    NavigatorAction action_current_;

    // list of books
    std::vector<Book> book_list_;

    // methods
    bool book_read_label();
    bool book_read_weight();

};


#endif // RANGERLIBRARIAN_H
