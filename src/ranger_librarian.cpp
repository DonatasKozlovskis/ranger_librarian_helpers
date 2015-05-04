#include "ranger_librarian.h"

// CONSTRUCTOR
RangerLibrarian::RangerLibrarian():
    nh_("~"), it_(nh_), nh_rate_(NODE_RATE),
    read_label_(false), read_label_success_(false), weight_max_reached_(false),

    lr_(OCR_FRAME_SKIP, QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE, IMG_WIDTH, IMG_HEIGHT)

{
    string rgb_image_topic;
    string depth_low_duration;
    string depth_low_action;
    string scale_topic;
    string scale_filtered_topic;

    // get parameters for subscribers, if not use defaults
    nh_.param<string>("rgb_image", rgb_image_topic, RGB_IMAGE_TOPIC);
    nh_.param<string>("depth_low_duration", depth_low_duration, "/depth_below_timer/depth_low_duration");
    nh_.param<string>("depth_low_action", depth_low_action,     "/depth_below_timer/depth_low_action");

    nh_.param<string>("scale", scale_topic, SCALE_TOPIC);
    nh_.param<string>("scale_filtered", scale_filtered_topic, SCALE_FILTERED_TOPIC);


    // Subscribers
    sub_rgb_    = it_.subscribe(rgb_image_topic, 1, &RangerLibrarian::rgb_callback, this);

    sub_depth_low_duration_  = nh_.subscribe<const std_msgs::Float64&>(depth_low_duration, 1, &RangerLibrarian::depth_low_duration_callback, this);
    sub_depth_low_action_  = nh_.subscribe<const std_msgs::String&>(depth_low_action, 2, &RangerLibrarian::depth_low_action_callback, this);

    sub_scale_  =           nh_.subscribe<const std_msgs::Float64&>(scale_topic, 1, &RangerLibrarian::scale_callback, this);
    sub_scale_filtered_  =  nh_.subscribe<const ranger_librarian::WeightFiltered&>(scale_filtered_topic, 1, &RangerLibrarian::scale_filtered_callback, this);

    // get the rest of paramters
    nh_.param("weight_max_allowed", weight_max_allowed_,     int(5));
    nh_.param("time_depth_low_read", time_depth_low_read_,  int(2));
    nh_.param("time_wait_read_label", time_wait_read_label_, int(5));
    nh_.param("time_wait_add_book", time_wait_add_book_,    int(8));

}

/// DESTRUCTOR
RangerLibrarian::~RangerLibrarian()
{

}

/////////////////////////////////////////////////////////////////////
/// CALLBACKS
void RangerLibrarian::rgb_callback(const sensor_msgs::ImageConstPtr& msg) {

    if (DEBUG) {
        printf("rgb_callback msg received.\n");
    }



    // get mutable cv image from cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // prepare user output image view
    cv::Mat userImage = cv_ptr->image.clone();
    lr_.prepareUserImage(userImage);


    if (read_label_) {
        putText(cv_ptr->image, "Reading", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }
    if (read_label_success_) {
        putText(cv_ptr->image, "read_label_success_", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }

    // Update GUI Window
    namedWindow("OUTPUT_WINDOW", WINDOW_AUTOSIZE);
    cv::imshow("OUTPUT_WINDOW", userImage);
    cv::waitKey(3);
}

void RangerLibrarian::depth_low_duration_callback(const std_msgs::Float64& msg) {

//    if (DEBUG) {
//        printf("depth_below_duration msg received:  %0.2f\n", msg.data);
//    }

    double depth_below_duration = msg.data;

//    if (depth_below_duration > time_depth_low_read_) {
//        read_label_ = true;
//    } else {
//        read_label_ = false;
//    }
}

void RangerLibrarian::depth_low_action_callback(const std_msgs::String& msg) {

    string depth_low_action = msg.data;

    if (DEBUG) {
        printf("depth_low_action msg received:  %s\n", depth_low_action.c_str());
    }

    if (depth_low_action.compare("stop")==0 ) {
        read_label_ = true;
    } else {
        read_label_ = false;
    }

    if (read_label_) {
        std::cout <<  "try read label " << std::endl;
        read_label_success_ = book_read_label();
        if (read_label_success_) {
             std::cout <<  "read label success! waiting for book " << std::endl;

             if (book_read_weight()) {
                 std::cout <<  "read book added! " << std::endl;
             } else {
                 std::cout <<  "read book add failed! " << std::endl;
             }
        } else {
            std::cout <<  "read label failed! " << std::endl;
        }

        read_label_ = false;

    }

}

void RangerLibrarian::scale_callback(const std_msgs::Float64& msg) {

    double weight_current = msg.data;

    if (DEBUG) {
        printf("Scale msg received:  %0.2f\n", weight_current);
    }

    if (weight_current > weight_max_allowed_) {
        weight_max_reached_ = true;
    } else {
        weight_max_reached_ = false;
    }

    // implement control of max weight reached
}

void RangerLibrarian::scale_filtered_callback(const ranger_librarian::WeightFiltered& msg) {

    double weight_stable = msg.weight_stable;
    double weight_change = msg.weight_change;
    bool weight_changed =  msg.weight_changed;

    if (DEBUG) {
        printf("scale_filtered_callback msg received:  %0.3f\n", weight_stable);
    }

    if (weight_changed) {
        if (weight_change > 0) {
            last_book_add_time_ = msg.change_time;
            std::cout << "book added" << std::endl;
        } else {
            std::cout << "book removed" << std::endl;
        }
    } else {
        // no change
    }

}
/////////////////////////////////////////////////////////////////////

/// SPINNER
int RangerLibrarian::run() {

    long int current_time;

    while (ros::ok()) {

        current_time = ros::Time::now().toSec();

//        std::cout << "move around" << std::endl;


        ros::spinOnce();
        nh_rate_.sleep();
    }

}

bool RangerLibrarian::book_read_label() {

    bool read_success = false;

    ros::Time start_time = ros::Time::now();
    ros::Time run_time = ros::Time::now();


    lr_.readLabel(true);

    while (!read_success && (run_time-start_time < ros::Duration(time_wait_read_label_)) && ros::ok() ) {

        read_success = lr_.processFrame(cv_ptr->image);

        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        run_time = ros::Time::now();
    }

    if (read_success){
        //update last book values
        last_book_add_time_ = ros::Time::now();
        last_book_add_.author = lr_.getAuthor();
        last_book_add_.callNumber = lr_.getCallNumber();
        last_book_add_.weight = 0;
    } else {
        last_book_add_time_ = ros::Time(0);
        last_book_add_.weight = 0;
        last_book_add_.author = "";
        last_book_add_.callNumber = "";
    }

    lr_.reset();

    return read_success;
}

bool RangerLibrarian::book_read_weight() {

    bool book_added = false;

    ros::Time start_time = ros::Time::now();
    ros::Time run_time = ros::Time::now();

    while (!book_added && (run_time-start_time < ros::Duration(time_wait_read_label_)) && ros::ok() ) {

        if (last_book_add_time_ > start_time ) {
            book_added = true;
        }

        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        run_time = ros::Time::now();
    }

    return book_added;
}
