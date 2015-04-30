#include "ranger_librarian.h"

// CONSTRUCTOR
RangerLibrarian::RangerLibrarian():
    nh_("~"), it_(nh_), nh_rate_(NODE_RATE),
    depth_read_label_(false), read_success_(false),
    robot_stuck_(false),
    depth_below_counter_(0),
    weight_change_(0),
    last_label_time_(0), last_label_author_(""), last_label_number_(""), last_label_weight_(0),

    lr_(OCR_FRAME_SKIP, QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE, IMG_WIDTH, IMG_HEIGHT)

{
    string rgb_image_topic;
    string depth_image_topic;
    string scale_topic;

    // get parameters for subscribers, if not use defaults
    nh_.param<string>("rgb_image", rgb_image_topic, "/camera/rgb/image_raw");
    nh_.param<string>("depth_image", depth_image_topic, "/camera/depth_registered/image_raw");
    nh_.param<string>("scale", scale_topic, "/ranger_scale");


    // Subscribers
    sub_rgb_    = it_.subscribe(rgb_image_topic, 1, &RangerLibrarian::rgb_callback, this);
    sub_depth_  = it_.subscribe(depth_image_topic, 1, &RangerLibrarian::depth_callback, this);
    sub_scale_  = nh_.subscribe<const std_msgs::Float64&>(scale_topic, 1, &RangerLibrarian::scale_callback, this);

    // get the rest of paramters
    nh_.param("depth_below_read", depth_low_time_read, int(10));
    nh_.param("depth_below_stuck", depth_low_time_stuck, int(210));
    nh_.param("depth_below_threshold", depth_below_threshold, double(0.2));

    // get the scale paramters
    nh_.param("weight_sensitivity", weight_sensitivity, double(0.1));
    nh_.param("weight_max_allowed", weight_max_allowed, int(6));
    nh_.param("weight_wait_time_book", weight_wait_time_book, int(8));

    // calculate max used time for circular buffer capacities
    int max_time = max(max(depth_low_time_read,depth_low_time_stuck), weight_wait_time_book);

    // set circular buffer sizes
    weight_measures.set_capacity((int)(NODE_RATE*max_time));
    depth_measures.set_capacity((int)(NODE_RATE*max_time));
}

// DESTRUCTOR
RangerLibrarian::~RangerLibrarian()
{

}


int RangerLibrarian::run() {

    double current_time;

    while (ros::ok()) {
        current_time = ros::Time::now().toSec();

        // check if we should add book
        if (last_label_time_>0){
            std::cout <<  "waiting for book" << std::endl;
            if ( (current_time - last_label_time_) > weight_wait_time_book ) {

                weight_change_ = weight_last_ -  last_label_weight_;
                if (weight_change_ > weight_sensitivity) {
                    std::cout << "book added! weight: " << weight_change_ << std::endl;
                    break;
                } else {
                    std::cout << "failed to add book" << std::endl;
                }

                last_label_time_ = 0;
            }

        }


        if (weight_last_ > weight_max_allowed ) {
            std::cout << "max weigh allowed, going back" << std::endl;
        }

        ros::spinOnce();
        nh_rate_.sleep();
    }


}

// CALLBACKS
void RangerLibrarian::rgb_callback(const sensor_msgs::ImageConstPtr& msg) {

    // get mutable cv image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //process
    lr_.readLabel(depth_read_label_);
    read_success_ = lr_.processFrame(cv_ptr->image);

    if (read_success_){
        //update last read values
        last_label_time_ = ros::Time::now().toSec();
        last_label_author_ = lr_.getAuthor();
        last_label_number_ = lr_.getCallNumber();
        last_label_weight_ = weight_last_;

        lr_.reset();
        depth_read_label_ = false;
    }


    // prepare user output image view
    lr_.prepareUserImage(cv_ptr->image);


    if (depth_read_label_) {
        putText(cv_ptr->image, "Reading", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }
    if (robot_stuck_) {
        putText(cv_ptr->image, "Stuck", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }

    // Update GUI Window
    namedWindow("OUTPUT_WINDOW", WINDOW_AUTOSIZE);
    cv::imshow("OUTPUT_WINDOW", cv_ptr->image);
    cv::waitKey(3);
}

//bool RangerLibrarian::add_book() {

//}

void RangerLibrarian::depth_callback(const sensor_msgs::ImageConstPtr& msg) {

    // Data access using CV bridge, shared, returning a const CvImage
    cv_bridge::CvImageConstPtr depth_img_ptr;
    try
    {
        depth_img_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not deserialize depth image: %s", e.what());
        return;
    }

    // copy image matrix header
    Mat depth_image = depth_img_ptr->image;
    // create center image region taking 1/(3^2) of image
    // e.g. from cols/3 to cols/3*2 and from rows/3 to rows/3*2
    Rect center_region = Rect(depth_image.cols/3, depth_image.rows/3, depth_image.cols/3, depth_image.rows/3);

    // create a matrix header of a part of the bigger matrix
    Mat depth_image_center( depth_image, center_region );
    // replace NaN with zeros
    depth_image_center.setTo(0, depth_image_center!=depth_image_center);

    // calculate depth image center mean
    double depth_center_mean = mean(depth_image_center)[0];

    // convert if needed
    if (msg->encoding == "16UC1") {
        // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
        depth_center_mean = depth_center_mean / 1000; // (mm to m)
    }

    // add to circular buffer
    depth_measures.push_front(depth_center_mean);

//    for (auto rit=depth_measures.begin(); (rit!= depth_measures.end() && std::distance( depth_measures.begin(), rit) < 5); ++rit)
//        std::cout << *rit << " ";
//    std::cout << std::endl;


    // control counter of conse frames
    if (depth_center_mean < depth_below_threshold) {
        // increase
        depth_below_counter_ += 1;
    } else {
        // reset counter
        depth_below_counter_ = 0;

        robot_stuck_ = false;
    }


    if (!depth_read_label_ && (depth_below_counter_ < depth_low_time_stuck) && (depth_below_counter_ > depth_low_time_read)) {
        depth_read_label_ = true;
    }

    if ((depth_below_counter_ > depth_low_time_stuck)) {
        robot_stuck_ = true;
        depth_read_label_ = false;
    }

}

void RangerLibrarian::scale_callback(const std_msgs::Float64& msg) {


    double weight_current = msg.data;

    // add to circular buffer
    weight_measures.push_front(weight_current);

    weight_change_ =  weight_current - weight_last_;

    if (abs(weight_change_) > weight_sensitivity) {
        // significant change;
        weight_change_ = 0;
    } else {
        // not significant weight increase
        weight_change_ = 0;
    }


    weight_last_ = weight_current;

//    ROS_INFO("weight callback: %0.2f", weight_last_);
}


