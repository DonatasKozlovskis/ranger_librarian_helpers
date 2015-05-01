#include "ranger_librarian.h"

// CONSTRUCTOR
RangerLibrarian::RangerLibrarian():
    nh_("~"), it_(nh_), nh_rate_(NODE_RATE),
    depth_read_label_(false), read_success_(false),
    robot_stuck_(false),
    depth_below_counter_(0), depth_below_time_(0),
    weight_change_(0),
    last_label_time_(0), last_label_author_(""), last_label_number_(""), last_label_weight_(0),

    lr_(OCR_FRAME_SKIP, QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE, IMG_WIDTH, IMG_HEIGHT)

{
    string rgb_image_topic;
    string depth_image_topic;
    string scale_topic;

    // get parameters for subscribers, if not use defaults
    nh_.param<string>("rgb_image", rgb_image_topic, "/usb_cam/image_raw");
    nh_.param<string>("depth_image", depth_image_topic, "/camera/depth_registered/image_raw");
    nh_.param<string>("scale", scale_topic, "/ranger_scale");


    // Subscribers
    sub_rgb_    = it_.subscribe(rgb_image_topic, 1, &RangerLibrarian::rgb_callback, this);
    sub_depth_  = it_.subscribe(depth_image_topic, 1, &RangerLibrarian::depth_callback, this);
    sub_scale_  = nh_.subscribe<const std_msgs::Float64&>(scale_topic, 1, &RangerLibrarian::scale_callback, this);

    // get the rest of paramters
    nh_.param("depth_below_read", depth_low_time_read, int(2));
    nh_.param("depth_below_stuck", depth_low_time_stuck, int(5));
    nh_.param("depth_below_threshold", depth_below_threshold, double(0.2));

    nh_.param("read_time_label", read_time_label, int(5));



    // get the scale paramters
    nh_.param("weight_sensitivity", weight_sensitivity, double(0.15));
    nh_.param("weight_max_allowed", weight_max_allowed, int(6));
    nh_.param("weight_wait_time_book", weight_wait_time_book, int(8));



    // calculate max used time to init circular buffer capacities
    int max_time = max(max(depth_low_time_read,depth_low_time_stuck), weight_wait_time_book);

    // set circular buffer sizes
    weight_measures.set_capacity((int)(NODE_RATE*max_time));
    depth_measures.set_capacity((int)(NODE_RATE*max_time));
}

// DESTRUCTOR
RangerLibrarian::~RangerLibrarian()
{

}


// CALLBACKS
void RangerLibrarian::rgb_callback(const sensor_msgs::ImageConstPtr& msg) {

    // get mutable cv image
    //cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // prepare user output image view
    cv::Mat userImage = cv_ptr->image.clone();
    lr_.prepareUserImage(userImage);


    if (depth_read_label_) {
        putText(cv_ptr->image, "Reading", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }
    if (robot_stuck_) {
        putText(cv_ptr->image, "Stuck", cv::Point(15,15), CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(0,0,250));
    }

    // Update GUI Window
    namedWindow("OUTPUT_WINDOW", WINDOW_AUTOSIZE);
    cv::imshow("OUTPUT_WINDOW", userImage);
    cv::waitKey(3);
}

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

    // Logical control

    // last known time when depth is bigger than threshold
    if (depth_center_mean > depth_below_threshold) {
        depth_below_time_ = ros::Time::now().toSec();
    }



}

void RangerLibrarian::scale_callback(const std_msgs::Float64& msg) {


    double weight_current = msg.data;

    // add to circular buffer
    weight_measures.push_front(weight_current);

    weight_last_ = weight_current;

//    weight_change_ =  weight_current - weight_last_;

//    if (abs(weight_change_) > weight_sensitivity) {
//        // significant change;
//        weight_change_ = 0;
//    } else {
//        // not significant weight increase
//        weight_change_ = 0;
//    }


}


int RangerLibrarian::run() {

    long int current_time;

    while (ros::ok()) {

        current_time = ros::Time::now().toSec();

        std::cout << "move around" << std::endl;


        if ( depth_below_time_!=0 && ( current_time-depth_below_time_>= depth_low_time_read)) {
            depth_read_label_ = true;
            depth_below_time_ = 0;
        }
        if (depth_read_label_) {
            std::cout <<  "try read label " << std::endl;
            if (read_label()) {
                 std::cout <<  "read label success! waiting for book " << std::endl;

                 if (read_book_weight()) {
                     std::cout <<  " book added! " << std::endl;
                 } else {
                     std::cout <<  " book added failed! " << std::endl;
                 }

            } else {
                std::cout <<  "read label failed! " << std::endl;
            }


            depth_read_label_ = false;

        }


        ros::spinOnce();
        nh_rate_.sleep();
    }

}

bool RangerLibrarian::read_label() {

    read_success_ = false;

    long int current_time = ros::Time::now().toSec();
    long int start_time = current_time;


    lr_.readLabel(true);

    while (!read_success_&& (current_time-start_time < read_time_label) && ros::ok() ) {

        read_success_ = lr_.processFrame(cv_ptr->image);

        ros::spinOnce();
        nh_rate_.sleep();
        current_time = ros::Time::now().toSec();
    }

    if (read_success_){
        //update last read values
        last_label_time_ = ros::Time::now().toSec();
        last_label_author_ = lr_.getAuthor();
        last_label_number_ = lr_.getCallNumber();
        last_label_weight_ = weight_last_;
    } else {
        last_label_time_ = 0;
        last_label_weight_ = 0;
        last_label_author_ = "";
        last_label_number_ = "";
    }

    lr_.reset();

    return read_success_;
}
bool RangerLibrarian::read_book_weight() {

    bool book_added = false;

    long int current_time = ros::Time::now().toSec();
    long int start_time = current_time;

    double weight_before_book = weight_last_;

    while (!book_added && ( (current_time-start_time)< weight_wait_time_book) && ros::ok() ) {

        double mean_weight = weight_homogeneous(0,60);


        if (mean_weight && ( (mean_weight - weight_before_book) > weight_sensitivity )) {


            std::cout << "mean weight: " << mean_weight << " before: " << weight_before_book << std::endl;

//            bool weight_equal = true;
//            int counter = 0;
//            double weight_sum = 0;
//            int window_size = 60;
//            int frame_begin = 0;

//            auto it=weight_measures.begin() + frame_begin;

//            double weight_previous = *it;
//            while( weight_equal && counter < window_size) {

//                weight_sum += (*it);

//                double  weight_diff = (*it) - weight_previous;
//                bool two_frames_equal = std::abs(weight_diff) < weight_sensitivity;
//                weight_equal = weight_equal && two_frames_equal;

//                std::cout << (*it) << " " << *(it+1) << " " << weight_diff << " " << two_frames_equal << std::endl;

//                weight_previous = (*it);
//                it++;
//                counter++;
//            }


            book_added = true;
        }


        ros::spinOnce();
        nh_rate_.sleep();
        current_time = ros::Time::now().toSec();
    }

    return book_added;
}

double RangerLibrarian::weight_homogeneous(int frame_begin, int window_size) {

    double weight_mean = std::nan("");


    if (weight_measures.size() > (frame_begin+window_size)) {

        bool weight_equal = true;
        int counter = 0;
        double weight_sum = 0;

        auto it=weight_measures.begin() + frame_begin;

        double weight_previous = *it;
        while( weight_equal && counter < window_size) {

            weight_sum += (*it);

            double  weight_diff = (*it) - weight_previous;
            bool two_frames_equal = std::abs(weight_diff) < weight_sensitivity;
            weight_equal = weight_equal && two_frames_equal;


            weight_previous = (*it);
            it++;
            counter++;
        }

        if (weight_equal) {
            weight_mean = weight_sum / window_size;
        }
    }

    return weight_mean;
}
