// ROS
#include <ros/ros.h>

////CV bridge
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

//// OpenCV
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>


#include "label_reader.h"
#include "ranger_librarian.h"

//LabelReader lr(5, 10, 0.7, 640, 480);
//bool readLabel = false;

//unsigned int depth_below_counter = 0;
//float depth_below_threshold = 0.2;
//unsigned int depth_below_frames = 10;


//void rgb_callback(const sensor_msgs::ImageConstPtr& msg) {

//    // get mutable cv image
//    cv_bridge::CvImagePtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    //process
//    lr.readLabel(readLabel);

//    if (lr.processFrame(cv_ptr->image)){
//        std::cout << lr.getAuthor() << endl;
//        lr.reset();
//        readLabel = false;
//    }

//    if (readLabel) {
//        putText(cv_ptr->image, "Reading", cv::Point(10,10), CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0,250));
//    }
//    // Update GUI Window
//    namedWindow("OUTPUT_WINDOW", WINDOW_AUTOSIZE);
//    cv::imshow("OUTPUT_WINDOW", cv_ptr->image);
//    cv::waitKey(3);
//}

//void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
//    // Direct deserialize to data array pointer;
//    //const float * depth_row = reinterpret_cast<const float * >( &depth_image_msg->data[0] );

//    cv_bridge::CvImageConstPtr depth_img_ptr;
//    // Data access using CV bridge, shared, returning a const CvImage
//    try {
//        depth_img_ptr = cv_bridge::toCvShare(msg, msg->encoding);

//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("Could not deserialize depth image: %s", e.what());
//        return;
//    }

//    // copy Mat header
//    Mat depth_image = depth_img_ptr->image;
//    // create center image region taking 1/(3^2) of image
//    // e.g. from cols/3 to cols/3*2 and from rows/3 to rows/3*2
//    Rect center_region = Rect(depth_image.cols/3, depth_image.rows/3, depth_image.cols/3, depth_image.rows/3);

//    // create a matrix header for a part of the bigger matrix
//    Mat depth_image_center( depth_image, center_region );
//    // create mask for non nan values
//    Mat depth_center_not_nan = ~(depth_image_center!=depth_image_center);
//    // calculate mean
//    float center_mean = mean(depth_image_center, depth_center_not_nan)[0];

//    // convert if needed
//    if (msg->encoding == "16UC1") {
//        // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
//        center_mean = center_mean / 1000; // (mm to m)
//    }

//    if (center_mean < depth_below_threshold) {
//        depth_below_counter += 1;
//    } else {
//        depth_below_counter = 0;
//    }

//    if (depth_below_counter > depth_below_frames) {
//        readLabel = true;
//    }
//}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "ranger_librarian");
//    ros::NodeHandle nh_;
//    image_transport::ImageTransport it_(nh_);

//    // Subscribers
//    image_transport::Subscriber sub_rgb_;
//    image_transport::Subscriber sub_depth_;

//    string rgb_image, depth_image;

//    // get parameters, if not use defaults
//    nh_.param<string>("rgb_image", rgb_image, "/camera/rgb/image_raw");
//    nh_.param<string>("depth_image", depth_image, "/camera/depth_registered/image_raw");

//    // Subscribe
//    sub_rgb_ = it_.subscribe(rgb_image, 1, rgb_callback);
//    sub_depth_ = it_.subscribe(depth_image, 1, depth_callback);

//    ros::spin();
//    return 0;
//}

/// MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ranger_librarian_node");
    RangerLibrarian rl;
    rl.run();
}
