// ROS
#include <ros/ros.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

//CV bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using std::string;


ros::Publisher pub_depth_low_duration;

/// Class
class DepthBelowTimeNode
{
private:
    // Node, publishers and subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    ros::Publisher pub_depth_low_duration_;
    ros::Publisher pub_depth_low_action_;
    // subscriber
    image_transport::Subscriber sub_depth_;

    // depth image control parameters;
    double depth_below_threshold_;      // depth threshold to to assume depth val = 0
    double depth_low_time_slow_;        // time (s) for depth val = 0 to action slow
    double depth_low_time_stop_;        // time (s) for depth val = 0 to action read
    double depth_low_time_stuck_;       // time (s) for depth val = 0 to action stuck

    // timer
    double depth_above_time_;
    // last action
    string action_before_;

public:
    // constructor & destructor
    DepthBelowTimeNode ();
    ~DepthBelowTimeNode ();

    // callback
    void depth_callback(const sensor_msgs::ImageConstPtr& msg);
};



/// CONSTRUCTOR
DepthBelowTimeNode::DepthBelowTimeNode() :
    nh_("~"), it_(nh_),
    depth_above_time_(0), action_before_("")
{
    string depth_image_topic;

    // get parameter for subscriber, if not use default
    nh_.param<string>("depth_image", depth_image_topic, "/camera/depth_registered/image_raw");
    // get the rest of parameters
    nh_.param<double>("depth_below_threshold", depth_below_threshold_, double(0.2));

    nh_.param<double>("depth_low_time_slow",   depth_low_time_slow_,   double(0.5));
    nh_.param<double>("depth_low_time_read",   depth_low_time_stop_,   double(2));
    nh_.param<double>("depth_low_time_stuck",  depth_low_time_stuck_,  double(10));

    // Subscriber
    sub_depth_  = it_.subscribe(depth_image_topic, 1, &DepthBelowTimeNode::depth_callback, this);

    // Publishers
    pub_depth_low_duration_ =   nh_.advertise<std_msgs::Float64>("depth_low_duration", 1);
    pub_depth_low_action_ =     nh_.advertise<std_msgs::String>("depth_low_action", 1);
}

/// DESTRUCTOR
DepthBelowTimeNode::~DepthBelowTimeNode()
{

}

void DepthBelowTimeNode::depth_callback(const sensor_msgs::ImageConstPtr& msg) {

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
    cv::Mat depth_image = depth_img_ptr->image;
    // create center image region taking 1/(3^2) of image
    // e.g. from cols/3 to cols/3*2 and from rows/3 to rows/3*2
    cv::Rect center_region = cv::Rect(depth_image.cols/3, depth_image.rows/3, depth_image.cols/3, depth_image.rows/3);

    // create a matrix header of a part of the bigger matrix
    cv::Mat depth_image_center( depth_image, center_region );
    // replace NaN with zeros
    depth_image_center.setTo(0, depth_image_center!=depth_image_center);

    // calculate depth image center mean
    double depth_center_mean = cv::mean(depth_image_center)[0];

    // convert if needed
    if (msg->encoding == "16UC1") {
        // depths are 16-bit unsigned ints, in mm. Convert to 32-bit float (meters)
        depth_center_mean = depth_center_mean / 1000; // (mm to m)
    }


    // update time when depth was bigger than threshold
    double current_time = ros::Time::now().toSec();

    if (depth_center_mean > depth_below_threshold_) {
        depth_above_time_ = current_time;
    }

    // compute time
    double time_below = current_time - depth_above_time_;

    // compute action
    string action;
    if (time_below > depth_low_time_stuck_) {
        action = "stuck";
    } else if (time_below > depth_low_time_stop_) {
        action = "stop";
    } else if (time_below > depth_low_time_slow_) {
        action = "slow";
    } else {
        action = "go";
    }


    // create timer message;
    std_msgs::Float64 tm_msg;
    // publish
    tm_msg.data = time_below;
    pub_depth_low_duration_.publish(tm_msg);


    // publish action if not equal to the last one
    if (action.compare(action_before_)!=0) {
        // create action message;
        std_msgs::String str_msg;
        str_msg.data = action;
        pub_depth_low_action_.publish(str_msg);
    }

    // update action before;
    action_before_ = action;
}

/// MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_below_time_node");
    DepthBelowTimeNode depth_below_time_node;

    ros::spin();

    return 0;
}
