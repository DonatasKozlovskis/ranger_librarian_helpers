// ROS
#include <ros/ros.h>
// ROS messages
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include "ranger_librarian/LabelReadingAction.h"

//CV bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// other
#include "label_reader.h"

using std::string;

/**@brief OCR parameters.*/
static const int OCR_FRAME_SKIP = 5;        // parameter to process each xx frame with OCR
static const int QUEUE_MAX_LENGTH = 10;     // how many historical values to keep in queue
static const double QUEUE_ACCEPT_RATE = 0.7;// last repeated element acceptance rate

class LabelReadingAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ranger_librarian::LabelReadingAction> as_;
    string action_name_;

    ranger_librarian::LabelReadingFeedback feedback_;
    ranger_librarian::LabelReadingResult result_;

    ros::Subscriber sub_;

    // Label reader object
    LabelReader lr_;

    int data_count_, goal_;
    float sum_, sum_sq_;

    string camera_topic_;

public:
    
    LabelReadingAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name),
        lr_(OCR_FRAME_SKIP, QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE )
    {
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&LabelReadingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&LabelReadingAction::preemptCB, this));

        as_.start();
        lr_.readLabel(true);
    }

    ~LabelReadingAction(void)
    {
    }

    void goalCB()
    {
        // accept the new goal
        camera_topic_ = as_.acceptNewGoal()->camera_topic;
        //subscribe to the data topic of interest
        sub_ = nh_.subscribe(camera_topic_, 1, &LabelReadingAction::camera_callback, this);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());

        lr_.reset();
        sub_.shutdown();
        // set the action state to preempted
        as_.setPreempted();

    }

    void camera_callback(const sensor_msgs::ImageConstPtr& msg)
    {


        // make sure that the action hasn't been canceled
        if (!as_.isActive())
            return;


         bool read_success = false;


        // get mutable cv image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            lr_.readLabel(true);
            read_success = lr_.processFrame(cv_ptr->image);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (read_success) {
            result_.author = lr_.getAuthor();
            result_.call_number = lr_.getCallNumber();
            // set the action state to succeeded
            as_.setSucceeded(result_);
            lr_.reset();
        }


    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "label_reader");

    LabelReadingAction label_reading(ros::this_node::getName());
    ROS_INFO("Starting label reading server");
    ros::spin();

    return 0;
}
