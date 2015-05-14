// ROS
#include <ros/ros.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
// custom message
#include <ranger_librarian/WeightFiltered.h>

//other
#include <boost/circular_buffer.hpp>            // circular queue
#include <vector>

using std::string;

// default param values
static const string SCALE_TOPIC = "/scale";
static const double SCALE_SENSITIVITY = 0.15;
static const int SUBSET_SIZE = 4;

/// Class
class WeightChangeMonitor
{
private:
    // Node
    ros::NodeHandle nh_;
    // publisher
    ros::Publisher pub_scale_filtered_;
    // subscriber
    ros::Subscriber sub_scale_;

    // scale processing parameters
    double scale_sensitivity_;  // scale sensitivity
    int subset_size_;           // how many historical values of weight to keep

    // variables
    boost::circular_buffer<double> weight_measures; // buffer of weight measurements

    double weight_stable_;                          // current stable weight value
    double weight_change_;                           // weight  value difference between current and last stable weigths
    ros::Time change_time_;                          // change time
    bool weight_changed_;                           // change occured?


    // methods
    double weight_median();

public:
    // constructor & destructor
    WeightChangeMonitor ();
    ~WeightChangeMonitor ();

    // callback
    void scale_callback(const std_msgs::Float64& msg);
};

/// CONSTRUCTOR
WeightChangeMonitor::WeightChangeMonitor() :
    nh_(),
    weight_stable_(0),
    weight_change_(0),
    change_time_(ros::Time::now()),
    weight_changed_(false)
{
    string scale_topic;

    // get topic name for subscriber, if not use default
    nh_.param<string>("scale_topic", scale_topic, SCALE_TOPIC);

    // get the rest of parameters
    nh_.param<double>("scale_sensitivity", scale_sensitivity_, double(SCALE_SENSITIVITY));
    nh_.param<int>("subset_size", subset_size_, int(SUBSET_SIZE));


    // Subscriber
    sub_scale_  = nh_.subscribe<const std_msgs::Float64&>(scale_topic, 1, &WeightChangeMonitor::scale_callback, this);

    // Publisher
    pub_scale_filtered_ =   nh_.advertise<ranger_librarian::WeightFiltered>("scale_filtered", 2);


    // set circular buffer capacity
    weight_measures.set_capacity(subset_size_);

}

/// DESTRUCTOR
WeightChangeMonitor::~WeightChangeMonitor()
{

}
/// CALLBACK
void WeightChangeMonitor::scale_callback(const std_msgs::Float64& msg) {

    //get new message data
    double weight = msg.data;

    double weight_stable_new;
    bool weight_equal = true;
    weight_changed_ = false;


    for (auto it:weight_measures) {

        double  weight_diff = weight - (it);

        bool two_frames_equal = std::abs(weight_diff) < scale_sensitivity_;

        weight_equal = weight_equal && two_frames_equal;

        if (!weight_equal)
            break;
    }

    // update circular buffer
    weight_measures.push_front(weight);

    if (weight_equal) {

        weight_stable_new = weight_median();


        double weight_stable_diff = weight_stable_new - weight_stable_;

        if (std::abs(weight_stable_diff) > scale_sensitivity_) {
            weight_change_ = weight_stable_diff;
            weight_changed_ = true;
            weight_stable_ = weight_stable_new;
            change_time_ = ros::Time::now();

        }


    }



    // create message
    ranger_librarian::WeightFiltered weight_msg;

    // assign data
    weight_msg.change_time = change_time_;
    weight_msg.weight_changed = weight_changed_;

    weight_msg.weight_stable = weight_stable_;
    weight_msg.weight_change = weight_change_;


    // publish
    pub_scale_filtered_.publish(weight_msg);





}

// returns median of buffered weight values
double WeightChangeMonitor::weight_median() {
    double median;
    int size = weight_measures.size();

    sort(weight_measures.begin(), weight_measures.end());

    if (size  % 2 == 0) {
        median = (weight_measures[size / 2 - 1] + weight_measures[size / 2]) / 2;
    } else {
        median = weight_measures[size / 2];
    }

    return median;
}


/// MAIN
/// Node publishes time (s) showing how long depth sensor is covered
int main(int argc, char **argv)
{
    ros::init(argc, argv, "weight_change_monitor_node");
    WeightChangeMonitor weight_monitor_node;

    ros::spin();

    return 0;
}
