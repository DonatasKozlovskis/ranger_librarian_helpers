// ROS
#include <ros/ros.h>

// other
#include "label_reader.h"
#include "ranger_librarian.h"



/// MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ranger_librarian_node");
    RangerLibrarian rl;
    rl.run();
}
