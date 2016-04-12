/**
 * The purpose of this program is to load the laser_scan and tf measurements
 * from a provided rosbag (or later by subscribing to a topic) and after that,
 * to calculate and publish the relative 2d pose of the robot.
 */

#include <ros/ros.h>
// include the folder - start from the /opt/ros/include folder and move forward
#include <laser_scan_matcher/laser_scan_matcher.h> 
#include <rosbag/bag.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_parser");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("nh_private");

    scan_tools::LaserScanMatcher lsm(nh, nh_priv);


    ros::spin();
    return 0;
}

