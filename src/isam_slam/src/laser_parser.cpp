/**
 * laser_paraser
 *
 * The purpose of the node is to parse the laser_scan measurements
 * by subscribing to the corresponding /scan topic and
 * to calculate and publish the relative 2d pose of the robot.
 */

// STL
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

// ROS ADDITIONAL PKGS
#include <laser_scan_matcher/laser_scan_matcher.h> 

#include "slam_params.h"

int main(int argc, char **argv)
{
  std::string rosnode_name = "laser_parser";
  ros::init(argc, argv, rosnode_name);
  ROS_INFO_STREAM_COND(slam_params::kPrintRosNode, 
      rosnode_name << "initialised");


  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("nh_private");

  scan_tools::LaserScanMatcher lsm(nh, nh_priv);

  ros::spin();
  return 0;
}

