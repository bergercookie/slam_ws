// STL
#include <vector>
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>


// iSAM
#include <isam/isam.h>
#include <Eigen/LU>

// PROJECT
#include <isam_slam/GraphProperties.h>
#include "CallbackHandler.h"
#include "slam_params.h"

/**
 * MAIN
 * TODO - Include descriptive docstring
 */
int main(int argc, char **argv)
{
  /**
   * Initialization
   */

  std::string rosnode_name = "isam_slam";
  const int robot_id = 1; // TODO - Assign this dynamically

  ros::init(argc, argv, rosnode_name);
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);
  ROS_INFO_STREAM_COND(slam_params::kPrintRosNode, rosnode_name << "Initialised.");

  // TODO - waiting for X seconds helps to see the correct ros::Time::now() -
  // otherwise it returns zero ?!
  // find a better solution
  ros::Duration(1.0).sleep();

  CallbackHandler cb_handler;

  /**
   * Publishers & Subscribers
   */

  // subscriber for reading the current Pose2D
  ros::Subscriber pose2d_sub = nh.subscribe(
      "/pose2D", 1000,
      &CallbackHandler::checkOdometricConstraint, &cb_handler);
  ROS_INFO_COND(slam_params::kPrintInitSubPub, "Pose Subscriber initialised.");

  // subscriber for reading the current  LaserScan
  ros::Subscriber laser_sub = nh.subscribe(
      "/scan", 1000,
      &CallbackHandler::getCurLaserScan, &cb_handler);
  ROS_INFO_COND(slam_params::kPrintInitSubPub, "Laser Subscriber initialised.");

  // publisher for graph properties
  ros::Publisher graph_props_pub = nh.advertise<isam_slam::GraphProperties>(
      "graph_properties", 100, /* latch= */ true);
  ROS_INFO_COND(slam_params::kPrintInitSubPub, "GraphProperties Publisher initialised.");

  /** 
   * Main ROS loop
   */
  while (ros::ok()) {

    ROS_INFO_COND(slam_params::kPrintProgPos, "Started Main ROS loop");;

    // graph-properties publisher
    cb_handler.postGraphProperties(graph_props_pub, robot_id);
    ROS_INFO_ONCE("Posted First GraphProperties Message"); // TODO - Can this be with COND too?

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

