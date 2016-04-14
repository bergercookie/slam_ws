// STL
#include <vector>
#include <cmath>

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

  const int robot_id = 1; // TODO - Assign this dynamically

  ros::init(argc, argv, "pose_subscriber");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

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
  ROS_INFO("Pose Subscriber initialised.");

  // subscriber for reading the current  LaserScan
  ros::Subscriber laser_sub = nh.subscribe(
      "/scan", 1000,
      &CallbackHandler::getCurLaserScan, &cb_handler);
  ROS_INFO("Laser Subscriber initialised.");


  ros::Publisher graph_props_pub = nh.advertise<isam_slam::GraphProperties>(
      "graph_properties", 100, /* latch= */ true);

  /** 
   * Main ROS loop
   */

  while (ros::ok()) {

    // graph-properties publisher
    cb_handler.postGraphProperties(graph_props_pub, robot_id);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

