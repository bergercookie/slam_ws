// STL
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <scan_match/GraphProperties.h>

#include <isam/isam.h>
#include <Eigen/LU>

#include "CallbackHandler.h"
#include "slam_params.h"

// using
using namespace isam;
using namespace Eigen;


Noise noise = SqrtInformation(10. * eye(3));

/** 
 * Supplementary Functions
 *
 */

/**
 * init_graph initializes the empty Slam object and the corresponding
 * nodes/time vectors
 */
//void init_graph
  //(
  //Slam *slam
  //, std::vector<Node*> *node_list
  //, std::vector<ros::Time*> *node_stamps
  //, std::vector<geometry_msgs::Pose2D*> *g_pose2d_list
  //)
//{
  ////ROS_INFO("In init_graph fun");
  
  //// start working with nodes and factors here
  //Pose2d prior_origin(0., 0., 0.);

  //// pose nodes and constraints
  //Pose2d_Node *a0 = new Pose2d_Node();
  //slam->add_node(a0);
  //node_list->push_back(a0);

  //ros::Time *stamp_start = new ros::Time();
  //*stamp_start = ros::Time::now();
  ////ROS_INFO_STREAM("init_graph: current time: " << *stamp_start);
  //node_stamps->push_back(stamp_start);

  //// TODO - Putting noise in the slam_params namespace raises a linker error
  //Pose2d_Factor *p_a0 = new Pose2d_Factor(a0, prior_origin, noise);
  //slam->add_factor(p_a0);
  
  //// add the current g_pose to the g_pose2d_list
  //geometry_msgs::Pose2D *g_pose2d_ptr = new geometry_msgs::Pose2D;
  //g_pose2d_ptr->x = 0.; g_pose2d_ptr->y = 0.; g_pose2d_ptr->theta = 0.;
  //g_pose2d_list->push_back(g_pose2d_ptr);

  //ROS_INFO_STREAM("init_graph: Added g_pose = " << *g_pose2d_list->back());


//}


/**
 * MAIN
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


  ros::Publisher graph_props_pub = nh.advertise<scan_match::GraphProperties>(
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

