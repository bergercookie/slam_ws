/**
 * This tutorial demonstrates simple subscription and reception of messages
 * in ROS.
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <scan_match/Person.h>

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
void init_graph(Slam *slam, std::vector<Node*> *node_list, std::vector<ros::Time*> *node_tstamps)
{
  // start working with nodes and factors here
  Pose2d prior_origin(0., 0., 0.);

  // pose nodes and constraints
  Pose2d_Node *a0 = new Pose2d_Node();
  slam->add_node(a0);
  node_list->push_back(a0);


  ros::Time *tstamp_start = new ros::Time();
  *tstamp_start = ros::Time::now();
  ROS_INFO_STREAM("init_graph: current time: " << *tstamp_start);
  node_tstamps->push_back(tstamp_start);

  // TODO - Putting noise in the slam_params namespace raises a linker error
  Pose2d_Factor *p_a0 = new Pose2d_Factor(a0, prior_origin, noise);
  slam->add_factor(p_a0);

}


/**
 * MAIN
 */

int main(int argc, char **argv)
{
  /**
   * Initialization
   */

  ros::init(argc, argv, "pose_subscriber");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);

  // TODO - waiting for X seconds helps to see the correct ros::Time::now() -
  // otherwise it returns zero ?!
  // find a better solution
  ros::Duration(1.0).sleep();

  Slam *slam = new Slam;

  std::vector<Node*> node_list;
  std::vector<ros::Time*> node_tstamps;

  // initialize the graph
  init_graph(slam, &node_list, &node_tstamps);
  

  // initialize a Subscribing for reading the current pose
  CallbackHandler cb_handler(slam, &node_list, &node_tstamps);
  ros::Subscriber sub = nh.subscribe("/pose2D", 1000, \
      &CallbackHandler::checkOdometricConstraint, &cb_handler);
  ROS_INFO("Pose subscriber initialised.");

  // initialize a standard message publisher
  ros::Publisher std_pub = nh.advertise<std_msgs::String>("std_topic", 5, true);
  ros::Publisher custom_pub = nh.advertise<scan_match::Person>("custom_topic", 5, true);

  while (ros::ok()) {

/*    // TODO - remove this after done*/
    //// Publishing a std message..
    //std_msgs::String str;
    //str.data = "hello world!";
    //std_pub.publish(str);

    // TODO - remove this after done
    // custom message - Person
    scan_match::Person a_person;
    a_person.name = "Kalimeras";
    a_person.height = 1.89;
    a_person.width = 3.0;
    a_person.age = 22;
    a_person.quote = "holaa";


    // graph-properties publisher
    // TODO

    ros::spinOnce();
    loop_rate.sleep();

  

  }

  return 0;
}


