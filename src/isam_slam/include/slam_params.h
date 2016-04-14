#ifndef SLAM_PARAMS_H
#define SLAM_PARAMS_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/console.h>
#include <std_msgs/String.h>


// iSAM
#include <isam/isam.h>
#include <Eigen/LU>


// using 
using namespace isam;
using namespace Eigen;



namespace slam_params {


/** 
 * Decide when to insert a new node in the graph
 */
const double kOdometryDistanceThresh = 3.0;
const double kOdometryAngleThresh = 10.0; // degrees
const double kOdometryTimeThresh = 5.0; // max seconds



} // slam_params

#endif /* end of include guard: SLAM_PARAMS_H */
