#ifndef SLAM_PARAMS_H
#define SLAM_PARAMS_H

// STL
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/console.h>
#include <std_msgs/String.h>


// iSAM
#include <isam/isam.h>
#include <Eigen/LU>


namespace slam_params {

/** 
 * Decide when to insert a new node in the graph
 */
const double kOdometryDistanceThresh = 1.0;
const double kOdometryAngleThresh = 45.0; // degrees
const double kOdometryTimeThresh = 5.0; // max seconds


/**
 * Level of console output
 */

// TODO - add silent flag
//
// TODO - make the following be just constant booleans
const bool kPrintProgPos = false;
const bool kPrintRosNode = true;
const bool kPrintInitSubPub = true;

//kPrintAll = true; // TODO -  implement this
//kPrintNothing = false; // TODO - implement this

} // slam_params

#endif /* end of include guard: SLAM_PARAMS_H */
