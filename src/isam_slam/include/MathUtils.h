#ifndef MATHUTILS_H
#define MATHUTILS_H

// STL
#include <cmath>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/console.h>
#include <std_msgs/String.h>

// iSAM
#include <isam/isam.h>
#include <Eigen/LU>

// PROJECT
#include <slam_params.h>


/**
 * TODO - Include docstring..
 */
class MathUtils
{
  public:
    MathUtils() {}
    ~MathUtils() {}

    static void computeDifference(
        const geometry_msgs::Pose2DConstPtr &a
        , const isam::Node *b
        , double* distance
        , double* rot);

  private:

  protected:

};

#endif /* end of include guard: MATHUTILS_H */
