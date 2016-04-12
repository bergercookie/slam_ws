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

// using
using namespace isam;
using namespace Eigen;


class MathUtils
{
  public:
    MathUtils() {}
    ~MathUtils() {}

    static double computeDistance(const geometry_msgs::Pose2DConstPtr &a, Node *b);

  private:

};

#endif /* end of include guard: MATHUTILS_H */
