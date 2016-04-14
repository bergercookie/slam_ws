#include "MathUtils.h"

double MathUtils::computeDistance( 
    const geometry_msgs::Pose2DConstPtr &a
    , isam::Node *b)
{
  double deltax, deltay, deltath;
  double distance;

  Eigen::Vector3d bvector = b->vector();

  deltax = fabs(a->x - bvector(0));
  deltay = fabs(a->y - bvector(1));

  distance = sqrt(pow(deltax, 2.0) + pow(deltay, 2.0));
  return distance;
}


