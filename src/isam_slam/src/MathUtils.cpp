#include "MathUtils.h"

/** 
 * MathUtils::computeDifference
 *
 * Compute and return the difference (distance and rotation) between the two
 * given poses.
 */
void MathUtils::computeDifference( 
    const geometry_msgs::Pose2DConstPtr &a
    , const isam::Node *b
    , double* distance
    , double* rot)
{
  ROS_INFO_COND(slam_params::kPrintProgPos, "In computeDifference fun.");

  double delta_x, delta_y, delta_th;

  Eigen::Vector3d bvector = b->vector();

  delta_x = fabs(a->x - bvector(0));
  delta_y = fabs(a->y - bvector(1));
  delta_th = fabs(isam::standardRad(a->theta - bvector(2)));

  *distance = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
  *rot = delta_th;

  return;
}


