#ifndef CALLBACK_HANDLER_H

#define CALLBACK_HANDLER_H

// STL
#include <vector>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/console.h>
#include <std_msgs/String.h>


// iSAM
#include <isam/isam.h>
#include <Eigen/LU>


// CUSTOM
#include "MathUtils.h"
#include "slam_params.h"

// using
using namespace isam;
using namespace Eigen;

/** 
 * Class for handling callbacks in a uniform and structured way
 */
class CallbackHandler 
{
  public:
    CallbackHandler(Slam *slamPtr, 
      std::vector<Node*> *node_l, \
      std::vector<ros::Time*> *timestamps_l);
    virtual ~CallbackHandler();

    void checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in);

    Slam *slam_ptr;

    std::vector<Node*> *node_list;
    std::vector<ros::Time*> *timestamps_list;

  private:

    Noise noise_;

    double distance_;
    double twist_;

    double diff_x_, diff_y_, diff_th_;

    ros::Duration time_thresh_; // maximum time duration between successive nodes
    ros::Duration time_diff_; // maximum time duration between successive nodes

    
};


#endif /* end of include guard: CALLBACK_HANDLER_H */
