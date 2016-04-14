#ifndef CALLBACK_HANDLER_H

#define CALLBACK_HANDLER_H

// STL
#include <vector>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <scan_match/GraphProperties.h>


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
    CallbackHandler
      (
        Slam *slam_ptr_in
      , std::vector<Node*> *node_list_in
      , std::vector<sensor_msgs::LaserScan*> *laser_scans_in
      , std::vector<geometry_msgs::Pose2D*> *g_pose2d_list_in
      , std::vector<ros::Time*> *timestamps_list_in
      );

    virtual ~CallbackHandler();

    void checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in);
    void getCurLaserScan(const sensor_msgs::LaserScanPtr& laser_in);
    void postGraphProperties(ros::Publisher &graph_props_pub, const int robot_id) const;
    void postString(ros::Publisher &std_pub); // TODO - remove this

    Slam *slam_ptr;

    std::vector<Node*> *node_list;
    std::vector<sensor_msgs::LaserScan*> *laser_scans;
    std::vector<geometry_msgs::Pose2D*> *g_pose2d_list;
    std::vector<ros::Time*> *timestamps_list;

  private:
    Noise noise_;

    sensor_msgs::LaserScan current_scan_;
    ros::Duration time_thresh_; // maximum time duration between successive nodes
    ros::Duration time_diff_; // maximum time duration between successive nodes

    double distance_;
    double twist_;
    double diff_x_, diff_y_, diff_th_;

    bool initialised_laser_scans_; // boolean to know if the laser_scans vector contains at least one element
    
};


#endif /* end of include guard: CALLBACK_HANDLER_H */
