#ifndef CALLBACK_HANDLER_H

#define CALLBACK_HANDLER_H

// STL
#include <vector>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>


// iSAM
#include <isam/isam.h>
#include <Eigen/LU>


// PROJECT
#include <isam_slam/GraphProperties.h>
#include "MathUtils.h"
#include "slam_params.h"


/** 
 * Class for handling callbacks in a uniform and structured way
 * TODO - Include docstring
 */
class CallbackHandler 
{
  public:
    CallbackHandler(void);
    ~CallbackHandler();

    void checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in);
    void getCurLaserScan(const sensor_msgs::LaserScanPtr& laser_in);
    void postGraphProperties(const ros::Publisher &graph_props_pub, const int robot_id) const;
    void initGraph(void);

    Slam *slam_ptr;

    std::vector<isam::Node*> *node_list;
    std::vector<sensor_msgs::LaserScan*> *laser_scans;
    std::vector<geometry_msgs::Pose2D*> *g_pose2d_list;
    std::vector<ros::Time*> *timestamps_list;

  protected:

  private:
    isam::Noise noise_;

    sensor_msgs::LaserScan current_scan_;
    ros::Duration time_thresh_; // maximum time duration between successive nodes
    ros::Duration time_diff_; // maximum time duration between successive nodes

    double distance_;
    double twist_;

    bool initialised_laser_scans_; // boolean to know if the laser_scans vector contains at least one element
    
};


#endif /* end of include guard: CALLBACK_HANDLER_H */
