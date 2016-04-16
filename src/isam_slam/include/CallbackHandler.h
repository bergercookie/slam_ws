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
 * Class for handling callbacks in a uniform and structured manner
 * TODO - Include docstring
 */
class CallbackHandler 
{
  public:
    CallbackHandler(void);
    ~CallbackHandler();

    /**
     * CallbackHandler::checkOdometricConstraint
     *
     * Called when a new pose is available from the laser_scan_matcher. 
     * If the distance or the degrees between the current estimated pose and the
     * previous inserted node is greater than the corresponding fixed thresholds
     * then add a new node to the graph.
     */
    void checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in);

    /**
     * CallbackHandler::getCurLaserScan
     *
     * Fetch the newest laser scan from the corresponding topic and save it locally
     * so it can be accessed from within the CallbackHandler
     */
    void getCurLaserScan(const sensor_msgs::LaserScanPtr& laser_in);

    /**
     * CallbackHandler::initGraph
     *
     * initGraph initializes the Slam object and the corresponding
     * nodes/time vectors
     */
    void initGraph(void);

    /** 
     * CallbackHandler::postGraphProperties
     *
     * Factor for posting the current graph Properties to the corresponding /graphProps topic
     * Function is designed to be as generic as possible, so only a reference to
     * the ros::Publisher and a robot id is given
     *
     */
    void postGraphProperties(const ros::Publisher &graph_props_pub, const int robot_id) const;

    // TODO - talk about what should be private and what should be public
    isam::Slam *slam_ptr;

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
