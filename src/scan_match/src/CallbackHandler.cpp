// relative headers
#include "CallbackHandler.h"

// using
using namespace isam;
using namespace Eigen;

/**
 * class containing trivial mathematical utilities
 */


/**
 * CallbachHandler Class Implementations
 */

CallbackHandler::CallbackHandler(Slam *slamPtr, 
    std::vector<Node*> *node_l, 
    std::vector<ros::Time*> *timestamps_l):
slam_ptr(slamPtr),
node_list(node_l),
timestamps_list(timestamps_l),
noise_(Information(100. * eye(3))),
time_thresh_(ros::Duration(slam_params::kOdometryTimeThresh))
{ }

CallbackHandler::~CallbackHandler()
{ }


/**
 * Called when a new pose is available from the laser_scan_matcher. 
 * If the distance or the degrees between the current estimated pose and the
 * previous inserted node is greater than the corresponding fixed thresholds
 * then add a new node to the graph.
 */

void CallbackHandler::checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in)
{
  // TODO - Remove these
  ROS_INFO("------------------------------------------");
  ROS_INFO("Pose2D captured: (%.2f, %.2f, %.2f)", pose_in->x, pose_in->y, pose_in->theta);

  ros::Time *cur_timestamp_ = new ros::Time();
  *cur_timestamp_ = ros::Time::now();
 
  /**
   * Get distance, twist and time difference from the last inserted node.
   */

  Node *last_node = node_list->back();
  ros::Time *last_timestamp_ = timestamps_list->back();

  ROS_INFO("last node: (%.2f, %.2f, %.2f)", 
      last_node->vector()(0), last_node->vector()(1), last_node->vector()(2));
  ROS_INFO_STREAM("Last time:" << *last_timestamp_);

  distance_ = MathUtils::computeDistance(pose_in, last_node);
  ROS_INFO("Distance from last node = %.2f", distance_);
  
  twist_ = standardRad(last_node->vector()(2) - pose_in->theta); 
  ROS_INFO("Twist from last node = %.2f rad", twist_);

  time_diff_ = *cur_timestamp_ - *last_timestamp_;
  ROS_INFO_STREAM("Time difference: " << time_diff_);



  /** 
   * Decide on wether to insert new node
   */
  if ( (distance_ >= slam_params::kOdometryDistanceThresh) || 
       (twist_ >= deg_to_rad(slam_params::kOdometryAngleThresh))  ||
       (time_diff_.toSec() >= slam_params::kOdometryTimeThresh)
     )  
  {
    ROS_INFO("***Adding new node..***");
    
    /** 
     * Initialize a temp Pose2d at the current pose estimate. Given the temp and
     * the previous node get the odometry constraint using *ominus* method
     * Add the slam graph node and the corresponding factor 
     */

    Pose2d *temp_pose = new Pose2d(pose_in->x, pose_in->y, pose_in->theta);

    Pose2d_Node *last_node_casted = dynamic_cast<Pose2d_Node*>(last_node);
    Pose2d last_node_pose2d = last_node_casted->value();
    Pose2d laser_odom = temp_pose->ominus(last_node_pose2d);
    ROS_INFO_STREAM("Constraint to add: " << laser_odom);

    timestamps_list->push_back(cur_timestamp_);

    /** 
     * Put everything to slam_ptr
     */
    Pose2d_Node *next_node = new Pose2d_Node();
    node_list->push_back(next_node);
    slam_ptr->add_node(next_node);

    Pose2d_Pose2d_Factor *constraint = new Pose2d_Pose2d_Factor(last_node_casted, 
        next_node, laser_odom, noise_);
    // TODO - consider initializing every pointer only once in the header file
    slam_ptr->add_factor(constraint);
    
    // print the new node
    ROS_INFO_STREAM("Added new node: " << *next_node);

  }

}

