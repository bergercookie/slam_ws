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

/*****************************
 * CONSTRUCTORS & DESTRUCTORS
 *****************************/

CallbackHandler::CallbackHandler(
    Slam *slam_ptr_in, 
    std::vector<Node*> *node_list_in, 
    std::vector<sensor_msgs::LaserScan*> *laser_scans_in, // TODO change the in names to be consistent
    std::vector<geometry_msgs::Pose2D*> *g_pose2d_list_in,
    std::vector<ros::Time*> *timestamps_list_in/*, sensor_msgs::LaserScan cur_laser_s */):

slam_ptr(slam_ptr_in),
node_list(node_list_in),
laser_scans(laser_scans_in),
g_pose2d_list(g_pose2d_list_in),
timestamps_list(timestamps_list_in),

noise_(Information(100. * eye(3))), // TODO change its position...
time_thresh_(ros::Duration(slam_params::kOdometryTimeThresh))
{ 
  //ROS_INFO("In the CalbackHander Constructor");

  initialised_laser_scans_ = false;
}

CallbackHandler::~CallbackHandler()
{ 
  //ROS_INFO("In the CalbackHander Destructor");
  // TODO - delete all the dynamically allocated objects / std::vector<..*>
}

/*****************************
 * CLASS MEMBER FUNCTIONS
 *****************************/

/**
 * CallbackHandler::checkOdometricConstraint
 *
 * Called when a new pose is available from the laser_scan_matcher. 
 * If the distance or the degrees between the current estimated pose and the
 * previous inserted node is greater than the corresponding fixed thresholds
 * then add a new node to the graph.
 */
void CallbackHandler::checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in)
{
  // TODO - Add docstring here..
  if (!initialised_laser_scans_)
  {

    ROS_INFO("CallbackHandler::checkOdometricConstraint: initialising laser_scans_..");
    sensor_msgs::LaserScan* current_scan_ptr_ = new sensor_msgs::LaserScan;
    *current_scan_ptr_ = current_scan_;
    laser_scans->push_back(current_scan_ptr_);

    initialised_laser_scans_ = true;
  }

  else 
  {
    // TODO Check this part again..  seems like it adds nodes for no reason at
    // some point
    
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
      
      // add the current laser scan
      // We assume that the current laser scan corresponds to the pose we are
      // investigating inside this function

      //ROS_INFO("+++++++++++++++++++++");
      //ROS_INFO_STREAM("current_scan_");
      //ROS_INFO_STREAM("angle_min =" << current_scan_.angle_min);
      //ROS_INFO_STREAM("angle_max =" << current_scan_.angle_max);
      //ROS_INFO_STREAM("angle_increment =" << current_scan_.angle_increment);
      //ROS_INFO_STREAM("time_increment =" << current_scan_.time_increment);
      //ROS_INFO_STREAM("scan_time =" << current_scan_.scan_time);
      
      sensor_msgs::LaserScan* current_scan_ptr_ = new sensor_msgs::LaserScan;
      *current_scan_ptr_ = current_scan_;
      laser_scans->push_back(current_scan_ptr_);
    
      //add the current timestamp
      timestamps_list->push_back(cur_timestamp_);

      // add the current geometry pose (needed for passing the graph props
      // message)
      geometry_msgs::Pose2D *g_pose2D_ptr_ = new geometry_msgs::Pose2D;
      *g_pose2D_ptr_ = *pose_in;

      g_pose2d_list->push_back(g_pose2D_ptr_);
      ROS_INFO_STREAM("g_pose2D_ptr_ = " << *g_pose2D_ptr_);

    }
  }
}

/**
 * CallbackHandler::getCurLaserScan
 *
 * Fetch the newest laser scan from the corresponding topic and save it locally
 * so it can be accessed from within the CallbackHandler
 */
void CallbackHandler::getCurLaserScan(const sensor_msgs::LaserScanPtr& laser_in)
{
  //ROS_INFO("In the getCurLaserScan fun");
  
  // The goal is to keep the contents of the laser scan after the end of the
  // function, so DO NOT use a pointer as it will be a dangling pointer once
  // this function is out of scope. Instead use a real object in the stack
  current_scan_ = *laser_in;

}

/** 
 * CallbackHandler::postGraphProperties
 *
 * Factor for posting the current graph Properties to the corresponding /graphProps topic
 * Function is designed to be as generic as possible, so only a reference to
 * the ros::Publisher and a robot id is given
 *
 */
void CallbackHandler::postGraphProperties(ros::Publisher &graph_props_pub, const int robot_id) const
{
  ROS_INFO("In the postGrphProperties fun");

  scan_match::GraphProperties graph_props;
  graph_props.robot_id = robot_id;

  //graph_props.node_list = *g_pose2d_list;
  for (std::vector<geometry_msgs::Pose2D*>::iterator it = g_pose2d_list->begin(); 
      it != g_pose2d_list->end(); ++it) {
    graph_props.node_list.push_back(**it);
  }

  //graph_props.laser_scans = *laser_scans;
  for (std::vector<sensor_msgs::LaserScan*>::iterator it = laser_scans->begin(); 
      it != laser_scans->end(); ++it) {
    graph_props.laser_scans.push_back(**it);
  }

  //graph_props.timestamps = *timestamps_list;
  for (std::vector<ros::Time*>::iterator it = timestamps_list->begin(); 
      it != timestamps_list->end(); ++it) {
    std_msgs::Time next_timestamp;
    next_timestamp.data = **it;
    graph_props.timestamps.push_back(next_timestamp);
  }

  ROS_INFO("Publishing the graph_props");
  graph_props_pub.publish(graph_props);

}

// TODO - remove this
void CallbackHandler::postString(ros::Publisher &std_pub)
{
  std_msgs::String a_string;
  a_string.data = "kalimera";

  std_pub.publish(a_string);
  
}
