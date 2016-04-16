/**
 * TODO - What about docstrings in the .cpp file. Consult so, Google practices..
 */


// relative headers
#include "CallbackHandler.h"

/**
 * CallbachHandler Class Implementations
 */

/*****************************
 * CONSTRUCTORS & DESTRUCTORS
 *****************************/

CallbackHandler::CallbackHandler():
  noise_(isam::Information(100. * isam::eye(3))), // TODO change its position...
  time_thresh_(ros::Duration(slam_params::kOdometryTimeThresh))
{ 
  ROS_INFO_COND(slam_params::kPrintProgPos, "In the CalbackHander Constructor"); // TODO - add condition for printing the "In" messages

  // Initialize the objects on the heap
  slam_ptr = new isam::Slam;
  node_list = new std::vector<isam::Node*>;
  laser_scans = new std::vector<sensor_msgs::LaserScan*>;
  g_pose2d_list = new std::vector<geometry_msgs::Pose2D*>;
  timestamps_list = new std::vector<ros::Time*>;

  initialised_laser_scans_ = false;

  // put the first nodes in the graph
  initGraph();
}

CallbackHandler::~CallbackHandler()
{ 
  ROS_INFO_COND(slam_params::kPrintProgPos, "In the CalbackHander Destructor");
  // TODO - delete all the dynamically allocated objects / std::vector<..*>
}

/*****************************
 * CLASS MEMBER FUNCTIONS
 *****************************/

void CallbackHandler::checkOdometricConstraint(const geometry_msgs::Pose2DConstPtr& pose_in)
{
  ROS_INFO_COND(slam_params::kPrintProgPos, "In the checkOdometricConstraint fun..");
  // TODO - Add docstring here..
  if (!initialised_laser_scans_)
  {

    ROS_INFO_COND(slam_params::kPrintProgPos, "checkOdometricConstraint: initialising laser_scans_..");
    sensor_msgs::LaserScan* current_scan_ptr_ = new sensor_msgs::LaserScan;
    *current_scan_ptr_ = current_scan_;
    laser_scans->push_back(current_scan_ptr_);

    initialised_laser_scans_ = true;
  }

  else 
  {
    ROS_INFO_COND(slam_params::kPrintProgPos, "checkOdometricConstraint: Checking Constraints...................");

    ros::Time *cur_timestamp_ = new ros::Time();
    *cur_timestamp_ = ros::Time::now();

    /**
     * Get distance, rotation and time difference from the last inserted node.
     */

    isam::Node *last_node = node_list->back();
    ros::Time *last_timestamp_ = timestamps_list->back();

    double distance_, rot_;
    MathUtils::computeDifference(pose_in, last_node, &distance_, &rot_);
    ROS_INFO_STREAM("[Distance, Rotation] = [ " 
        << distance_ << "(m?), " << isam::rad_to_deg(rot_) << "(deg)]");

    time_diff_ = *cur_timestamp_ - *last_timestamp_;
    ROS_INFO_STREAM("Time difference: " << time_diff_);

    /** 
     * Decide on wether to insert new node
     */
    if ( (distance_ >= slam_params::kOdometryDistanceThresh) || 
        (rot_ >= isam::deg_to_rad(slam_params::kOdometryAngleThresh))  ||
        (time_diff_.toSec() >= slam_params::kOdometryTimeThresh)
       )  
    {
      ROS_INFO_COND(slam_params::kPrintProgPos,
          "***********************Adding new node..***********************");

      /** 
       * Initialize a temp Pose2d at the current pose estimate. Given the temp and
       * the previous node get the odometry constraint using *ominus* method
       * Add the slam graph node and the corresponding factor 
       */

      isam::Pose2d *temp_pose = new isam::Pose2d(pose_in->x, pose_in->y, pose_in->theta);

      isam::Pose2d_Node *last_node_casted = dynamic_cast<isam::Pose2d_Node*>(last_node);
      isam::Pose2d last_node_pose2d = last_node_casted->value();
      isam::Pose2d laser_odom = temp_pose->ominus(last_node_pose2d);
      ROS_INFO_STREAM("Constraint to add: " << laser_odom);

      /** 
       * Put everything to slam_ptr
       */
      isam::Pose2d_Node *next_node = new isam::Pose2d_Node();
      node_list->push_back(next_node);
      slam_ptr->add_node(next_node);

      isam::Pose2d_Pose2d_Factor *constraint = 
        new isam::Pose2d_Pose2d_Factor(
            last_node_casted
            , next_node
            , laser_odom
            , noise_);
      // TODO - consider initializing every pointer only once in the header file
      slam_ptr->add_factor(constraint);

      // add the current laser scan
      // We assume that the current laser scan corresponds to the pose we are
      // investigating inside this function

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

      // print the new node
      ROS_INFO_STREAM("Added new node: " << *next_node);

    }
  }
}

void CallbackHandler::getCurLaserScan(const sensor_msgs::LaserScanPtr& laser_in)
{
  ROS_INFO_COND(slam_params::kPrintProgPos, "In the getCurLaserScan fun..");

  // The goal is to keep the contents of the laser scan after the end of the
  // function, so DO NOT use a pointer as it will be a dangling pointer once
  // this function is out of scope. Instead use a real object in the stack
  current_scan_ = *laser_in;

}

void CallbackHandler::postGraphProperties(const ros::Publisher &graph_props_pub, const int robot_id) const
{
  ROS_INFO_COND(slam_params::kPrintProgPos, "In the postGraphProperties fun");

  isam_slam::GraphProperties graph_props;
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

  ROS_INFO_COND(slam_params::kPrintProgPos, "Publishing the graph_props");
  graph_props_pub.publish(graph_props);

}

void CallbackHandler::initGraph(void) {
  ROS_INFO_COND(slam_params::kPrintProgPos, "Inside the initGraph fun..");
  // start working with nodes and factors here
  isam::Pose2d prior_origin(0., 0., 0.);

  // pose nodes and constraints
  isam::Pose2d_Node *a0 = new isam::Pose2d_Node();
  slam_ptr->add_node(a0);
  node_list->push_back(a0);

  ros::Time *stamp_start = new ros::Time();
  *stamp_start = ros::Time::now();
  //ROS_INFO_STREAM("initGraph: current time: " << *stamp_start);
  timestamps_list->push_back(stamp_start);

  // TODO - Putting noise in the slam_params namespace raises a linker error
  isam::Pose2d_Factor *p_a0 = new isam::Pose2d_Factor(a0, prior_origin, noise_);
  slam_ptr->add_factor(p_a0);

  // add the current g_pose to the g_pose2d_list
  geometry_msgs::Pose2D *g_pose2d_ptr = new geometry_msgs::Pose2D;
  g_pose2d_ptr->x = 0.; g_pose2d_ptr->y = 0.; g_pose2d_ptr->theta = 0.;
  g_pose2d_list->push_back(g_pose2d_ptr);

  ROS_INFO("initGraph: Added g_pose = " );
  ROS_INFO_STREAM(*g_pose2d_list->back());

}
