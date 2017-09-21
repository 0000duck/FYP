//TODO: Add yaw planning
//TODO: Fix hover case
//TODO: Fix short segment crashes
//TODO: Find correct segment times based on initial condition
//TODO: Corridor constraint
//TODO: Fix reuse time issue, recompute time is the trajectory length is scaled siginficantly

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <dynamic_reconfigure/server.h>
#include <trajectory_generator/TrajectoryGeneratorUIConfig.h>
#include "pose_utils.h"
#include "polynomial_trajectory_generator.h"

using namespace arma;
using namespace std;

// ROS
ros::Publisher pubc;
ros::Publisher pubp;
ros::Publisher pubt;
quadrotor_msgs::PositionCommand position_cmd;
trajectory_generator::TrajectoryGeneratorUIConfig config;
bool isConfig = false;

// Control Mode FSM
#define TAKEOFF_MODE   0
#define HOVER_MODE     1
#define VELOCITY_MODE  2
#define POSITION_MODE  3
int control_mode = TAKEOFF_MODE;

// State
bool is_odom       = false;                // First odom received
colvec         p   = zeros<colvec>(3);     // Current state & time
colvec         v   = zeros<colvec>(3); 
double         yaw = 0;
ros::Time      t;

// For takeoff mode
double         init_vel    = 0.8;       // Takeoff speed
double         init_height = 0.5;       // Takeoff height
double         init_yaw    = 0.0;
bool           is_init_yaw = false;

// For hover mode
double hover_des[4] = {0.0,0.0,0.0,0.0};

// For velocity mode
bool   is_rc        = true;
double rc_max_v     = 2.0;
double rc_max_w     = 15.0*M_PI/180.0;
double rc_des[4]    = {0.0,0.0,0.0,0.0};
double rc_reg[4]    = {0.0,0.0,0.0,0.0};
bool   is_rc_reg[4] = {false,false,false,false};

// For position mode
PolynomialTrajectoryGenerator traj_gen;               // Trajectory generator, in the odom frame
vector<colvec> waypoints;                             // Waypoints [x, y, z, yaw] in world frame
double         add_waypoint_d     = 1.0;              // Thresholds that determine whether to add intermediate waypoints
double         add_waypoint_theta = 30 * M_PI / 180;
colvec         correction         = zeros<colvec>(6); // SLAM correction SLAM = correction (+) odom
void           visualization();                       // Visualization

/* ************************* State feedback ************************* */
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  is_odom = true;  
  if (!is_odom || !is_rc)
    return;
  // Get time, position and velocity
  t    = msg->header.stamp;
  p(0) = msg->pose.pose.position.x;  
  p(1) = msg->pose.pose.position.y;  
  p(2) = msg->pose.pose.position.z;  
  v(0) = msg->twist.twist.linear.x;
  v(1) = msg->twist.twist.linear.y;
  v(2) = msg->twist.twist.linear.z;
  yaw  = tf::getYaw(msg->pose.pose.orientation);
  // Get position command based on modes
  if (control_mode == TAKEOFF_MODE)
  {
    if (!is_init_yaw)
    {
      is_init_yaw = true;
      init_yaw = yaw;
    }
    position_cmd.position.x     = 0;
    position_cmd.position.y     = 0;
    position_cmd.position.z     = p(2);    
    position_cmd.velocity.x     = 0;
    position_cmd.velocity.y     = 0;
    position_cmd.velocity.z     = init_vel;    
    position_cmd.acceleration.x = 0;
    position_cmd.acceleration.y = 0;
    position_cmd.acceleration.z = 0;  
    position_cmd.yaw            = init_yaw;
    position_cmd.yaw_dot        = 0;    
    if (fabs(p(2) - init_height) < 0.15)
    {
      hover_des[0] = 0;
      hover_des[1] = 0;    
      hover_des[2] = init_height;        
      hover_des[3] = init_yaw;    
      control_mode = HOVER_MODE;    
    }
  }
  else if (control_mode == HOVER_MODE)
  {
    position_cmd.position.x     = hover_des[0];
    position_cmd.position.y     = hover_des[1];
    position_cmd.position.z     = hover_des[2];    
    position_cmd.velocity.x     = 0;
    position_cmd.velocity.y     = 0;
    position_cmd.velocity.z     = 0;    
    position_cmd.acceleration.x = 0;
    position_cmd.acceleration.y = 0;
    position_cmd.acceleration.z = 0;  
    position_cmd.yaw            = hover_des[3];
    position_cmd.yaw_dot        = 0;     
  }
  else if (control_mode == VELOCITY_MODE)
  {
    double dt = 0.05;
    position_cmd.position.x     = (is_rc_reg[0])?rc_reg[0]:(p(0) + rc_des[0]*dt);
    position_cmd.position.y     = (is_rc_reg[1])?rc_reg[1]:(p(1) + rc_des[1]*dt);
    position_cmd.position.z     = (is_rc_reg[2])?rc_reg[2]:p(2);
    position_cmd.velocity.x     = rc_des[0];
    position_cmd.velocity.y     = rc_des[1];
    position_cmd.velocity.z     = rc_des[2];    
    position_cmd.acceleration.x = 0;
    position_cmd.acceleration.y = 0;
    position_cmd.acceleration.z = 0;  
    position_cmd.yaw            = (is_rc_reg[3])?rc_reg[3]:(yaw + rc_des[3]);
    position_cmd.yaw_dot        = 0;    
  }
  else if (control_mode == POSITION_MODE)
  {       
    traj_gen.GetPositionCommand(t, position_cmd);      
  } 
  position_cmd.header.stamp = ros::Time::now();
  pubc.publish(position_cmd);  
}


void rc_callback(const quadrotor_msgs::OutputData::ConstPtr &msg)
{
  if (msg->radio_channel[4])
    is_rc = true;
  if (!is_odom || !is_rc)
    return;
  // Get and convert rc to metric scale
  double scale = 255.0/2.0;
  rc_des[0] = -((double)msg->radio_channel[0] - scale) / scale * rc_max_v;
  rc_des[1] = -((double)msg->radio_channel[1] - scale) / scale * rc_max_v;  
  rc_des[2] =  ((double)msg->radio_channel[2] - scale) / scale * rc_max_v;    
  rc_des[3] = -((double)msg->radio_channel[3] - scale) / scale * rc_max_w;  
  // transform body xy velocity to world frame
  double _rc_des[2];  
  double c   = cos(yaw);
  double s   = sin(yaw);
  _rc_des[0] =  c * rc_des[0] - s * rc_des[1];
  _rc_des[1] =  s * rc_des[0] + c * rc_des[1];
  rc_des[0]  = _rc_des[0];
  rc_des[1]  = _rc_des[1];
  // Determine mode switch
  if (!(fabs(rc_des[0]) < rc_max_v / 10 && fabs(rc_des[1]) < rc_max_v / 10) && (control_mode == POSITION_MODE || control_mode == HOVER_MODE))
  {
    control_mode = VELOCITY_MODE;
    is_rc_reg[0] = false;
    is_rc_reg[1] = false;
    is_rc_reg[2] = false;    
    is_rc_reg[3] = false;    
  }
  // Hover handling
  if (control_mode == VELOCITY_MODE)
  {  
    if (fabs(rc_des[0]) < rc_max_v / 10 && fabs(rc_des[1]) < rc_max_v / 10 && !is_rc_reg[0] && !is_rc_reg[1])
    {
      is_rc_reg[0] = true;
      is_rc_reg[1] = true;    
      rc_reg[0]    = p(0);
      rc_reg[1]    = p(1);    
    }    
    else if (!(fabs(rc_des[0]) < rc_max_v / 10 && fabs(rc_des[1]) < rc_max_v / 10))
    {
      is_rc_reg[0] = false;
      is_rc_reg[1] = false;
    } 
    if (fabs(rc_des[2]) < rc_max_v / 5 && !is_rc_reg[2])
    {
      is_rc_reg[2] = true;
      rc_reg[2]    = p(2);
    }
    else if (!(fabs(rc_des[2]) < rc_max_v / 5))
    {
      is_rc_reg[2] = false;
    }
    if (fabs(rc_des[3]) < rc_max_w / 5 && !is_rc_reg[3])
    {
      is_rc_reg[3] = true;
      rc_reg[3]    = yaw;
    }
    else if (!(fabs(rc_des[3]) < rc_max_w / 5))
    {
      is_rc_reg[3] = false;
    }
  }
}

void trigger_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  if (!is_odom)
    return;
  if (control_mode == VELOCITY_MODE)
  {
    hover_des[0] = p(0);
    hover_des[1] = p(1);    
    hover_des[2] = p(2);        
    hover_des[3] = yaw;
    control_mode = HOVER_MODE;    
  }
}

void config_callback(trajectory_generator::TrajectoryGeneratorUIConfig &_config, uint32_t level) 
{
  isConfig = true;
  config = _config;
  rc_max_v = config.vel_kinematics;
  ROS_INFO("Config, Vel: %f %f, Acc: %f", config.vel_trajectory, config.vel_kinematics, config.acc_trajectory);
}

void waypoints_callback(const nav_msgs::Path::ConstPtr &msg)
{   
  if ( !is_odom || !(control_mode == HOVER_MODE || control_mode == POSITION_MODE) )
    return;
  // Decode waypoints
  waypoints.clear();
  waypoints.resize(msg->poses.size(), zeros<colvec>(4));
  for (unsigned int k = 0; k < msg->poses.size(); k++)
  {
    waypoints[k](0) = msg->poses[k].pose.position.x;
    waypoints[k](1) = msg->poses[k].pose.position.y;
    waypoints[k](2) = msg->poses[k].pose.position.z;    
    waypoints[k](3) = tf::getYaw(msg->poses[k].pose.orientation);
  }
  // Add waypoints if necessary
  int waypointCntBefore = waypoints.size(); 
  vector<colvec> waypointsadd;
  for (unsigned int k = 0; k < waypoints.size()-1; k++)
  {
    colvec p0    = ((k == 0)?p:waypoints[k-1]).rows(0,2);
    colvec p1    = waypoints[k].rows(0,2);
    colvec p2    = waypoints[k+1].rows(0,2);
    double D01   = norm(p1-p0, 2);
    double D12   = norm(p2-p1, 2);
    double theta = acos(dot(p2-p1, p1-p0) / D01 / D12);
    colvec wpt   = zeros<colvec>(4);
    if (theta > add_waypoint_theta && D01 > add_waypoint_d)
    {
      wpt.rows(0,2) = p1 - (p1 - p0) / D01 * add_waypoint_d / 2;
      wpt(3)        = waypoints[k](3);
      waypointsadd.push_back(wpt);
    }
    waypointsadd.push_back(waypoints[k]);
    if (theta > add_waypoint_theta && D12 > add_waypoint_d)
    {
      wpt.rows(0,2) = p1 + (p2 - p1) / D12 * add_waypoint_d / 2;
      wpt(3)        = waypoints[k](3);
      waypointsadd.push_back(wpt);        
    }
  }
  waypointsadd.push_back(waypoints.back());  
  waypoints = waypointsadd;
  int waypointCntAfter = waypoints.size();  
  // Transform waypoints to odom frame according to SLAM correction 
  vector<colvec> _waypoints = waypoints;
  for (unsigned int k = 0; k < waypoints.size(); k++)
  {
    colvec wpt = zeros<colvec>(6);
    wpt.rows(0,3) = waypoints[k];
    wpt = pose_update(pose_inverse(correction), wpt);
    _waypoints[k] = wpt.rows(0,3);
  }  
  // Generate trajectory, use the desired acceleration for the new init acceleration for the new trajectory
  quadrotor_msgs::PositionCommand cmd;            
  colvec a    = zeros<colvec>(3);    
  double dyaw = 0;       
  if (control_mode == POSITION_MODE)
  {
    traj_gen.GetPositionCommand(t, cmd);   
    a(0) = cmd.acceleration.x;
    a(1) = cmd.acceleration.y;
    a(2) = cmd.acceleration.z;    
    dyaw = cmd.yaw_dot;    
  }
  ros::Time t0 = ros::Time::now();      
  if (isConfig)
  {
    traj_gen.SetVelocity(config.vel_trajectory);
    traj_gen.SetAcceleration(config.acc_trajectory);
  }
  traj_gen.SetWaypoints(p, v, a, yaw, dyaw, t, _waypoints); 
  ros::Time t1 = ros::Time::now();    
  ROS_WARN("Trajectory Generated, Time (ms):  %f   WaypointCnt:  %d + %d \n", 
          (t1-t0).toSec()*1000, waypointCntBefore, waypointCntAfter - waypointCntBefore);       
  visualization();
  // Change mode
  control_mode = POSITION_MODE;      
}

void correction_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (!is_odom || control_mode != POSITION_MODE)
    return;
  // Get correction
  colvec q(4);
  correction(0)        = msg->pose.position.x;
  correction(1)        = msg->pose.position.y;
  correction(2)        = msg->pose.position.z;        
  q(0)                 = msg->pose.orientation.w;
  q(1)                 = msg->pose.orientation.x;
  q(2)                 = msg->pose.orientation.y;
  q(3)                 = msg->pose.orientation.z;            
  correction.rows(3,5) = R_to_ypr(quaternion_to_R(q)); 
  // Only ge-generate trajectory when there is points remaining  
  int idx = traj_gen.GetCurrWaypointIdx(t);  
  if (idx >= 0)
  {
    // Only keep and transform unreached waypoints to odom frame according to SLAM correction
    waypoints.erase(waypoints.begin(), waypoints.begin()+idx);
    vector<colvec> _waypoints = waypoints;
    for (unsigned int k = 0; k < waypoints.size(); k++)
    {
      colvec wpt = zeros<colvec>(6);
      wpt.rows(0,3) = waypoints[k];
      wpt = pose_update(pose_inverse(correction), wpt);
      _waypoints[k] = wpt.rows(0,3);
    }
    // Re-generate trajectory using remaining waypoints
    quadrotor_msgs::PositionCommand cmd;          
    traj_gen.GetPositionCommand(t, cmd);                
    colvec _p(3);
    _p(0) = cmd.position.x;
    _p(1) = cmd.position.y;
    _p(2) = cmd.position.z;  
    colvec _v(3);
    _v(0) = cmd.velocity.x;
    _v(1) = cmd.velocity.y;
    _v(2) = cmd.velocity.z;        
    colvec _a(3);
    _a(0) = cmd.acceleration.x;
    _a(1) = cmd.acceleration.y;
    _a(2) = cmd.acceleration.z;     
    ros::Time t0 = ros::Time::now();          
    traj_gen.SetWaypoints(_p, _v, _a, cmd.yaw, cmd.yaw_dot, t, _waypoints, true);          
    ros::Time t1 = ros::Time::now();    
    ROS_WARN("Trajectory Re-Generated, Time (ms):  %f \n", (t1-t0).toSec()*1000);       
    visualization();
  }
}

// Visualization all waypoints and trajectory in the odom frame
void visualization()
{
  vector<colvec> _waypoints = traj_gen.GetWaypoints();
  // Publish path for visualization
  nav_msgs::Path path;
  path.header.stamp    = t;
  path.header.frame_id = string("/map");
  geometry_msgs::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position.x  = p(0);
  pose.pose.position.y  = p(1);
  pose.pose.position.z  = p(2);        
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  path.poses.push_back(pose);
  for (unsigned int k = 0; k < _waypoints.size(); k++)
  {
    pose.pose.position.x  = _waypoints[k](0);
    pose.pose.position.y  = _waypoints[k](1);
    pose.pose.position.z  = _waypoints[k](2);        
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(_waypoints[k](3));
    path.poses.push_back(pose);    
  }
  pubp.publish(path);
  // Publish computed trajectory with 0.01s time increment
  nav_msgs::Path traj;
  traj.header.stamp    = t;
  traj.header.frame_id = string("/map");
  geometry_msgs::PoseStamped trajp;
  quadrotor_msgs::PositionCommand cmd;    
  trajp.header = traj.header;
  ros::Time tc = t;
  ros::Time ts = traj_gen.GetFinalTime();  
  while ((ts - tc).toSec() > 0)
  {
    traj_gen.GetPositionCommand(tc, cmd);        
    trajp.pose.position.x  = cmd.position.x;
    trajp.pose.position.y  = cmd.position.y;
    trajp.pose.position.z  = cmd.position.z;            
    trajp.pose.orientation = tf::createQuaternionMsgFromYaw(cmd.yaw);
    traj.poses.push_back(trajp);
    tc += ros::Duration(0.01);
  }
  pubt.publish(traj); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle n("~");
  
  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  position_cmd.header.frame_id = "/" + quadrotor_name;  
  
  n.param("gains/pos/x", position_cmd.kx[0], 3.7);
  n.param("gains/pos/y", position_cmd.kx[1], 3.7);
  n.param("gains/pos/z", position_cmd.kx[2], 5.2);
  n.param("gains/vel/x", position_cmd.kv[0], 2.4);
  n.param("gains/vel/y", position_cmd.kv[1], 2.4);
  n.param("gains/vel/z", position_cmd.kv[2], 3.0);  
  n.param("init_vel"   , init_vel          , 0.3);        
  n.param("init_height", init_height       , 0.5);    
  n.param("add_waypoint_dist_thr" , add_waypoint_d     , 1.0);    
  n.param("add_waypoint_theta_thr", add_waypoint_theta , 30*M_PI/180);   
  
  ros::Subscriber sub1 = n.subscribe("odom",       10, odom_callback, ros::TransportHints().tcpNoDelay());        
  ros::Subscriber sub2 = n.subscribe("rc",         10, rc_callback, ros::TransportHints().tcpNoDelay());        
  ros::Subscriber sub3 = n.subscribe("trigger",    10, trigger_callback, ros::TransportHints().tcpNoDelay());          
  ros::Subscriber sub4 = n.subscribe("waypoints",  10, waypoints_callback, ros::TransportHints().tcpNoDelay()); 
  ros::Subscriber sub5 = n.subscribe("correction", 10, correction_callback, ros::TransportHints().tcpNoDelay());               
  pubc                 = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);  
  pubp                 = n.advertise<nav_msgs::Path>(                 "path",    1, true);    
  pubt                 = n.advertise<nav_msgs::Path>(                 "traj",    1, true);    
  
  // Dynamic Reconfig
  dynamic_reconfigure::Server<trajectory_generator::TrajectoryGeneratorUIConfig> server;
  dynamic_reconfigure::Server<trajectory_generator::TrajectoryGeneratorUIConfig>::CallbackType ff;
  ff = boost::bind(&config_callback, _1, _2);
  server.setCallback(ff);   
  
  // Set trajectory parameters
  traj_gen.SetVelocity(1);
  traj_gen.SetAcceleration(1);
  traj_gen.SetYawVelocity(30*PI/180);
  traj_gen.SetDerivativeOrder(3);
  traj_gen.SetYawDerivativeOrder(2);  
  
  ros::spin();
  
  return 0;
}
