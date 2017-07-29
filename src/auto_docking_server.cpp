#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mxnet_actionlib/AutoDockingAction.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DockInfraRed.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

#include  "../include/mxnet_dock_drive.hpp"
using namespace kobuki;

typedef message_filters::sync_policies::ApproximateTime<
  nav_msgs::Odometry,
  kobuki_msgs::SensorState,
  kobuki_msgs::DockInfraRed
> SyncPolicy;

class FibonacciAction
{
  DockDrive dock_;
  ros::Subscriber debug_;
  ros::Publisher velocity_commander_, motor_power_enabler_, debug_jabber_;

  boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > odom_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::DockInfraRed> > ir_sub_;
  boost::shared_ptr<message_filters::Subscriber<kobuki_msgs::SensorState> > core_sub_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mxnet_actionlib::AutoDockingAction> as_;
  std::string action_name_;
  std::string name_;

  // create messages that are used to published feedback/result
  mxnet_actionlib::AutoDockingFeedback feedback_;
  mxnet_actionlib::AutoDockingResult result_;
  mxnet_actionlib::AutoDockingGoal goal_;
/*  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }*/

public:

  //as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
  FibonacciAction(std::string name)

  : action_name_(name)
  , as_(name, false)
    
  {
    action_name_ = name;
    name_ = name;
    as_.registerGoalCallback(boost::bind(&FibonacciAction::goalCb, this));
    as_.registerPreemptCallback(boost::bind(&FibonacciAction::preemptCb, this));
    as_.start();
    ROS_INFO_STREAM( "Action Server Name: "+ name);
  }

  ~FibonacciAction(void)
  {
  }

bool init(ros::NodeHandle& nh)
{
  ROS_INFO_STREAM( "Initialising Docking Action Server...");
  // Configure docking drive
  double min_abs_v, min_abs_w;
  if (nh_.getParam("min_abs_v", min_abs_v) == true)
    dock_.setMinAbsV(min_abs_v);

  if (nh_.getParam("min_abs_w", min_abs_w) == true)
    dock_.setMinAbsW(min_abs_w);

  // Publishers and subscribers
  velocity_commander_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  debug_jabber_ = nh_.advertise<std_msgs::String>("debug/feedback", 10);

  debug_ = nh_.subscribe("debug/mode_shift", 10, &FibonacciAction::debugCb, this);

  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom", 10));
  core_sub_.reset(new message_filters::Subscriber<kobuki_msgs::SensorState>(nh_, "/mobile_base/sensors/core", 10));
  ir_sub_.reset(new message_filters::Subscriber<kobuki_msgs::DockInfraRed>(nh_, "/mobile_base/sensors/dock_ir", 10));
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *odom_sub_, *core_sub_, *ir_sub_));
  sync_->registerCallback(boost::bind(&FibonacciAction::syncCb, this, _1, _2, _3));

 ROS_INFO_STREAM( "Initialised Docking Action Server...");
  return dock_.init();
  }

  void syncCb(const nav_msgs::OdometryConstPtr& odom,
              const kobuki_msgs::SensorStateConstPtr& core,
              const kobuki_msgs::DockInfraRedConstPtr& ir);
  void debugCb(const std_msgs::StringConstPtr& msg);


void goalCb()
{
  ROS_INFO_STREAM("goalCb");
  if (dock_.isEnabled()) {
    goal_ = *(as_.acceptNewGoal());
    result_.text = "Rejected: dock_drive is already enabled.";
    as_.setAborted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] New goal received but rejected.");
  } else {
    dock_.enable();
    goal_ = *(as_.acceptNewGoal());
    ROS_INFO_STREAM("[" << name_ << "] New goal received and accepted.");
  }
}

void preemptCb()
{
  ROS_INFO_STREAM("preemptCb");
  //ROS_DEBUG_STREAM("[" << name_ << "] Preempt requested.");
  dock_.disable();
  if (as_.isNewGoalAvailable()) {
    result_.text = "Preempted: New goal received.";
    as_.setPreempted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] " << result_.text );
  } else {
    result_.text = "Cancelled: Cancel requested.";
    as_.setPreempted( result_, result_.text );
    ROS_INFO_STREAM("[" << name_ << "] " << result_.text );
    dock_.disable();
  }
}
};

void FibonacciAction::syncCb(const nav_msgs::OdometryConstPtr& odom,
                            const kobuki_msgs::SensorStateConstPtr& core,
                            const kobuki_msgs::DockInfraRedConstPtr& ir)
{
  //ROS_INFO_STREAM("syncCb");
  //process and run
  if(this->dock_.isEnabled()) 
  {
    //ROS_INFO_STREAM("syncCb1");
    //conversions
    KDL::Rotation rot;
    tf::quaternionMsgToKDL( odom->pose.pose.orientation, rot );

    double r, p, y;
    rot.GetRPY(r, p, y);
//ROS_INFO_STREAM("syncCb2");
    ecl::LegacyPose2D<double> pose;
    pose.x(odom->pose.pose.position.x);
    pose.y(odom->pose.pose.position.y);
    pose.heading(y);

    //update
    this->dock_.update(ir->data, core->bumper, core->charger, pose);

    //publish debug stream
    std_msgs::StringPtr debug_log(new std_msgs::String);
    debug_log->data = this->dock_.getDebugStream();
    debug_jabber_.publish(debug_log);
//ROS_INFO_STREAM("syncCb");
    //publish command velocity
    if (this->dock_.canRun()) {
      geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist);
      cmd_vel->linear.x = this->dock_.getVX();
      cmd_vel->angular.z = this->dock_.getWZ();
      velocity_commander_.publish(cmd_vel);
    }
  }

  //action server execution
  if( as_.isActive() ) {
    if ( dock_.getState() == RobotDockingState::DONE ) {
      result_.text = "Arrived on docking station successfully.";
      as_.setSucceeded(result_);
      ROS_INFO_STREAM( "[" << name_ << "]: Arrived on docking station successfully.");
      ROS_DEBUG_STREAM( "[" << name_ << "]: Result sent.");
      dock_.disable();
    } else if ( !dock_.isEnabled() ) { //Action Server is activated, but DockDrive is not enabled, or disabled unexpectedly
      ROS_ERROR_STREAM("[" << name_ << "] Unintended Case: ActionService is active, but DockDrive is not enabled..");
      result_.text = "Aborted: dock_drive is disabled unexpectedly.";
      as_.setAborted( result_, "Aborted: dock_drive is disabled unexpectedly." );
      ROS_INFO_STREAM("[" << name_ << "] Goal aborted.");
      dock_.disable();
    } else {
      feedback_.state = dock_.getStateStr();
      feedback_.text = dock_.getDebugStr();
      as_.publishFeedback(feedback_);
      ROS_DEBUG_STREAM( "[" << name_ << "]: Feedback sent.");
    }
  }
  return;
}

void FibonacciAction::debugCb(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO_STREAM("debugCb");
  dock_.modeShift(msg->data);
}


/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci(ros::this_node::getName());
  ros::spin();

  return 0;
}
*/



int main(int argc, char** argv){
  ros::init(argc, argv, "fibonacci");

std::string nodelet_name("Auto_Docking_Action_Server");
ros::NodeHandle nh;

 FibonacciAction fibonacci(ros::this_node::getName());
 fibonacci.init(nh);
  ros::spin();

  return 0;
}