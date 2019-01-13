#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <../../../../devel_isolated/mxnet_actionlib/include/mxnet_actionlib/AutoDockingAction.h>


using namespace mxnet_actionlib;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const AutoDockingResultConstPtr& result)
{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Answer: %s", result->text.c_str());
//  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const AutoDockingFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback state %s", feedback->state.c_str());
	ROS_INFO("Got Feedback text %s", feedback->text.c_str());

}

 void spinThread()
{
 	ros::spin();
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<mxnet_actionlib::AutoDockingAction> ac("fibonacci", true);
	boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  mxnet_actionlib::AutoDockingGoal goal;
//  goal.order = 20;
//  ac.sendGoal(goal);

	// Need to register for feedback
	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult();
	actionlib::SimpleClientGoalState state = ac.getState();

	//ros::spin();

	if (state == actionlib::SimpleClientGoalState::LOST)
  {
		ROS_INFO("Docking Lost - Attempting Redocking");
	}
	else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		 ROS_INFO("Docked OK!");
	}
	else
	{
    ROS_INFO("Docking Failed!");
	}

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
