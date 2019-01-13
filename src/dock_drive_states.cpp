/*
This is an example of the dock IR message. A 3 byte array arranged thus:

[Right IR Sensor, Middle IR Sensor, Left IR Sensor]

Message Example

---
header:
  seq: 62605
  stamp:
    secs: 1547332290
    nsecs: 880964496
  frame_id: "dock_ir_link"
data: [48, 0, 0]
---

*/

/*****************************************************************************
** includes
*****************************************************************************/

#include "../include/mxnet_dock_drive.hpp"
#include <ros/ros.h>

using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {
  /*********************************************************
   * Shared variables among states
   * @ dock_detector : records + or - when middle IR sensor detects docking signal
   * @ rotated : records how much the robot has rotated in scan state
   * @ bump_remainder : from processBumpChargerEvent.
   *********************************************************/


  /*******************************************************
   * Idle
   *  @breif Entry of auto docking state machine
   *
   *  Shared variable
   *  @dock_detecotr - indicates where the dock is located. Positive means dock is on left side of robot
   *  @rotated       - indicates how much the robot has rotated while scan
   *******************************************************/
  void DockDrive::idle(RobotDockingState::State& nstate,double& nvx, double& nwz) {
    dock_detector = 0;
    rotated = 0.0;
    nstate = RobotDockingState::SCAN;
    nvx = 0;
    nwz = 0.66;
  }

  /********************************************************
   * Scan
   *  @breif While it rotates ccw, determines the dock location with only middle sensor.
   *         If its middle sensor detects center ir, the robot is aligned with docking station
   *
   *  Shared variable
   *  @dock_detecotr - indicates where the dock is located. Positive means dock is on left side of robot
   *  @rotated       - indicates how much the robot has rotated while scan
   ********************************************************/
  void DockDrive::scan( RobotDockingState::State& nstate,
                        double& nvx, 
                        double& nwz, 
                        const std::vector<unsigned char>& signal_filt, 
                        const ecl::LegacyPose2D<double>& pose_update, 
                        std::string& debug_str) 
  {
    unsigned char right = signal_filt[0];
    unsigned char mid   = signal_filt[1];
    unsigned char left  = signal_filt[2];

    RobotDockingState::State next_state;
    double next_vx;
    double next_wz;

    rotated += pose_update.heading() / (2.0 * M_PI);
    std::ostringstream oss;
    oss << "rotated: " << std::fixed << std::setprecision(2) << std::setw(4) << rotated;
    debug_str = oss.str();

 //cout << "Scan State" << endl;

    if((mid & DockStationIRState::FAR_CENTER) || (mid & DockStationIRState::NEAR_CENTER))
    {
      next_state = RobotDockingState::ALIGNED;
      next_vx = 0.05;
      next_wz = 0.0;
    }
    // robot is located left side of dock
    else if(mid & (DockStationIRState::FAR_LEFT + DockStationIRState::NEAR_LEFT))
    {
      dock_detector--;
      next_state = RobotDockingState::SCAN;
      next_vx = 0.0;
      next_wz = 0.66;
    }
    // robot is located right side of dock
    else if(mid & (DockStationIRState::FAR_RIGHT + DockStationIRState::NEAR_RIGHT))
    {
      dock_detector++;
      next_state = RobotDockingState::SCAN;
      next_vx = 0.0;
      next_wz = 0.66;
    }
    // robot is located in front of robot
    else if(mid) { // if mid sensor sees something, rotate slowly
      next_state = RobotDockingState::SCAN;
      next_vx = 0.0;
      next_wz = 0.10;
    }
    else if(std::abs(rotated) > 1.0)
    {
      next_state = RobotDockingState::FIND_STREAM;
      next_vx = 0;
      next_wz = 0;
    }
    else { // if mid sensor does not see anything, rotate fast
      next_state = RobotDockingState::SCAN;
      next_vx = 0.0;
      next_wz = 0.66;
    }

    nstate = next_state;
    nvx = next_vx;
    nwz = next_wz;
  }

  /********************************************************
   * Find stream
   *  @breif based on dock_detector variable, it determines the dock's location and rotates toward the center line of dock
   *
   *  Shared variable
   *  @dock_detector - to determine dock's location
   *
   ********************************************************/
  void DockDrive::find_stream(RobotDockingState::State& nstate,double& nvx, double& nwz, const std::vector<unsigned char>& signal_filt) {
    unsigned char right = signal_filt[0];
    unsigned char mid   = signal_filt[1];
    unsigned char left  = signal_filt[2];
    RobotDockingState::State next_state;
    double next_vx;
    double next_wz;


 //cout << "Find Stream" << endl;

    if(dock_detector > 0) // robot is located in right side of dock
    {
      // turn right, CW until get right signal from left sensor
      if(left & (DockStationIRState::FAR_RIGHT + DockStationIRState::NEAR_RIGHT)) 
      //if(right & (DockStationIRState::FAR_RIGHT + DockStationIRState::NEAR_RIGHT)) 
      {
        next_state = RobotDockingState::GET_STREAM;
        next_vx = 0.5;
        next_wz = 0.0;
      }
      else 
      {
        next_state = RobotDockingState::FIND_STREAM;
        next_vx = 0.0;
        next_wz = -0.33;
      }
    }
    else if(dock_detector < 0 ) // robot is located in left side of dock
    {
      // turn left, CCW until get left signal from right sensor
      if(right & (DockStationIRState::FAR_LEFT + DockStationIRState::NEAR_LEFT))
 //     if(left & (DockStationIRState::FAR_LEFT + DockStationIRState::NEAR_LEFT))
      {
        next_state = RobotDockingState::GET_STREAM;
        next_vx = 0.5;
        next_wz = 0.0;
      }
      else 
      {
        next_state = RobotDockingState::FIND_STREAM;
        next_vx = 0.0;
        next_wz = 0.33;
      }
    }

    nstate = next_state;
    nvx = next_vx;
    nwz = next_wz;
  }

 /********************************************************
  * Get stream
  *   @brief In this state, robot is heading the center line of dock. 
  *   When it passes the center, it rotates toward the dock
  *
  *   Shared Variable
  *   @ dock_detector - reset
  *   @ rotated       - reset
  ********************************************************/
  void DockDrive::get_stream( RobotDockingState::State& nstate,
                              double& nvx, 
                              double& nwz, 
                              const std::vector<unsigned char>& signal_filt)
  {
    unsigned char right = signal_filt[0];
    unsigned char mid   = signal_filt[1];
    unsigned char left  = signal_filt[2];
    RobotDockingState::State next_state;
    double next_vx;
    double next_wz;

 //cout << "Get Stream" << endl;

    if(dock_detector > 0) 
		{ 
			// robot is located in right side of dock
      if (left & (DockStationIRState::FAR_LEFT + DockStationIRState::NEAR_LEFT)) {
        ROS_INFO_STREAM("Robot is located on right side of dock - Next State SCAN");
        dock_detector = 0;
        rotated = 0;
        next_state = RobotDockingState::SCAN;
        next_vx = 0;
        next_wz = 0.1;
      }
      else 
      {
        ROS_INFO_STREAM("Robot Dock Location Unknown [RIGHT] - Next State GET_STREAM");
        next_state = RobotDockingState::GET_STREAM;
        next_vx = 0.05;
        next_wz = 0.0;
      }
    }
    else if(dock_detector < 0) 
		{
			 // robot is located left side of dock
      if(right & (DockStationIRState::FAR_RIGHT + DockStationIRState::NEAR_RIGHT)) 
      {
        ROS_INFO_STREAM("Robot is located on left side of dock - Next State SCAN");
        dock_detector = 0;
        rotated = 0;
        next_state = RobotDockingState::SCAN;
        next_vx = 0;
        next_wz = 0.1;
      }
      else 
			{
				ROS_INFO_STREAM("Robot Dock Location Unknown [LEFT] - Next State GET_STREAM");
        next_state = RobotDockingState::GET_STREAM;
        next_vx = 0.05;
        next_wz = 0.0;
      }
    }

    nstate = next_state;
    nvx = next_vx;
    nwz = next_wz;
  }


 /********************************************************
  * Aligned
  *   @breif Robot sees center IR with middle sensor. It is heading dock. It approaches to the dock only using mid sensor
  *
  *   Shared Variable
  *   @ dock_detector - reset
  *   @ rotated       - reset
  ********************************************************/
  void DockDrive::aligned(RobotDockingState::State& nstate,double& nvx, double& nwz, const std::vector<unsigned char>& signal_filt, std::string& debug_str)
  {
    unsigned char right = signal_filt[0];
    unsigned char mid   = signal_filt[1];
    unsigned char left  = signal_filt[2];
    RobotDockingState::State next_state = nstate;
    double next_vx = nvx;
    double next_wz = nwz;

 //cout << "Aligned Stream" << endl;

    if(mid)
    {
      if(((mid & DockStationIRState::NEAR) == DockStationIRState::NEAR_CENTER) || 
         ((mid & DockStationIRState::NEAR) == DockStationIRState::NEAR))
      {
        debug_str = "AlignedNearCenter";
        ROS_INFO_STREAM("AlignedNearCenter");
        next_state = RobotDockingState::ALIGNED_NEAR;
        next_vx = 0.05;
        next_wz = 0.0;
      }
      else if(mid & DockStationIRState::NEAR_LEFT) 
      {
        debug_str = "AlignedNearLeft";
        ROS_INFO_STREAM("AlignedNearLeft");
        next_state = RobotDockingState::ALIGNED_NEAR;
        next_vx = 0.05;
        next_wz = 0.5;
      }
      else if(mid & DockStationIRState::NEAR_RIGHT) 
      {
        debug_str = "AlignedNearRight";
        ROS_INFO_STREAM("AlignedNearRight");
        next_state = RobotDockingState::ALIGNED_NEAR;
        next_vx = 0.05;
        next_wz = -0.5;
      }
      else if(((mid & DockStationIRState::FAR) == DockStationIRState::FAR_CENTER) || 
              ((mid & DockStationIRState::FAR) == DockStationIRState::FAR)) 
      {
        debug_str = "AlignedFarCenter";
        ROS_INFO_STREAM("AlignedFarCenter");
        next_state = RobotDockingState::ALIGNED_FAR;
        next_vx = 0.1;
        next_wz = 0.0;
      }
      else if(mid & DockStationIRState::FAR_LEFT) {
        debug_str = "AlignedFarLeft";
        ROS_INFO_STREAM("AlignedFarLeft");
        next_state = RobotDockingState::ALIGNED_FAR;
        next_vx = 0.1;
        next_wz = 0.3;
      }
      else if(mid & DockStationIRState::FAR_RIGHT) 
      {
        debug_str = "AlignedFarRight";
        ROS_INFO_STREAM("AlignedFarRight");
        next_state = RobotDockingState::ALIGNED_FAR;
        next_vx = 0.1;
        next_wz = -0.3;
      }
      else 
      {
        ROS_INFO_STREAM("Aligned Else 1");
        dock_detector = 0;
        rotated = 0.0;
        next_state = RobotDockingState::SCAN;
        next_vx = 0.0;
        next_wz = 0.66;
      }
    }
    else 
    {
        ROS_INFO_STREAM("Aligned Else 2");
        next_state = RobotDockingState::SCAN;
        next_vx = 0.0;
        next_wz = 0.66;
    }

    nstate = next_state;
    nvx = next_vx;
    nwz = next_wz;
  }

 /********************************************************
  * Bumped
  *  @breif Robot has bumped somewhere. Go backward for 10 iteration
  *
  ********************************************************/
  void DockDrive::bumped(RobotDockingState::State& nstate,double& nvx, double& nwz, int& bump_count)
  {
//cout << "Bumped" << endl;

    if(bump_count < 10)
    {
      nvx = -0.05;
      nwz = 0.0;
      bump_count++;
    }
    else {
      nstate = RobotDockingState::SCAN;
      nvx = 0.0;
      nwz = 0.0;
      bump_count = 0;
    }

  }
}
