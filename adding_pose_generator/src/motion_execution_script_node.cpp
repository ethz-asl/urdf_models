/*
 * motion_execution_script_node.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: yoshidah
 */


// motion execution script
#include "motion_execution_script/MotionExecutionScript.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_execution_script");
  ros::NodeHandle nodeHandle("~");

  motion_execution_script::MotionExecutionScript motionExecutionScript(nodeHandle);

  ros::AsyncSpinner spinner(2);
  spinner.start();
//  ros::Rate(50);
  ros::waitForShutdown();

  return 0;
}

