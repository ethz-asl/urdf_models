
//#include <ros/ros.h>
#include "adding_pose_generator/PoseOptimizer.h"



int main(int argc, char **argv)
{
  ros::init (argc, argv, "adding_pose_generator");
  ros::NodeHandle node_handle("~");

  adding_pose_generator::PoseOptimizer poseOpt(node_handle);

  ros::AsyncSpinner spinner(1);

   spinner.start();
   ros::waitForShutdown();

  return 0;
}
