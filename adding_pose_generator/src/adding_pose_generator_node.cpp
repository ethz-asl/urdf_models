
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include "gazebo_msgs/LinkStates.h"


typedef  gazebo_msgs::LinkStates gazebo_msgs_;

void GazeboLinkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	//ROS_INFO_STREAM("Callback: " << *msg);
	gazebo_msgs_ = *msg;
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(10.0);
  sleep_time.sleep();


 //ros::Subscriber getGazeboLinkState_
  //ros::Subscriber getGazeboLinkState_ = node_handle.subscribe<gazebo_msgs::LinkStates>("/link_states", 1000, &GazeboLinkCallback, this);

  // Advertise the required topic
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Note that this topic may need to be remapped in the launch file
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

// Define the attached object message
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// We will use this message to add or
// subtract the object from the world
// and to attach the object to the robot
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "table_leg";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "table_leg";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

// Note that attaching an object to the robot requires
// the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;



  ros::shutdown();
  return 0;
}
