/*
 * MotionExecutionScript.hpp
 *
 *  Created on: Mar 9, 2016
 *      Author: yoshidah
 */

# pragma once

// c++
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <string>
#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>
//#include <geometry_msgs/Pose.h>

//moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

//moveit_msgs
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

// robotiq
#include <robotiq_force_torque_sensor/ft_sensor.h>
#include <robotiq_msgs/SModelRobotOutput.h>
#include <robotiq_msgs/SModelRobotInput.h>

// gazebo
#include "gazebo_msgs/LinkStates.h"

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>


namespace motion_execution_script {


class MotionExecutionScript
{

public:
	MotionExecutionScript(ros::NodeHandle& nh);
	virtual ~MotionExecutionScript();

	void CloseGripper();
	void OpenGripper();
	void ActivateGripper();
	void ShutdownGripper();

private:

	// callback functions
	void FTSensorCallback(const robotiq_force_torque_sensor::ft_sensor::ConstPtr& msg);

	void GripperCallback(const robotiq_msgs::SModelRobotInput::ConstPtr& msg);

	void GazeboLinkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

	void RequestObjectCallback(const moveit_msgs::PlanningScene::ConstPtr  &msg);

    void subscribeAndAdvertise(const std::string& get_planning_scene_service = "/get_planning_scene",
                               const std::string& set_planning_scene_topic = "/planning_scene");


	bool getObjectPose(const std::string& object_name, const float timeout_wait_object, geometry_msgs::PoseStamped& pose);
	// end callback functions



	bool ExecuteScript(std_srvs::Empty::Request& request,
            std_srvs::Empty::Response& response);


	void IncrementalMove();

	void Move(std::vector<double> joint_values)	;


	ros::NodeHandle nh_;

	ros::Publisher gripperCommandPublisher_;

	ros::Subscriber FTSubscriber_;

	ros::Subscriber GripperSubscriber_;

	ros::ServiceServer executeMotionServer_;

	ros::Subscriber getGazeboLinkState_;

	ros::Subscriber getCollisionObject_;

	robotiq_force_torque_sensor::ft_sensor force_torque_feedback_;

	robotiq_msgs::SModelRobotOutput gripper_command_;

	robotiq_msgs::SModelRobotInput gripper_msg_;

	gazebo_msgs::LinkStates gazebo_msgs_;

	moveit_msgs::PlanningScene planning_scene_msgs_;

	moveit::planning_interface::MoveGroup group_;

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

	// Speed scaling [0, 1].
	double scaling_;


//
//	moveit::planning_interface::MoveGroup::Plan my_plan;
//
//	bool success;

};

} // motion_execution_script
