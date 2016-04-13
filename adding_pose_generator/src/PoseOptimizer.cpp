/*
 * Copyright 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "adding_pose_generator/PoseOptimizer.h"

//#include "gazebo_msgs/

namespace adding_pose_generator {


void PoseOptimizer::GazeboLinkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
	//ROS_INFO_STREAM("Callback: " << *msg);
	gazebo_link_states_ = *msg;
}

////////////////////////////////////////////////////////////////////////////////
PoseOptimizer::PoseOptimizer(ros::NodeHandle& nh)
: nh_(nh)
{
	getGazeboLinkState_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &PoseOptimizer::GazeboLinkCallback, this);
	executeOptServer_ = nh_.advertiseService("test_advertise", &PoseOptimizer::simpleService, this);
	//pose_optimization_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/SetModelState");
}

bool PoseOptimizer::simpleService(gazebo_msgs::GetModelProperties::Request &request, gazebo_msgs::GetModelState::Response &response)
{
	if(request.model_name == "2_poly"){
		ROS_INFO_STREAM("position " << response.pose.position);
		//gazebo::physics::WorldPtr world =  gazebo::physics::Model
		//ROS_INFO_STREAM(gazebo_link_states_.pose[0]);
		//gazebo::physics::WorldPtr world = gazebo::physics::get_world("World");
		//model_ = world->GetByName(request.model_name);

		//ROS_INFO_STREAM(model_->GetWorldPose());
		//ROS_INFO_STREAM(response.pose);
	}
	//ROS_INFO()
	return true;
}



////////////////////////////////////////////////////////////////////////////////
PoseOptimizer::~PoseOptimizer()
{

}

}
