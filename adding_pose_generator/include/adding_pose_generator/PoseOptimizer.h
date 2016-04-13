# pragma once
#include <ros/ros.h>


#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelProperties.h"

//#include "gazebo_msgs/SetPhysicsProperties.h"
//#include "gazebo_msgs/ApplyBodyWrench.h"
//#include "gazebo_msgs/ODEPhysics.h"


#include "adding_pose_generator/string.h"
#include <std_srvs/Empty.h>


namespace adding_pose_generator {

class PoseOptimizer {
public:

    PoseOptimizer(ros::NodeHandle& nh);
    virtual ~PoseOptimizer();
    void GazeboLinkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void GazeboModelCallback(const gazebo_msgs::ModelStates::ConstPtr & msg);

    bool simpleService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

private:
    ros::NodeHandle nh_;

   // ros::ServiceClient spawn_object;
    ros::ServiceServer executeOptServer_;
    ros::ServiceClient pose_optimization_client_;
    ros::Subscriber getGazeboLinkState_;


    gazebo::physics::ModelPtr model_;
    gazebo_msgs::LinkStates  gazebo_link_states_;
    gazebo_msgs::ModelStates gazebo_model_states_;

   // gazebo_msgs::ApplyBodyWrench wrench_;
    //gazebo_msgs::GetWorldProperties world_;
   // gazebo_msgs::ODEPhysics physics_;

};
// Register this plugin with the simulator
//GZ_REGISTER_PHYSICS_ENGINE(PoseOptimizer, )
}
