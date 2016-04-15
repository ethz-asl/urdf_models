#ifndef GAZEBO_GAZEBOPLUGINLOADER_H
#define GAZEBO_GAZEBOPLUGINLOADER_H

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class GazeboPluginLoader : public WorldPlugin
{
public: 
	//GazeboPluginLoader();
	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
	//bool service_callback(gazebo_msgs::LinkStates::std_msgs::Empty::ConstPtr  &req);
	void Update();
private:

    ros::NodeHandle rosnode_;
    ros::CallbackQueue callback_queue_;
    physics::WorldPtr world_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_event_;
    ros::ServiceServer switch_service_;

    physics::LinkPtr body_;
    ros::Subscriber joint_command_subscriber_;

};

}
#endif  // GAZEBO_GAZEBOPLUGINLOADER_H
