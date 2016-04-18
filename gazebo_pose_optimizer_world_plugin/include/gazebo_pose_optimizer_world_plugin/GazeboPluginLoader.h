#ifndef GAZEBO_GAZEBOPLUGINLOADER_H
#define GAZEBO_GAZEBOPLUGINLOADER_H

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>

#include <motion_execution_msgs/SetObjectStatic.h>

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{

class GazeboPluginLoader : public WorldPlugin
{
public: 
  typedef object_msgs::Object ObjectMsg;
  typedef object_msgs::ObjectInfo ObjectInfoMsg;

	//GazeboPluginLoader();
  GazeboPluginLoader();
	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
	//bool service_callback(gazebo_msgs::LinkStates::std_msgs::Empty::ConstPtr  &req);
	void Update();
private:
  bool requestObject(object_msgs::ObjectInfo::Request  &req, object_msgs::ObjectInfo::Response &res);
  bool requestStatic(motion_execution_msgs::SetObjectStatic::Request  &req, object_msgs::ObjectInfo::Response &res);

  void advertEvent(const ros::TimerEvent& e);
  void onWorldUpdate();
  ObjectMsg getObject(physics::ModelPtr& model, bool include_shape);



private:
  bool PUBLISH_OBJECTS;
  std::string WORLD_OBJECTS_TOPIC;
  std::string REQUEST_OBJECTS_TOPIC;
  std::string ROOT_FRAME_ID;

  physics::WorldPtr world;

  event::ConnectionPtr update_connection;
  ros::Publisher object_pub;
  ros::ServiceServer request_object_srv;
  ros::ServiceServer request_object_static;


  ros::Timer publishTimer;

  std::vector<ObjectMsg> lastGeneratedObjects;
  bool reGenerateObjects;

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
