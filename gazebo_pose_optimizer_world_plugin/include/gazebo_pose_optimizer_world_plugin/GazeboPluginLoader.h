#ifndef GAZEBO_GAZEBOPLUGINLOADER_H
#define GAZEBO_GAZEBOPLUGINLOADER_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/rendering.hh>


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

private:
  bool requestObject(object_msgs::ObjectInfo::Request  &req, object_msgs::ObjectInfo::Response &res);
  bool requestStatic(motion_execution_msgs::SetObjectStatic::Request  &req, motion_execution_msgs::SetObjectStatic::Response &res);

  void advertEvent(const ros::TimerEvent& e);
  void onWorldUpdate();
  ObjectMsg getObject(physics::ModelPtr& model, bool include_shape);



private:
  bool PUBLISH_OBJECTS;
  std::string WORLD_OBJECTS_TOPIC;
  std::string REQUEST_OBJECTS_TOPIC;
  std::string ROOT_FRAME_ID;

  physics::WorldPtr world;
  physics::ModelPtr model_;

  rendering::UserCameraPtr camera_;
  rendering::ScenePtr scene_;


  event::ConnectionPtr update_connection;
  ros::Publisher object_pub;
  ros::ServiceServer request_object_srv;
  ros::ServiceServer request_object_static;


  ros::Timer publishTimer;

  std::vector<ObjectMsg> lastGeneratedObjects;
  bool reGenerateObjects;
};

}
#endif  // GAZEBO_GAZEBOPLUGINLOADER_H
