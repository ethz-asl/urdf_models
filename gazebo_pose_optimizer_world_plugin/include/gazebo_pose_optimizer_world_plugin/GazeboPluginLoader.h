#ifndef GAZEBO_GAZEBOPLUGINLOADER_H
#define GAZEBO_GAZEBOPLUGINLOADER_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class GazeboPluginLoader : public WorldPlugin
{
public: 
	//GazeboPluginLoader();
	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:

};

}
#endif  // GAZEBO_GAZEBOPLUGINLOADER_H
