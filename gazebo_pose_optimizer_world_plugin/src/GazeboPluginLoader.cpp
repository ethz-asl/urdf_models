#include <gazebo_pose_optimizer_world_plugin/GazeboPluginLoader.h>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/MeshShape.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <algorithm>
#include <assert.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>


#define DEFAULT_PUBLISH_OBJECTS false
#define DEFAULT_WORLD_OBJECTS_TOPIC "world/objects"
#define DEFAULT_REQUEST_OBJECTS_TOPIC "world/request_object"
#define DEFAULT_ROOT_FRAME_ID "world"

//publishing rate
#define UPDATE_RATE 5

#define OBJECT_QUEUE_SIZE 100

using gazebo::GazeboPluginLoader;

GazeboPluginLoader::GazeboPluginLoader() :
    WorldPlugin(){
    //reGenerateObjects=true;
}

void GazeboPluginLoader::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{

  ros::NodeHandle node("/pose_generator");

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    node.param<bool>("publish_world_objects", PUBLISH_OBJECTS, DEFAULT_PUBLISH_OBJECTS);
    ROS_INFO("GazeboObjInfo: Got objects publish flag: <%i>", PUBLISH_OBJECTS);

    node.param<std::string>("world_objects_topic", WORLD_OBJECTS_TOPIC, DEFAULT_WORLD_OBJECTS_TOPIC);
    ROS_INFO("GazeboObjInfo: Got objects topic name: <%s>", WORLD_OBJECTS_TOPIC.c_str());

    node.param<std::string>("request_object_service", REQUEST_OBJECTS_TOPIC, DEFAULT_REQUEST_OBJECTS_TOPIC);
    ROS_INFO("GazeboObjInfo: Got objects topic name: <%s>", REQUEST_OBJECTS_TOPIC.c_str());

    node.param<std::string>("objects_frame_id", ROOT_FRAME_ID, DEFAULT_ROOT_FRAME_ID);
    ROS_INFO("GazeboObjInfo: Got objects frame id: <%s>", ROOT_FRAME_ID.c_str());

    world=_world;

    if (PUBLISH_OBJECTS){
        update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPluginLoader::onWorldUpdate, this));
        object_pub = node.advertise<GazeboPluginLoader::ObjectMsg>(WORLD_OBJECTS_TOPIC, OBJECT_QUEUE_SIZE);
    }

    ros::Rate rate(UPDATE_RATE);

    ROS_INFO("Hello World!");
    world=_world;

    request_object_srv = node.advertiseService(REQUEST_OBJECTS_TOPIC, &GazeboPluginLoader::requestObject,this);

	//ROS_INFO_STREAM(models_);
}

bool GazeboPluginLoader::requestObject(object_msgs::ObjectInfo::Request &req, object_msgs::ObjectInfo::Response &res) {

    std::string modelName=req.name;
    physics::ModelPtr model=world->GetModel(modelName);

    if (!model.get()) {
        // ROS_ERROR("Model %s not found",modelName.c_str());
        res.success=false;
        res.error_code=object_msgs::ObjectInfo::Response::OBJECT_NOT_FOUND;
        return true;
    }

    res.error_code=object_msgs::ObjectInfo::Response::NO_ERROR;
    res.success=true;
   // res.object=createBoundingBoxObject(model,req.get_geometry);
    //ROS_INFO("Received service request for object info!");
    return true;
}

void GazeboPluginLoader::onWorldUpdate() {
    if (object_pub.getNumSubscribers()==0) return;
//
//    if (!reGenerateObjects) return;
//
//    physics::Model_V models=world->GetModels();
//    physics::Model_V::iterator m_it;
//
//    bool send_shape=false;
//    for (m_it=models.begin(); m_it!=models.end(); ++m_it) {
//        //ROS_INFO_STREAM("Have model "<<(*m_it)->GetName());
//        GazeboObjectInfo::ObjectMsg obj=createBoundingBoxObject(*m_it,send_shape);
//        lastGeneratedObjects.push_back(obj);
//    }
//
//    reGenerateObjects=false;
}


//
//void GazeboPluginLoader::onWorldCreate(){
//    ROS_INFO("On-World-Create: Reading list of Gazebo world plugins to be loaded from parameters...");
//
//    ros::NodeHandle node("/gazebo_state_plugins");
//
//    ROS_INFO_STREAM("Reading parameters from namespace "<<node.getNamespace());
//
//    std::string world_name;
//    node.getParam("world_name", world_name);
//    ROS_INFO_STREAM("Loading plugins for world '"<<world_name<<"'");
//
//    physics::WorldPtr world = gazebo::physics::get_world(world_name);
//    if (!world)
//    {
//        ROS_ERROR("Could not obtain pointer to world from system plugin, so won't be able to load plugins");
//        return;
//    }
//
//    XmlRpc::XmlRpcValue world_plugins;
//    node.getParam("world_plugins", world_plugins);
//    if (world_plugins.getType() != XmlRpc::XmlRpcValue::TypeArray)
//    {
//      ROS_ERROR("Parameter world_plugins should be specified as an array. Gazebo world plugins won't be loaded.");
//      return;
//    }
//
//     get the list of names
//    for (int i = 0 ; i < world_plugins.size() ; ++i)
//    {
//      if (!world_plugins[i].hasMember("name") || !world_plugins[i].hasMember("file"))
//      {
//            ROS_ERROR("World plugin parameter specification should have 'name' and 'file'. Gazebo world plugins won't be loaded.");
//            continue;
//      }
//      std::string name=world_plugins[i]["name"];
//      std::string file=world_plugins[i]["file"];
//      sdf::ElementPtr _sdf;  // will be initialized to NULL
//      world->LoadPlugin(file, name, _sdf);
//    }
//
//    ROS_INFO("Eligible plugins loaded.");
//}

GZ_REGISTER_WORLD_PLUGIN(GazeboPluginLoader)
