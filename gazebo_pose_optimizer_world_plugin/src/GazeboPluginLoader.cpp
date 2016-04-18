#include <gazebo_pose_optimizer_world_plugin/GazeboPluginLoader.h>

#include <object_msgs_tools/ObjectFunctions.h>
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

  ros::NodeHandle node("~");

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

    world=_world;

    request_object_srv = node.advertiseService(REQUEST_OBJECTS_TOPIC, &GazeboPluginLoader::requestObject,this);
    request_object_static = node.advertiseService("world/object_static", &GazeboPluginLoader::requestStatic,this);

	//ROS_INFO_STREAM(models_);
}

bool GazeboPluginLoader::requestStatic(motion_execution_msgs::SetObjectStatic::Request  &req, object_msgs::ObjectInfo::Response &res)
{
  std::string modelName=req.name;
  physics::ModelPtr model=world->GetModel(modelName);
  model-> SetStatic(req.set_static);

  if(req.set_static) ROS_INFO("model %s was set with static", modelName.c_str());
  else ROS_INFO("model %s was set with active", modelName.c_str());
  return true;
}

bool GazeboPluginLoader::requestObject(object_msgs::ObjectInfo::Request &req, object_msgs::ObjectInfo::Response &res) {

    std::string modelName=req.name;
    physics::ModelPtr model=world->GetModel(modelName);

    if (!model.get()) {
         ROS_ERROR("Model %s not found",modelName.c_str());
        res.success=false;
        res.error_code=object_msgs::ObjectInfo::Response::OBJECT_NOT_FOUND;
        return true;
    }

    res.error_code=object_msgs::ObjectInfo::Response::NO_ERROR;
    res.success=true;
    res.object=getObject(model,false);
    ROS_INFO("Received service request for object info!");
    return true;
}

void GazeboPluginLoader::advertEvent(const ros::TimerEvent& e) {
    if (object_pub.getNumSubscribers()==0) return;
    std::vector<GazeboPluginLoader::ObjectMsg>::iterator it;
    for (it=lastGeneratedObjects.begin(); it!=lastGeneratedObjects.end(); ++it) {
        object_pub.publish(*it);
    }
    lastGeneratedObjects.clear();
    reGenerateObjects=true;
}

void GazeboPluginLoader::onWorldUpdate() {
    if (object_pub.getNumSubscribers()==0) return;

    if (!reGenerateObjects) return;

    physics::Model_V models=world->GetModels();
    physics::Model_V::iterator m_it;

    bool send_shape=false;
    for (m_it=models.begin(); m_it!=models.end(); ++m_it) {
        ROS_INFO_STREAM("Have model "<<(*m_it)->GetName());
        GazeboPluginLoader::ObjectMsg obj=getObject(*m_it,send_shape);
        lastGeneratedObjects.push_back(obj);
    }

    reGenerateObjects=false;
}


GazeboPluginLoader::ObjectMsg GazeboPluginLoader::getObject(physics::ModelPtr& model, bool include_shape)
{
  GazeboPluginLoader::ObjectMsg obj;


    physics::Link_V links=model->GetLinks();
    physics::Link_V::iterator l_it;

    obj.name=model->GetName();
    obj.header.stamp=ros::Time::now();
    obj.header.frame_id=ROOT_FRAME_ID;
    // obj.type =  no object type given
    // the custum origin (Object::origin) is going to be set to the first link encountered
    bool origin_init = false;
    for (l_it=links.begin(); l_it!=links.end(); ++l_it)
    {
        physics::LinkPtr link=*l_it;

        std::string linkName=link->GetName();

        math::Pose link_pose=link->GetWorldPose();
        //ROS_INFO("Link for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),link_pose.pos.x,link_pose.pos.y,link_pose.pos.z);
        //ROS_INFO("Link found for model %s: %s",model->GetName().c_str(),link->GetName().c_str());

        physics::Collision_V colls=link->GetCollisions();
        physics::Collision_V::iterator cit;
        for (cit=colls.begin(); cit!=colls.end(); ++cit)
        {
            physics::CollisionPtr c=*cit;

            math::Pose rel_pose=c->GetRelativePose();

            //ROS_INFO("Collision for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),rel_pose.pos.x,rel_pose.pos.y,rel_pose.pos.z);
            //math::Pose w_pose=c->GetWorldPose();
            //ROS_INFO("World pos for model %s: %s, pos %f %f %f",model->GetName().c_str(),link->GetName().c_str(),w_pose.pos.x,w_pose.pos.y,w_pose.pos.z);

            math::Pose coll_pose=rel_pose+link_pose; //XXX c->GetWorldPose() does not work for non-static objects, so it has to be relative pose and link world pose
            geometry_msgs::Pose pose;
            pose.position.x=coll_pose.pos.x;
            pose.position.y=coll_pose.pos.y;
            pose.position.z=coll_pose.pos.z;
            pose.orientation.x=coll_pose.rot.x;
            pose.orientation.y=coll_pose.rot.y;
            pose.orientation.z=coll_pose.rot.z;
            pose.orientation.w=coll_pose.rot.w;

            if(c->GetShape()->HasType(gazebo::physics::Base::MESH_SHAPE)) obj.mesh_poses.push_back(pose);
            if(c->GetShape()->HasType(gazebo::physics::Base::BOX_SHAPE)) obj.primitive_poses.push_back(pose);

            if (!origin_init)
            {
                obj.origin = pose;
            }
//
//            if (include_shape) {
//              if(c->GetShape()->HasType(gazebo::physics::Base::MESH_SHAPE)){
//                ROS_INFO_ONCE("Mesh was recognized");
//                //shape_msgs::Mesh * mesh = getMesh(c);
//                obj.meshes.push_back(*mesh);
//                delete mesh;
//              }
//
//              if(c->GetShape()->HasType(gazebo::physics::Base::BOX_SHAPE)){
//                //shape_msgs::SolidPrimitive * solid = getSolidPrimitive(c);
//                obj.primitives.push_back(*solid);
//                    if (!solid) {
//                        ROS_WARN("Skipping coll %s of link %s of model %s, could not get SolidPrimitive. ",c->GetName().c_str(),linkName.c_str(),obj.name.c_str());
//                        continue;
//                    }
//                    delete solid;
//              }
//                obj.content=GazeboObjectInfo::ObjectMsg::SHAPE;
//            }else {
//                obj.content=GazeboObjectInfo::ObjectMsg::POSE;
//                //ROS_INFO("Pub pose for %s (%s %s)",c->GetName().c_str(),link->GetName().c_str(),model->GetName().c_str());
//            }
        }
    }

    obj.primitive_origin = GazeboPluginLoader::ObjectMsg::ORIGIN_AVERAGE;
    obj.mesh_origin = GazeboPluginLoader::ObjectMsg::ORIGIN_UNDEFINED;

    return obj;
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
