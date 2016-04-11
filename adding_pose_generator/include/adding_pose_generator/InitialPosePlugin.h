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
#ifndef GAZEBO_INITIAL_POSE_PLUGIN_HH
#define GAZEBO_INITIAL_POSE_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>



class InitialPosePlugin : public gazebo::ModelPlugin
{

  /// \brief Constructor.
  public: InitialPosePlugin();

  /// \brief Destructor.
  public: virtual ~InitialPosePlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);


  // Pointer to the model
   private: gazebo::physics::ModelPtr model;

   private: gazebo::physics::WorldPtr world;


   // Pointer to the update event connection
   private: gazebo::event::ConnectionPtr updateConnection;

};

#endif  // GAZEBO_INITIAL_POSE_PLUGIN_HH
