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
#include "ur_gazebo_plugins/InitialPosePlugin.h"

////////////////////////////////////////////////////////////////////////////////
InitialPosePlugin::InitialPosePlugin()
{

}

////////////////////////////////////////////////////////////////////////////////
InitialPosePlugin::~InitialPosePlugin()
{

}

////////////////////////////////////////////////////////////////////////////////
void InitialPosePlugin::Load(gazebo::physics::ModelPtr _parent,
                             sdf::ElementPtr _sdf)
{
  this->model = _parent;
//  this->world = this->model->GetWorld();
//  this->world->SetPaused(true);
//  this->sdf = _sdf;
  this->model->GetJoint("shoulder_pan_joint")->SetPosition(0, -2.89);
  this->model->GetJoint("shoulder_lift_joint")->SetPosition(0, -1.35);
  this->model->GetJoint("elbow_joint")->SetPosition(0, -1.86);
  this->model->GetJoint("wrist_1_joint")->SetPosition(0, -1.44);
  this->model->GetJoint("wrist_2_joint")->SetPosition(0, 1.59);
  this->model->GetJoint("wrist_3_joint")->SetPosition(0, -0.168);

//  this->world->SetPaused(false);

}


GZ_REGISTER_MODEL_PLUGIN(InitialPosePlugin)
