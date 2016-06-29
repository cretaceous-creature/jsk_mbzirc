/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <jsk_mbzirc_common/mbzirc_gazebo_world_plugin.h>
#include <string>

namespace gazebo
{

GazeboWORLD::GazeboWORLD()
{
}

GazeboWORLD::~GazeboWORLD()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboWORLD::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  modelname_ = model_->GetName();
  namespace_.clear();
  terminated_ = false;


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // load parameters from sdf
  if (_sdf->HasElement("modelNamespace"))
      namespace_ = _sdf->GetElement("modelNamespace")->Get<std::string>();


  node_handle_ = new ros::NodeHandle("worldplugin");
  pub_score_ = node_handle_->advertise<std_msgs::String>("score111", 1, true);  // set latch true

  visual_sub_ = node_handle_->subscribe<std_msgs::String>("/test",1,&GazeboWORLD::VisualCallback,this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWORLD::Update, this));
}
static bool pause = false;
void GazeboWORLD::VisualCallback(const std_msgs::String isvisible)
{
    if(isvisible.data == modelname_)
    {
        pause = !pause;
        math::Pose tmppose = model_->GetWorldPose();
        if(pause)
        {
            tmppose.pos.z = 1000;
            model_->SetStatic(false);
            world_->StepWorld(20);
            model_->SetWorldPose(tmppose);
            world_->SetPaused(true);
            world_->StepWorld(20);
            model_->SetAutoDisable(true);
        }
        else
        {
            tmppose.pos.z = 0;
            model_->SetWorldPose(tmppose);
            world_->StepWorld(20);
            model_->SetStatic(true);
            world_->SetPaused(false);
        }
    }

    std_msgs::String test;
    test.data = link_name_;
    pub_score_.publish(test);
}
////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboWORLD::Update()
{    
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboWORLD::Reset()
{
  state_stamp_ = ros::Time();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboWORLD)

}  // namespace gazebo
