/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "RocketPlugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)

////////////////////////////////////////////////////////////////////////////////
RocketPlugin::RocketPlugin()
{
  this->cmds.fill(0.0f);

  // PID default parameters.
  this->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
  this->propellerPID.SetCmd(0.0);

  for (auto &pid : this->controlSurfacesPID)
  {
    pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
    pid.SetCmd(0.0);
  }
}

/////////////////////////////////////////////////
RocketPlugin::~RocketPlugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool RocketPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::JointPtr &_joint)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string jointName = _sdf->Get<std::string>(_sdfParam);
  _joint = this->model->GetJoint(jointName);
  if (!_joint)
  {
    gzerr << "Failed to find joint [" << jointName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool RocketPlugin::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf,
    physics::LinkPtr &_link)
{
  // Read the required plugin parameters.
  if (!_sdf->HasElement(_sdfParam))
  {
    gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
    return false;
  }

  std::string linkName = _sdf->Get<std::string>(_sdfParam);
  _link = this->model->GetLink(linkName);
  if (!_link)
  {
    gzerr << "Failed to find link [" << linkName
          << "] aborting plugin load." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void RocketPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "RocketPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "RocketPlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the required parameter for the propeller max RPMs.
  //if (!_sdf->HasElement("propeller_max_rpm"))
  //{
    //gzerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
    //return;
  //}
  //this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
  //if (this->propellerMaxRpm == 0)
  //{
    //gzerr << "Maximum propeller RPMs cannot be 0" << std::endl;
    //return;
  //}

  // Read the required joint name parameters.
  std::vector<std::string> requiredParams = {"fin1", "fin2", "fin3", "fin4"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
      return;
  }

  // Find body link to apply propulsion forces
  if (!this->FindLink("body", _sdf, this->body)) {
    return;
  }

  // Overload the PID parameters if they are available.
  if (_sdf->HasElement("propeller_p_gain"))
    this->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));

  if (_sdf->HasElement("propeller_i_gain"))
    this->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));

  if (_sdf->HasElement("propeller_d_gain"))
    this->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));

  if (_sdf->HasElement("surfaces_p_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
  }

  if (_sdf->HasElement("surfaces_i_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
  }

  if (_sdf->HasElement("surfaces_d_gain"))
  {
    for (auto &pid : this->controlSurfacesPID)
      pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&RocketPlugin::Update, this, std::placeholders::_1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<msgs::Rocket>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &RocketPlugin::OnControl, this);

  gzlog << "Rocket ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void RocketPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    double dt = (curTime - this->lastControllerUpdateTime).Double();
    this->UpdatePIDs(dt);

    auto inertial = this->body->GetInertial();
    float m = inertial->Mass();
    float m_dot = 0.1;
    float m_empty = 0.2;
    m = m - m_dot*dt;
    if (m < m_empty) {
      m = m_empty;
      m_dot = 0;
    }
    float ve = 2000;
    this->body->AddRelativeForce(ignition::math::Vector3d(0, 0, m_dot*ve));
    inertial->SetMass(m);
    inertial->SetInertiaMatrix(m, m, m, 0, 0, 0);
    this->PublishState();
    this->lastControllerUpdateTime = curTime;
  }
  

}

/////////////////////////////////////////////////
void RocketPlugin::OnControl(ConstRocketPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_fin1())
    this->cmds[kFin1] = _msg->cmd_fin1();
  if (_msg->has_cmd_fin2())
    this->cmds[kFin2] = _msg->cmd_fin2();
  if (_msg->has_cmd_fin3())
    this->cmds[kFin3] = _msg->cmd_fin3();
  if (_msg->has_cmd_fin4())
    this->cmds[kFin4] = _msg->cmd_fin4();
}

/////////////////////////////////////////////////
void RocketPlugin::UpdatePIDs(double _dt)
{
  // Position PID for the control surfaces.
  for (size_t i = 0; i < this->controlSurfacesPID.size(); ++i)
  {
    double pos = this->joints[i]->Position(0);
    double error = pos - this->cmds[i];
    double force = this->controlSurfacesPID[i].Update(error, _dt);
    this->joints[i]->SetForce(0, force);
  }
}

/////////////////////////////////////////////////
void RocketPlugin::PublishState()
{
  // Read the current state.
  float fin1 = this->joints[kFin1]->Position(0);
  float fin2 = this->joints[kFin2]->Position(0);
  float fin3 = this->joints[kFin3]->Position(0);
  float fin4 = this->joints[kFin4]->Position(0);

  msgs::Rocket msg;
  // Set the observed state.
  msg.set_fin1(fin1);
  msg.set_fin2(fin2);
  msg.set_fin3(fin3);
  msg.set_fin4(fin4);

  // Set the target state.
  msg.set_cmd_fin1(this->cmds[kFin1]);
  msg.set_cmd_fin2(this->cmds[kFin2]);
  msg.set_cmd_fin3(this->cmds[kFin3]);
  msg.set_cmd_fin4(this->cmds[kFin4]);

  this->statePub->Publish(msg);
}
