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
#include "F16Plugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(F16Plugin)

////////////////////////////////////////////////////////////////////////////////
F16Plugin::F16Plugin()
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
F16Plugin::~F16Plugin()
{
  this->updateConnection.reset();
}

/////////////////////////////////////////////////
bool F16Plugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf,
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
void F16Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "F16Plugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "F16Plugin _sdf pointer is NULL");
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
  std::vector<std::string> requiredParams = {"left_aileron",
    "right_aileron", "left_elevator", "right_elevator", "rudder"};

  for (size_t i = 0; i < requiredParams.size(); ++i)
  {
    if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]))
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
    std::bind(&F16Plugin::Update, this, std::placeholders::_1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  std::string prefix = "~/" + this->model->GetName() + "/";
  this->statePub = this->node->Advertise<msgs::F16>(prefix + "state");
  this->controlSub = this->node->Subscribe(prefix + "control",
    &F16Plugin::OnControl, this);

  gzlog << "F16 ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void F16Plugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // Update the control surfaces and publish the new state.
    this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
    this->PublishState();

    this->lastControllerUpdateTime = curTime;
  }
}

/////////////////////////////////////////////////
void F16Plugin::OnControl(ConstF16Ptr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (_msg->has_cmd_left_aileron())
    this->cmds[kLeftAileron] = _msg->cmd_left_aileron();
  if (_msg->has_cmd_right_aileron())
    this->cmds[kRightAileron] = _msg->cmd_right_aileron();
  if (_msg->has_cmd_left_elevator())
    this->cmds[kLeftElevator] = _msg->cmd_left_elevator();
  if (_msg->has_cmd_right_elevator())
    this->cmds[kRightElevator] = _msg->cmd_right_elevator();
  if (_msg->has_cmd_rudder())
    this->cmds[kRudder] = _msg->cmd_rudder();
}

/////////////////////////////////////////////////
void F16Plugin::UpdatePIDs(double _dt)
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
void F16Plugin::PublishState()
{
  // Read the current state.
  float leftAileron = this->joints[kLeftAileron]->Position(0);
  float rightAileron = this->joints[kRightAileron]->Position(0);
  float leftElevator = this->joints[kLeftElevator]->Position(0);
  float rightElevator = this->joints[kRightElevator]->Position(0);
  float rudder = this->joints[kRudder]->Position(0);

  msgs::F16 msg;
  // Set the observed state.
  msg.set_left_aileron(leftAileron);
  msg.set_right_aileron(rightAileron);
  msg.set_left_elevator(leftElevator);
  msg.set_right_elevator(rightElevator);
  msg.set_rudder(rudder);

  // Set the target state.
  msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
  msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
  msg.set_cmd_left_elevator(this->cmds[kLeftElevator]);
  msg.set_cmd_right_elevator(this->cmds[kRightElevator]);
  msg.set_cmd_rudder(this->cmds[kRudder]);

  this->statePub->Publish(msg);
}
