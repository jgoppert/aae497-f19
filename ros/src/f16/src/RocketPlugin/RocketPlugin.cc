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
RocketPlugin::RocketPlugin():
    _double_this(double_this_functions())
{
    std::cout << "hello rocket plugin" << std::endl;
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

  // Find motor link to apply propulsion forces
  if (!this->FindLink("motor", _sdf, this->motor)) {
    GZ_ASSERT(false, "RocketPlugin failed to find motor");
  }

  // Find body link to apply
  if (!this->FindLink("body", _sdf, this->body)) {
    GZ_ASSERT(false, "RocketPlugin failed to find body");
  }

  // Update time.
  this->lastUpdateTime = this->model->GetWorld()->SimTime();

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&RocketPlugin::Update, this, std::placeholders::_1));

  gzlog << "Rocket ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void RocketPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  gazebo::common::Time curTime = this->model->GetWorld()->SimTime();

  if (curTime > this->lastUpdateTime)
  {
    double dt = (curTime - this->lastUpdateTime).Double();

    auto inertial = this->motor->GetInertial();
    float m = inertial->Mass();
    float m_dot = 0.1;
    float m_empty = 0.1;
    m = m - m_dot*dt;
    float P0 = 101325.0; // freestream pressure
    float Pe = 1.0*P0; // disable effect for now
    if (m < m_empty) {
      m = m_empty;
      m_dot = 0;
      Pe = P0;
    }

    float r = 0.1; // nozzle radius
    float A = M_PI*r*r; // nozzle exit area
    float ve = 320; // exit velocity, m/s, guess
    this->motor->AddRelativeForce(ignition::math::Vector3d(0, 0, m_dot*ve + A*(Pe - P0)));
    inertial->SetMass(m);
    inertial->SetInertiaMatrix(0.01, 0.01, 0.01, 0, 0, 0); // treat as point mass
    this->lastUpdateTime = curTime;

    std::cout << "hello rocket plugin" << std::endl;
    double states[14] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    double inputs[4] = {1, 2, 3, 4};
    double parameters[15] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    double F_aero[3] = {0, 0, 0};
    double F_prop[3] = {0, 0, 0};

    ignition::math::Vector3d vel = this->body->WorldLinearVel();
    _double_this.arg(0, states);
    _double_this.arg(1, inputs);
    _double_this.arg(2, parameters);
    _double_this.res(0, F_aero);
    _double_this.res(1, F_prop);
    _double_this.eval();
    std::cout << "F_aero" << F_aero[0] << F_aero[1] << F_aero[2] << std::endl;
    std::cout << "F_prop" << F_prop[0] << F_prop[1] << F_prop[2] << std::endl;
    std::cout << "velocity" << vel << std::endl;
  }
}
