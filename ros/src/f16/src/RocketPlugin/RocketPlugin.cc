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
#include "math.h"
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include "RocketPlugin.hh"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)

////////////////////////////////////////////////////////////////////////////////
RocketPlugin::RocketPlugin():
    state_from_gz(state_from_gz_functions()),
    rocket_u_to_fin(rocket_u_to_fin_functions()),
    rocket_control(rocket_control_functions()),
    rocket_force_moment(rocket_force_moment_functions())
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

  if (!this->FindLink("body", _sdf, this->body)) {
    GZ_ASSERT(false, "RocketPlugin failed to find body");
  }

  // Find body link to apply
  for (int i=0; i<4; i++ ) {
      std::string name = "fin" + std::to_string(i);
      if (!this->FindJoint(name, _sdf, this->fin[i])) {
        std::string error = "RocketPlugin failed to find " + name;
        GZ_ASSERT(false, error.c_str());
      }
  }

  // Disable gravity, we will handle this in plugin
  this->body->SetGravityMode(false);
  this->body->GetInertial()->SetMass(0.8); //  set your total mass with fuel here


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
  double t = this->model->GetWorld()->SimTime().Double();

  if (curTime > this->lastUpdateTime)
  {
    // elapsed time
    double dt = (curTime - this->lastUpdateTime).Double();
    this->lastUpdateTime = curTime;

    // parameters
    const double g = 9.8;
    const double Jx = 0.05;
    const double Jy = 1;
    const double Jz = 1;
    const double Jxz = 0;
    const double ve = 350;
    const double l_fin = 1.0;
    const double w_fin = 0.05;
    const double CL_alpha = 2*M_PI;
    const double CL0 = 0;
    const double CD0 = 0.01;
    const double K = 0.01;
    const double s = 0.05;
    const double rho = 1.225;
    const double m_empty = 0.2;
    const double l_motor = 1.0;
    double p[16] = {g, Jx, Jy, Jz, Jxz, ve, l_fin, w_fin, CL_alpha, CL0, CD0, K, s, rho, m_empty, l_motor};

    // state
    auto inertial = this->body->GetInertial();
    double m = inertial->Mass();
    auto vel_ENU = this->body->RelativeLinearVel();
    auto omega_ENU = this->body->RelativeAngularVel();
    auto pose = this->body->WorldPose();
    auto q_ENU_FLT = pose.Rot();
    auto pos_ENU = pose.Pos();
    double m_fuel = m - m_empty;

    // native gazebo state
    double x_gz[14] = {
      omega_ENU.X(), omega_ENU.Y(), omega_ENU.Z(),
      q_ENU_FLT.W(), q_ENU_FLT.X(), q_ENU_FLT.Y(), q_ENU_FLT.Z(),
      vel_ENU.X(), vel_ENU.Y(), vel_ENU.Z(),
      pos_ENU.X(), pos_ENU.Y(), pos_ENU.Z(),
      m_fuel};

    // state from gazebo state
    double x[14];
    state_from_gz.arg(0, x_gz);
    state_from_gz.res(0, x);
    state_from_gz.eval();

    // control
    double u[4];
    rocket_control.arg(0, x);
    rocket_control.arg(1, p);
    rocket_control.arg(2, &t);
    rocket_control.arg(3, &dt);
    rocket_control.res(0, u);
    rocket_control.eval();

    // force moment
    double F_FLT[3];
    double M_FLT[3];
    rocket_force_moment.arg(0, x);
    rocket_force_moment.arg(1, u);
    rocket_force_moment.arg(2, p);
    rocket_force_moment.res(0, F_FLT);
    rocket_force_moment.res(1, M_FLT);
    rocket_force_moment.eval();

    // set joints
    double fin[4];
    rocket_u_to_fin.arg(0, u);
    rocket_u_to_fin.res(0, fin);
    rocket_u_to_fin.eval();
    for (int i=0; i<4; i++) {
      this->fin[i]->SetPosition(0, fin[i]);
    }

    // integration for rocket mass flow
    double m_dot = u[0];
    m -= m_dot*dt;
    if (m < m_empty) {
      m = m_empty;
      m_dot = 0;
    }
    inertial->SetMass(m);
    inertial->SetInertiaMatrix(Jx + m_fuel*l_motor*l_motor, Jy + m_fuel*l_motor*l_motor, Jz, 0, 0, 0);

    // debug
    for (int i=0; i<14; i++) {
      gzdbg << "x[" << i << "]: " << x[i] << "\n";
    }
    for (int i=0; i<4; i++) {
      gzdbg << "u[" << i << "]: " << u[i] << "\n";
    }
    for (int i=0; i<15; i++) {
      gzdbg << "p[" << i << "]: " << p[i] << "\n";
    }
    gzdbg << std::endl;
    gzdbg << "mass: " << m << std::endl;
    gzdbg << "force: " << F_FLT[0] << " " << F_FLT[1] << " " << F_FLT[2] << std::endl;
    gzdbg << "moment: " << M_FLT[0] << " " << M_FLT[1] << " " << M_FLT[2] << std::endl;
    gzdbg << "rate: " << dt << std::endl << std::endl;

    // apply forces and moments
    for (int i=0; i<3; i++) {
      // check that forces/moments finite before we apply them
      GZ_ASSERT(isfinite(F_FLT[i]), "non finite force");
      GZ_ASSERT(isfinite(M_FLT[i]), "non finite moment");
    }
    this->body->AddRelativeForce(ignition::math::v4::Vector3d(F_FLT[0], F_FLT[1], F_FLT[2]));
    this->body->AddRelativeTorque(ignition::math::v4::Vector3d(M_FLT[0], M_FLT[1], M_FLT[2]));
  }
}
