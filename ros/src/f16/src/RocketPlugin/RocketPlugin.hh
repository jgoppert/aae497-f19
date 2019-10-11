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

#ifndef GAZEBO_PLUGINS_RocketPLUGIN_HH_
#define GAZEBO_PLUGINS_RocketPLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/transport/Node.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include "casadi/CasadiFunc.hpp"
#include "casadi_gen.h"

namespace gazebo
{

  class GAZEBO_VISIBLE RocketPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: RocketPlugin();

    /// \brief Destructor.
    public: ~RocketPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Read an SDF parameter with a joint name and initialize a pointer
    /// to this joint.
    /// \param[in] _sdfParam SDF parameter containing a joint name.
    /// \param[in] _sdf Pointer to the SDF element containing the parameters.
    /// \param[out] _joint Pointer to the joint to be initialized.
    /// \return True if the SDF parameter is found and the joint name is found,
    ///         false otherwise.
    private: bool FindJoint(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::JointPtr &_joint);

    /// \brief Read an SDF parameter with a joint name and initialize a pointer
    /// to this joint.
    /// \param[in] _sdfParam SDF parameter containing a joint name.
    /// \param[in] _sdf Pointer to the SDF element containing the parameters.
    /// \param[out] _link Pointer to the link to be initialized.
    /// \return True if the SDF parameter is found and the link name is found,
    ///         false otherwise.
    private: bool FindLink(const std::string &_sdfParam,
        sdf::ElementPtr _sdf, physics::LinkPtr &_link);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief keep track of controller update sim-time.
    private: gazebo::common::Time lastUpdateTime;

    /// \brief Controller update mutex.
    private: std::mutex mutex;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition node used for using Gazebo communications.
    private: ignition::transport::Node nodeIgn;

    /// \brief Ignition Publisher.
    private: ignition::transport::Node::Publisher statePubIgn;

    /// \brief Motor Link
    private: physics::LinkPtr body;

	/// \brief Our casadi function
    private: CasadiFunc rocket_force_moment;
  };
}
#endif
