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
#include "rocket.pb.h"
#include "casadi/CasadiFunc.hpp"
#include "casadi_gen.h"

namespace gazebo
{

  typedef const boost::shared_ptr<const msgs::Rocket> ConstRocketPtr;
  /// \brief Allow moving the control surfaces of a Rocket plane. This
  /// plugin might be used with other models that have similar control surfaces.
  ///
  /// The plugin requires the following parameters:
  /// <propeller>         Name of the joint controlling the propeller spin.
  /// <propeller_max_rpm> Maximum angular speed in rpm.
  /// <left_aileron>      Name of the joint controlling the left aileron.
  /// <left_flap>         Name of the joint controlling the left flap.
  /// <right_aileron>     Name of the joint controlling the right aileron.
  /// <right_flap>        Name of the joint controlling the right flap.
  /// <elevators>         Name of the joint controlling the rear elevators.
  /// <rudder>            Name of the joint controlling the rudder.
  ///
  /// The following parameters are optional:
  /// <propeller_p_gain> P gain for the PID that controls the propeller's speed.
  /// <propeller_i_gain> I gain for the PID that controls the propeller's speed.
  /// <propeller_d_gain> D gain for the PID that controls the propeller's speed.
  /// <surfaces_p_gain> P gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_i_gain> I gain for the PID that controls the position of the
  ///                   control surfaces.
  /// <surfaces_d_gain> D gain for the PID that controls the position of the
  ///                   control surfaces.
  ///
  /// The plugin will be subscribed to the following topic:
  /// "~/<model_name>/control" The expected value is a Rocket message.
  ///
  /// The plugin will advertise the following topic with the current state:
  /// "~/<model_name>/state"
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

    /// \brief Motor Link
    private: physics::LinkPtr motor;

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

	/// \brief Our casadi function
    private: CasadiFunc _double_this;
  };
}
#endif
