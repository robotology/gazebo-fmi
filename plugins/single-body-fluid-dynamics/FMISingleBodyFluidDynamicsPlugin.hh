/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef GAZEBO_FMI_SINGLE_BODY_FLUID_DYNAMICS_PLUGIN_HH
#define GAZEBO_FMI_SINGLE_BODY_FLUID_DYNAMICS_PLUGIN_HH

#include <functional>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo_fmi/FMUCoSimulation.hh>

namespace gazebo_fmi
{
/// \brief Properties for a FMU for single body fluid dynamics
class FMUSingleBodyFluidDynamicsProperties
{
    /// \brief An identifier for the fmu
    public: std::string name;

    public: std::string fmuAbsolutePath;

    /// \brief Default input variable names
    public: std::vector<std::string> m_inputVariablesDefaultNames;

    /// \brief Actual input variable names, after parsing variable_names
    public: std::vector<std::string> m_inputVariablesNames;

    /// \brief Default output variable names
    public: std::vector<std::string> m_outputVariablesDefaultNames;

    /// \brief Actual output variable names, after parsing variable_names
    public: std::vector<std::string> m_outputVariablesNames;

    public: FMUCoSimulation fmu;
    public: std::vector<fmi2_value_reference_t> inputVarReferences;
    public: std::vector<fmi2_value_reference_t> outputVarReferences;
    public: std::vector<double> inputVarBuffers;
    public: std::vector<double> outputVarBuffers;

};

/// \brief Plugin for interaction between a single body and a surrounding fluid
class GAZEBO_VISIBLE FMISingleBodyFluidDynamicsPlugin : public gazebo::ModelPlugin
{
    /// Documentation inherited
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private: bool ParseParameters(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private: bool LoadFMUs(gazebo::physics::ModelPtr _parent);

    private: gazebo::physics::LinkPtr FindLinkInModel(const std::string& linkName, const gazebo::physics::ModelPtr _parent);

    /// \brief Callback on world update begin
    private: void WorldUpdateBeginCallback(const gazebo::common::UpdateInfo & updateInfo);

    /// \brief The link of which we want to simulate the fluid dynamic forces
    private: gazebo::physics::LinkPtr link;

    /// \brief FMU
    private: FMUSingleBodyFluidDynamicsProperties m_fmu;

    /// \brief Connections to events associated with this class.
    private: gazebo::event::ConnectionPtr m_updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(FMISingleBodyFluidDynamicsPlugin);
}

#endif
