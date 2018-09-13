/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include "FMISingleBodyFluidDynamicsPlugin.hh"

#include <functional>

#include <experimental/filesystem>

#include <gazebo_fmi/SDFConfigurationParsing.hh>


using namespace gazebo_fmi;

namespace FMISingleBodyFluidDynamicsPluginNS
{
enum InputIndex
{
    relativeVelocity_x = 0,
    relativeVelocity_y,
    relativeVelocity_z,
    TotalInputs,
};

enum OutputIndex
{
    fluidDynamicForce_x = 0,
    fluidDynamicForce_y,
    fluidDynamicForce_z,
    fluidDynamicMoment_x,
    fluidDynamicMoment_y,
    fluidDynamicMoment_z,
    TotalOutputs,
};
}


//////////////////////////////////////////////////
void FMISingleBodyFluidDynamicsPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Configure default variable names
    m_fmu.m_inputVariablesDefaultNames.resize(FMISingleBodyFluidDynamicsPluginNS::TotalInputs);
    m_fmu.m_inputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_x] = "relativeVelocity_x";
    m_fmu.m_inputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_y] = "relativeVelocity_y";
    m_fmu.m_inputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_z] = "relativeVelocity_z";

    m_fmu.m_outputVariablesDefaultNames.resize(FMISingleBodyFluidDynamicsPluginNS::TotalOutputs);
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_x] = "fluidDynamicForce_x";
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_y] = "fluidDynamicForce_y";
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_z] = "fluidDynamicForce_z";
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_x] = "fluidDynamicMoment_x";
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_y] = "fluidDynamicMoment_y";
    m_fmu.m_outputVariablesDefaultNames[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_z] = "fluidDynamicMoment_z";

    // Bullet is not supported by this plugin, due to https://bitbucket.org/osrf/gazebo/issues/1476/implement-addxxxforce-for-bullet
    if (gazebo::physics::get_world()->Physics()->GetType() == "bullet")
    {
        gzerr << "FMISingleBodyFluidDynamicsPlugin: the bullet physics engine is not supported, as in Gazebo there is no way to excert an external force on a link." << std::endl;
        gzerr << "FMISingleBodyFluidDynamicsPlugin: see https://bitbucket.org/osrf/gazebo/issues/1476/implement-addxxxforce-for-bullet" << std::endl;
        return;
    }

    // Parse parameters
    if (!this->ParseParameters(_parent, _sdf))
    {
        gzerr << "FMISingleBodyFluidDynamicsPlugin: error in parsing SDF parameters, plugin loading failed."
              << std::endl;
    }

    if (!this->LoadFMUs(_parent))
    {
        gzerr << "FMISingleBodyFluidDynamicsPlugin: error in loading FMUs, plugin loading failed."
              << std::endl;
        return;
    }

    // Set up a physics update callback
    m_updateConnection =  gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&FMISingleBodyFluidDynamicsPlugin::WorldUpdateBeginCallback, this, _1));
}

//////////////////////////////////////////////////
// Read the SDF
bool FMISingleBodyFluidDynamicsPlugin::ParseParameters(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("single_body_fluid_dynamics"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("single_body_fluid_dynamics");
    {

      if (elem->HasElement("name"))
        m_fmu.name = elem->Get<std::string>("name");

      if (!elem->HasElement("link"))
      {
         gzerr << "FMISingleBodyFluidDynamicsPlugin: Invalid SDF, got actuator element without link."
               << std::endl;
        return false;
      }
      std::string linkName = elem->Get<std::string>("link");

      link=FindLinkInModel(linkName,_parent);

      // Store pointer to the joint we will actuate
      if (!link)
      {
         gzerr << "FMISingleBodyFluidDynamicsPlugin: Invalid SDF, link " << linkName << " does not "
               << "exist!" << std::endl;
        return false;
      }

      if (!elem->HasElement("fmu"))
      {
         gzerr << "FMISingleBodyFluidDynamicsPlugin: Invalid SDF, fmu parameter not found."
               << std::endl;
        return false;
      }

      m_fmu.fmuAbsolutePath =  gazebo::common::SystemPaths::Instance()->FindFile(elem->Get<std::string>("fmu"));
      if (!std::experimental::filesystem::exists(m_fmu.fmuAbsolutePath))
      {
        gzerr << "FMISingleBodyFluidDynamicsPlugin: Impossible to find FMU named " << elem->Get<std::string>("fmu")
              << " in the GAZEBO_RESOURCE_PATH directories" << std::endl;
        return false;
      }

      // Process variable_names element
      bool variableNamesOk = gazebo_fmi::parseVariableNamesSDFElement(elem,
                                                                      m_fmu.m_inputVariablesDefaultNames,
                                                                      m_fmu.m_inputVariablesNames,
                                                                      m_fmu.m_outputVariablesDefaultNames,
                                                                      m_fmu.m_outputVariablesNames);

      if (!variableNamesOk)
      {
        gzerr << "FMISingleBodyFluidDynamicsPlugin: failure in parsing variable_names tag" << std::endl;
        return false;
      }
    }
    return true;
  }
  else
  {
      gzerr << "FMISingleBodyFluidDynamicsPlugin: Invalid SDF, no single_body_fluid_fynamics tag found." << std::endl;
      return false;
  }
}

//////////////////////////////////////////////////
bool FMISingleBodyFluidDynamicsPlugin::LoadFMUs(gazebo::physics::ModelPtr _parent)
{
    double simulatedTimeInSeconds  = _parent->GetWorld()->SimTime().Double();

    std::string instanceName = link->GetScopedName()+"_fmuSingleBodyFluidDynamics";
    bool ok = m_fmu.fmu.load(m_fmu.fmuAbsolutePath, instanceName, simulatedTimeInSeconds);
    if (!ok) {
        return false;
    }

    // Get references for input variables
    ok = m_fmu.fmu.getInputVariableRefs(m_fmu.m_inputVariablesNames, m_fmu.inputVarReferences);
    if (!ok) {
        return false;
    }
    m_fmu.inputVarBuffers.resize(m_fmu.inputVarReferences.size());

    // Get references for output variables
    ok = m_fmu.fmu.getOutputVariableRefs(m_fmu.m_outputVariablesNames, m_fmu.outputVarReferences);
    if (!ok) {
        return false;
    }
    m_fmu.outputVarBuffers.resize(m_fmu.outputVarBuffers.size());

    return true;
}

//////////////////////////////////////////////////
void FMISingleBodyFluidDynamicsPlugin::WorldUpdateBeginCallback(const gazebo::common::UpdateInfo & updateInfo)
{
    // TODO(traversaro): review this part
    double simulatedTimeInSeconds  = updateInfo.simTime.Double();
    auto world = gazebo::physics::get_world(updateInfo.worldName);
    double stepSizeInSeconds = world->Physics()->GetMaxStepSize();

    // Get input: relative velocity in link frame
    // TODO: check orientation

    ignition::math::Vector3d worldRelativeVel = link->WorldLinearVel()-link->WorldWindLinearVel();
    ignition::math::Matrix3d link_R_world = ignition::math::Matrix3d(link->WorldPose().Rot()).Transposed();
    ignition::math::Vector3d linkRelativeVel = link_R_world*worldRelativeVel;

    // Set inputs
    m_fmu.inputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_x] = linkRelativeVel[0];
    m_fmu.inputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_y] = linkRelativeVel[1];
    m_fmu.inputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::relativeVelocity_z] = linkRelativeVel[2];
    bool ok = m_fmu.fmu.setInputVariables(m_fmu.inputVarReferences, m_fmu.inputVarBuffers);

    // Run fmu simulation
    ok = ok && m_fmu.fmu.doStep(simulatedTimeInSeconds, stepSizeInSeconds);

    // Get ouput
    ok = ok && m_fmu.fmu.getOutputVariables(m_fmu.outputVarReferences, m_fmu.outputVarBuffers);

    if (!ok)
    {
        gzerr << "gazebo_fmi: Failure in simulating single body fluid dynamics forces of link " << link->GetScopedName() << std::endl;
    }

    // This order should be coherent with the order defined in LoadFMUs
    ignition::math::Vector3d linkFluidForce, linkFluidMoment;
    linkFluidForce[0] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_x];
    linkFluidForce[1] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_y];
    linkFluidForce[2] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicForce_z];
    linkFluidMoment[0] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_x];
    linkFluidMoment[1] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_y];
    linkFluidMoment[2] = m_fmu.outputVarBuffers[FMISingleBodyFluidDynamicsPluginNS::fluidDynamicMoment_z];

    // Rotate the forces with the world orientation
    ignition::math::Matrix3d world_R_link = link_R_world.Transposed();
    ignition::math::Vector3d worldFluidForce = world_R_link*linkFluidForce;
    ignition::math::Vector3d worldFluidMoment = world_R_link*linkFluidMoment;

    link->AddForce(worldFluidForce);
    link->AddTorque(worldFluidMoment);
}

//////////////////////////////////////////////////
gazebo::physics::LinkPtr FMISingleBodyFluidDynamicsPlugin::FindLinkInModel(const std::string& linkName, const gazebo::physics::ModelPtr _parent)
{
    link=nullptr;
    for (gazebo::physics::LinkPtr currentLink: _parent->GetLinks())
    {
        const std::string& currentLinkName=currentLink->GetName();

        if(currentLinkName==linkName)
        {
            link=currentLink;
            break;
        }

        std::size_t lastJointNamePart = currentLinkName.find_last_of("::");
        std::string currentLinkNameUnscoped;
        if (lastJointNamePart==std::string::npos)
        {
            currentLinkNameUnscoped=currentLinkName;
        }
        else//in the model the joint names are scoped
        {
            currentLinkNameUnscoped=currentLinkName.substr(lastJointNamePart+1);
        }

        if (currentLinkNameUnscoped!=currentLinkName)
            continue;

        link=currentLink;
        break;
    }
    return link;
}
