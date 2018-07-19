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
/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include "FMIActuatorPlugin.hh"

#include <functional>

#include <experimental/filesystem>


using namespace gazebo_fmi;

//////////////////////////////////////////////////
void FMIActuatorPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Parse parameters
    if (!this->ParseParameters(_parent, _sdf))
    {
        gzerr << "FMIActuatorPlugin: error in parsing SDF parameters, plugin loading failed."
              << std::endl;
    }

    // Try to open fmu for all joints in the plugin
    if (!this->LoadFMUs(_parent))
    {
        gzerr << "FMIActuatorPlugin: error in loading FMUs, plugin loading failed."
              << std::endl;
        return;
    }

    // Set up a physics update callback
    this->connections.push_back(gazebo::event::Events::ConnectBeforePhysicsUpdate(
      boost::bind(&FMIActuatorPlugin::BeforePhysicsUpdateCallback, this, _1)));
}

//////////////////////////////////////////////////
// Read the SDF
bool FMIActuatorPlugin::ParseParameters(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("actuator"))
  {
    // TODO(traversaro) : support list of actuator elements
    sdf::ElementPtr elem = _sdf->GetElement("actuator");
    {

      // actuator name is currently an optional property
      if (elem->HasElement("name"))
        actuator.name = elem->Get<std::string>("name");

      if (!elem->HasElement("joint"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, got actuator element without joint."
               << std::endl;
        return false;
      }
      std::string jointName = elem->Get<std::string>("joint");

      joint=FindJointInModel(jointName,_parent);
      // Store pointer to the joint we will actuate
      if (!joint)
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, actuator joint " << jointName << " does not "
               << "exist!" << std::endl;
        return false;
      }

      if (!elem->HasElement("fmu"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, fmu parameter not found."
               << std::endl;
        return false;
      }

      actuator.fmuAbsolutePath =  gazebo::common::SystemPaths::Instance()->FindFile(elem->Get<std::string>("fmu"));
      if (!std::experimental::filesystem::exists(actuator.fmuAbsolutePath))
      {
        gzerr << "FMIActuatorPlugin: Impossible to find FMU named " << elem->Get<std::string>("fmu")
              << " in the GAZEBO_RESOURCE_PATH directories" << std::endl;
        return false;
      }

      if (!elem->HasElement("actuatorInputName"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, actuatorInputName parameter not found."
               << std::endl;
        return false;
      }
      actuator.actuatorInputName = elem->Get<std::string>("actuatorInputName");

      if (!elem->HasElement("jointPositionName"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, jointPositionName parameter not found."
               << std::endl;
        return false;
      }
      actuator.jointPositionName = elem->Get<std::string>("jointPositionName");


      if (!elem->HasElement("jointVelocityName"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, jointVelocityName parameter not found."
               << std::endl;
        return false;
      }
      actuator.jointVelocityName = elem->Get<std::string>("jointVelocityName");

      if (!elem->HasElement("jointTorqueName"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, jointTorqueName parameter not found."
               << std::endl;
        return false;
      }
      actuator.jointTorqueName = elem->Get<std::string>("jointTorqueName");

    }
    return true;
  }
  else
  {
      gzerr << "FMIActuatorPlugin: Invalid SDF, no actuator tag found." << std::endl;
      return false;
  }
}


//////////////////////////////////////////////////
bool FMIActuatorPlugin::LoadFMUs(gazebo::physics::ModelPtr _parent)
{
    double simulatedTimeInSeconds  = _parent->GetWorld()->SimTime().Double();


    //for (unsigned int i = 0; i < this->actuators.size(); i++)
    {
        // FMUActuatorProperties& actuator = this->actuators[i];
        // gazebo::physics::JointPtr joint
        std::string instanceName = joint->GetScopedName()+"_fmuTransmission";
        bool ok = actuator.fmu.load(actuator.fmuAbsolutePath, instanceName, simulatedTimeInSeconds);
        if (!ok) {
            return false;
        }

        // Configure input variables
        std::vector<std::string> inputVariablesNames;
        inputVariablesNames.push_back(actuator.actuatorInputName);
        inputVariablesNames.push_back(actuator.jointPositionName);
        inputVariablesNames.push_back(actuator.jointVelocityName);

        // Get references for input variables
        ok = actuator.fmu.getInputVariableRefs(inputVariablesNames, actuator.inputVarReferences);
        if (!ok) {
            return false;
        }
        actuator.inputVarBuffers.resize(actuator.inputVarReferences.size());

        // Configure output variables
        std::vector<std::string> outputVariablesNames;
        outputVariablesNames.push_back(actuator.jointTorqueName);

        // Get references for output variables
        ok = actuator.fmu.getOutputVariableRefs(outputVariablesNames, actuator.outputVarReferences);
        if (!ok) {
            return false;
        }
        actuator.outputVarBuffers.resize(actuator.outputVarBuffers.size());
    }

    return true;
}


//////////////////////////////////////////////////
void FMIActuatorPlugin::BeforePhysicsUpdateCallback(const gazebo::common::UpdateInfo & updateInfo)
{
    // TODO(traversaro): review this part
    double simulatedTimeInSeconds  = updateInfo.simTime.Double();
    auto world = gazebo::physics::get_world(updateInfo.worldName);
    double stepSizeInSeconds = world->Physics()->GetMaxStepSize();


    // Update the stored joints according to the desired model.
    //for (unsigned int i = 0; i < this->joints.size(); i++)
    {
        //FMUActuatorProperties& actuator = this->actuators[i];

        double actuatorInput = joint->GetForce(0u);
        const double position = joint->Position(0u);
        const double velocity = joint->GetVelocity(0u);

        // This order should be coherent with the order defined in LoadFMUs
        actuator.inputVarBuffers[0] = actuatorInput;
        actuator.inputVarBuffers[1] = position;
        actuator.inputVarBuffers[2] = velocity;

        // Set input
        bool ok = actuator.fmu.setInputVariables(actuator.inputVarReferences, actuator.inputVarBuffers);

        // Run fmu simulation
        ok = ok && actuator.fmu.doStep(simulatedTimeInSeconds, stepSizeInSeconds);

        // Get ouput
        ok = ok && actuator.fmu.getOutputVariables(actuator.outputVarReferences, actuator.outputVarBuffers);

        if (!ok)
        {
            gzerr << "gazebo_fmi: Failure in simulating trasmission of " << joint->GetScopedName() << std::endl;
        }

        // This order should be coherent with the order defined in LoadFMUs
        double jointTorque = actuator.outputVarBuffers[0];

        // Note: at least in ODE, two consecutive SetForce calls are added to the same buffer:
        // for this reason, to overwrite the previous value with subtract it
        joint->SetForce(0u, jointTorque-actuatorInput);
    }
}

//////////////////////////////////////////////////
gazebo::physics::JointPtr FMIActuatorPlugin::FindJointInModel(const std::string& jointName,const gazebo::physics::ModelPtr _parent)
{
    joint=nullptr;
    for (auto currentJoint:_parent->GetJoints())
    {
        const std::string& currentJointName=currentJoint->GetName();

        if(currentJointName==jointName)//if you use the scoped name
        {
            joint=currentJoint;
            break;
        }

        std::size_t lastJointNamePart = currentJointName.find_last_of("::");
        std::string currentJointNameUnscoped;
        if (lastJointNamePart==std::string::npos)//in the model the joint names are unscoped
        {
            currentJointNameUnscoped=currentJointName;
        }
        else//in the model the joint names are scoped
        {
            currentJointNameUnscoped=currentJointName.substr(lastJointNamePart+1);
        }

        if (currentJointNameUnscoped!=jointName)
            continue;

        joint=currentJoint;
        break;
    } 
    return joint;
} 
