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

    if(joint)
        gzlog << "FMIActuatorPlugin load for:"<<joint->GetScopedName()<<std::endl;

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

      // Check type
      bool typeOk = this->CheckJointType(joint);
      if (!typeOk)
      {
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

      if (!elem->HasElement("jointAccelerationName"))
      {
         gzerr << "FMIActuatorPlugin: Invalid SDF, jointAccelerationName parameter not found."
               << std::endl;
        return false;
      }
      actuator.jointAccelerationName = elem->Get<std::string>("jointAccelerationName");

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
        inputVariablesNames.push_back(actuator.jointAccelerationName);

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
bool FMIActuatorPlugin::CheckJointType(gazebo::physics::JointPtr jointPtr)
{
    if (jointPtr->DOF() != 1)
    {
        gzerr << "FMIActuatorPlugin: joint " << jointPtr->GetScopedName() << " has "
              << jointPtr->DOF() << ", only joint with 1 DOFs are currently supported." << std::endl;
        return false;
    }

    if (!(jointPtr->GetType() & gazebo::physics::Base::HINGE_JOINT))
    {
        gzwarn << "FMIActuatorPlugin: joint " << jointPtr->GetScopedName() << " is not an HingeJoint. " << std::endl;
        gzwarn << "Feedback on joint acceleration is currently available only for HingeJoint, "
               << "so it is possible to use this joint only with an FMU that is not using acceleration feedback." << std::endl;
        gzwarn << "See https://github.com/robotology/gazebo-fmi/issues/17 for more details." << std::endl;
    }

    return true;
}

//////////////////////////////////////////////////
double FMIActuatorPlugin::GetJointAcceleration(gazebo::physics::JointPtr jointPtr)
{
    if (jointPtr->GetType() & gazebo::physics::Base::HINGE_JOINT)
    {
        // Compute joint acceleration
        // For a link `L`, the method ignition::math::Vector3d WorldAngularAccel () const
        // returns the angular acceleration \$ {}^A \omega_{A,L} \$.
        // Given a hinge/revolute joint ${P, C}$ connecting the links $P$ (parent) and $C$ (child), the method
        // virtual ignition::math::Vector3d GlobalAxis( unsigned int  _index) const
        // returns the angular part of the joint motion subspace \$  ^A s_{P,C} \in \mathbb{R}^6  \$
        // that we indicate with \$ {}^A a_{P, C} \in \mathbb{R}^3 \$.
        //
        // The joint acceleration \$\ddot{\theta} \in \mathbb{R}\$ can then be computed as:
        // \$
        // \ddot{\theta} =  \left( ^C s_{P,C} \right)^T  {}^C \dot{\omega}\_{P,C} =  \left( ^C s_{P,C} \right)^T  ( {}^C \dot{\omega}\_{A,C} - {}^C \dot{\omega}\_{A,P} )  =
        // \left( ^A s_{P,C} \right)^T ( {}^A \dot{\omega}\_{A,C} - {}^A \dot{\omega}\_{A,P} )
        // \$

        ignition::math::Vector3d A_axis_P_C = jointPtr->GlobalAxis(0u);
        gazebo::physics::LinkPtr parent = jointPtr->GetParent();
        gazebo::physics::LinkPtr child = jointPtr->GetChild();
        ignition::math::Vector3d A_domega_A_P = parent->WorldAngularAccel();
        ignition::math::Vector3d A_domega_A_C = child->WorldAngularAccel();
        return A_axis_P_C.Dot(A_domega_A_C-A_domega_A_P);
    }
    else
    {
        // For other joints, just return 0 as acceleration
        // Appropriate warning are printed during the load, and this ensures that
        // the models that do not use acceleration (such as compliant models) still work fine
        return 0.0;
    }
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
        const double acceleration = this->GetJointAcceleration(joint);

        // This order should be coherent with the order defined in LoadFMUs
        actuator.inputVarBuffers[0] = actuatorInput;
        actuator.inputVarBuffers[1] = position;
        actuator.inputVarBuffers[2] = velocity;
        actuator.inputVarBuffers[3] = acceleration;          

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
