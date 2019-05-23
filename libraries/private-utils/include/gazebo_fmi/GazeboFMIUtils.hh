/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef GAZEBO_FMI_UTILS
#define GAZEBO_FMI_UTILS

#include <gazebo/physics/Joint.hh>

namespace gazebo_fmi
{

inline double ComputeJointAcceleration(gazebo::physics::JointPtr jointPtr)
{
    if (jointPtr->GetType() & gazebo::physics::Base::HINGE_JOINT)
    {
        // Compute joint acceleration
        // For a link `L`, the method ignition::math::Vector3d WorldAngularAccel () const
        // returns the angular acceleration \$ {}^A \omega_{A,L} \$.
        // Given a hinge/revolute joint ${P, C}$ connecting the links $P$ (parent) and $C$ (child), the method
        // virtual ignition::math::Vector3d GlobalAxis( unsigned int  _index) const
        // returns the angular part of the joint motion subspace \$  ^A s_{P,C} \in \mathbb{R}^6  \$
        // that we indicate with \$ {}^A a_{P, C} \in \mathbb{R}^3 \$ (axis in the code for readability).
        //
        // The joint acceleration \$\ddot{\theta} \in \mathbb{R}\$ can then be computed as:
        // \$
        // \ddot{\theta} =  \left( ^C s_{P,C} \right)^T  {}^C \dot{\omega}\_{P,C} =  \left( ^C s_{P,C} \right)^T  ( {}^C \dot{\omega}\_{A,C} - {}^C \dot{\omega}\_{A,P} )  =
        // \left( ^A s_{P,C} \right)^T ( {}^A \dot{\omega}\_{A,C} - {}^A \dot{\omega}\_{A,P} )
        // \$

        gazebo::physics::LinkPtr parent = jointPtr->GetParent();
        gazebo::physics::LinkPtr child = jointPtr->GetChild();
#if GAZEBO_MAJOR_VERSION >=8
        ignition::math::Vector3d A_axis_P_C = jointPtr->GlobalAxis(0u);
        ignition::math::Vector3d A_domega_A_P = parent->WorldAngularAccel();
        ignition::math::Vector3d A_domega_A_C = child->WorldAngularAccel();
#else
        gazebo::math::Vector3 A_axis_P_C = jointPtr->GetGlobalAxis(0u);
        gazebo::math::Vector3 A_domega_A_P = parent->GetWorldAngularAccel();
        gazebo::math::Vector3 A_domega_A_C = child->GetWorldAngularAccel();
#endif
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

}

#endif

