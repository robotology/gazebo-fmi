/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef GAZEBO_FMI_SDF_CONFIGURATION_PARSING_HH
#define GAZEBO_FMI_SDF_CONFIGURATION_PARSING_HH

#include <string>
#include <vector>

#include <sdf/Element.hh>

namespace gazebo_fmi
{

/**
 * \brief Parse the input/output variables names from the variable_names SDF element.
 *
 * This method searches for elements in the form:
 *
 * <variable_names>
 *   <defaultVariableName name="newVariableName"/>
 * </variable_names>
 *
 * If no elements of this kind is found, the variable names will just be the default ones.
 *
 * @param[in] sdf sdf::ElementPtr of the parent of the variable_names element
 * @param[in] defaultInputVariableNames default names for input variables
 * @param[out] inputVariableNames actual names for the input variables
 * @param[in] defaultOutputVariableNames default names for output variables
 * @param[out] outputVariableNames default names for output variables
 * @return true if all went well, false if there was some error in parsing.
 * @note if no variable_names element is found in the provided SDF::ElementPtr,
 *       no error is raised, and the variable names will be the default ones.
 */
bool parseVariableNamesSDFElement(sdf::ElementPtr sdf,
                                  const std::vector<std::string>& defaultInputVariableNames,
                                  std::vector<std::string>& inputVariableNames,
                                  const std::vector<std::string>& defaulOutputVariableNames,
                                  std::vector<std::string>& ouputVariableNames);


}

#endif
