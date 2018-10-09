/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <gazebo_fmi/SDFConfigurationParsing.hh>

#include <gazebo/common/Console.hh>

namespace gazebo_fmi
{

bool parseVariableNamesSDFElementHelper(sdf::ElementPtr sdf_elem,
                                        const std::vector<std::string>& defaultVariableNames,
                                        std::vector<std::string>& variableNames)
{
  // By default, use default variable names
  variableNames = defaultVariableNames;

  if (sdf_elem->HasElement("variable_names"))
  {
    sdf::ElementPtr variable_names_elem = sdf_elem->GetElement("variable_names");

    // Iterate over all input variables
    for (std::string const& defaultVariableName : defaultVariableNames)
    {
      if (variable_names_elem->HasElement(defaultVariableName))
      {
        // If the element is present but without "name" attribute, print an error
        sdf::ElementPtr variable_elem = variable_names_elem->GetElement(defaultVariableName);

        if (!variable_elem->HasAttribute("name"))
        {
          gzerr << "gazebo_fmi: tag " << defaultVariableName << " has no required attribute name." << std::endl;
          return false;
        }
        else
        {
          size_t variableIndex =
            find(defaultVariableNames.begin(), defaultVariableNames.end(), defaultVariableName)
                 - defaultVariableNames.begin();
          variableNames[variableIndex] = variable_elem->GetAttribute("name")->GetAsString();
        }
      }
    }
  }

  return true;
}



bool parseVariableNamesSDFElement(sdf::ElementPtr sdf_elem,
                                  const std::vector<std::string>& defaultInputVariableNames,
                                  std::vector<std::string>& inputVariableNames,
                                  const std::vector<std::string>& defaulOutputVariableNames,
                                  std::vector<std::string>& outputVariableName)
{
  bool ok = true;
  ok = ok && parseVariableNamesSDFElementHelper(sdf_elem, defaultInputVariableNames, inputVariableNames);
  ok = ok && parseVariableNamesSDFElementHelper(sdf_elem, defaulOutputVariableNames, outputVariableName);
  return ok;
}

}
