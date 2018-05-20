/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef GAZEBO_FMI_FMU_COSIMULATION
#define GAZEBO_FMI_FMU_COSIMULATION

#include <memory>
#include <vector>

// For fmi2ValueReference
#include <FMI2/fmi2_types.h>


namespace gazebo_fmi
{
    class FMUCoSimulationPrivate;

    /// \brief Class wrapping
    class FMUCoSimulation
    {
    private:
        std::unique_ptr<FMUCoSimulationPrivate> m_pimpl;

    public:
        FMUCoSimulation();
        ~FMUCoSimulation();

        /// \brief Load specified FMU
        /// @return true if the FMU was loaded correctly, false otherwise
        bool load(const std::string& fmuAbsolutePath, const std::string& instanceName, const double startTimeInSeconds);

        /// \brief return true if the class contains a correctly loaded FMU
        bool isLoaded();

        /// \brief Reset instance state to the initial one
        /// @return true if the FMU was reset correctly, false otherwise
        bool resetInstance(const double resetTimeInSeconds);

        /// \brief Simulate for a specified amount of time
        bool doStep(const double currentTimeInSeconds, const double stepTimeInSeconds);

        /// \brief Get input variables references
        bool getInputVariableRefs(const std::vector<std::string>& inputVariableNames,
                                  std::vector<fmi2_value_reference_t>& inputVariableReferences);

        /// \brief Get output variable references
        bool getOutputVariableRefs(const std::vector<std::string>& outputVariableNames,
                                  std::vector<fmi2_value_reference_t>& outputVariableReferences);

        /// \brief Set input variables
        bool setInputVariables(const std::vector<fmi2_value_reference_t>& inputVariableReferences,
                               const std::vector<double>& inputVariableS);

        /// \brief Get output variables
        bool getOutputVariables(const std::vector<fmi2_value_reference_t>& outputVariableReferences,
                                std::vector<double>& outputVariables);
        /// \brief Unload
        /// Unload the fmu, or do nothing if no fmu was loaded
        void unload();

        // TODO(traversaro): delete everything (Rule of 0)
    };
}

#endif
