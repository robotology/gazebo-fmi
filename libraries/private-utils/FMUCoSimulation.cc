/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <gazebo_fmi/FMUCoSimulation.hh>

#include <experimental/filesystem>

#include <fmilib.h>

#include <gazebo/common/Console.hh>

namespace gazebo_fmi
{

void GazeboFMI_importlogger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message)
{
    gzdbg << "gazebo_fmi : module = " << module << ", log level = "
          << jm_log_level_to_string(log_level) << ": " << message << std::endl;
}


class FMUCoSimulationPrivate
{
public:
    fmi2_callback_functions_t callBackFunctions;
    jm_callbacks callbacks;
    fmi_import_context_t* context{nullptr};
    fmi_version_enu_t version;
    fmi2_import_t* fmuHandle{nullptr};
    std::string instanceName;
    bool isLoaded{false};

    void cleanup()
    {
        if (fmuHandle)
        {
            fmi2_import_destroy_dllfmu(fmuHandle);
            fmi2_import_free(fmuHandle);
        }

        if (context)
        {
            fmi_import_free_context(context);
            context = nullptr;
        }

        isLoaded = false;
    }

    bool createInstance(const double startTime)
    {
        fmi2_status_t fmistatus;
        jm_status_enu_t jmstatus;

        jmstatus = fmi2_import_instantiate(fmuHandle, instanceName.c_str(), fmi2_cosimulation, NULL, fmi2_false);
        if (jmstatus == jm_status_error) {
            gzerr << "gazebo_fmi: fmi2_import_instantiate failed." << std::endl;
            this->cleanup();
            return false;
        }

        fmistatus = fmi2_import_setup_experiment(fmuHandle,
                                                fmi2_false, 0.0, startTime,
                                                fmi2_false, 0.0);

        if (fmistatus != fmi2_status_ok) {
            gzerr << "gazebo_fmi: fmi2_import_setup_experiment failed." << std::endl;
            this->cleanup();
            return false;
        }

        fmistatus = fmi2_import_enter_initialization_mode(fmuHandle);
        if (fmistatus != fmi2_status_ok) {
            gzerr << "gazebo_fmi: fmi2_import_enter_initialization_mode failed." << std::endl;
            this->cleanup();
            return false;
        }

        fmistatus = fmi2_import_exit_initialization_mode(fmuHandle);
        if (fmistatus != fmi2_status_ok) {
            gzerr << "gazebo_fmi: fmi2_import_exit_initialization_mode failed." << std::endl;
            this->cleanup();
            return false;
        }

        return true;
    }

    void deleteInstance()
    {
        // Close instance
        fmi2_import_terminate(this->fmuHandle);

        // Free instance
        fmi2_import_free_instance(this->fmuHandle);
    }
};

FMUCoSimulation::FMUCoSimulation(): m_pimpl(new FMUCoSimulationPrivate)
{
}

FMUCoSimulation::~FMUCoSimulation()
{
    this->unload();
}


bool FMUCoSimulation::load(const std::string& fmuAbsolutePath, const std::string& instanceName, const double startTimeInSeconds)
{
    // Check if an fmu is already loaded
    if (isLoaded())
    {
        gzerr << "gazebo_fmi: impossible to load FMU because an FMU was already loaded." << std::endl;
        return false;
    }

    // Check if file exists
    if (!std::experimental::filesystem::exists(fmuAbsolutePath))
    {
        gzerr << "gazebo_fmi: error impossible to find FMU at " << fmuAbsolutePath << std::endl;
        return false;
    }

    // Get temporary path
    std::string tmpPath = std::experimental::filesystem::temp_directory_path().string();

    // Setup callbacks
    m_pimpl->callbacks.malloc = malloc;
    m_pimpl->callbacks.calloc = calloc;
    m_pimpl->callbacks.realloc = realloc;
    m_pimpl->callbacks.free = free;
    m_pimpl->callbacks.logger = GazeboFMI_importlogger;
    m_pimpl->callbacks.log_level = jm_log_level_error;
    m_pimpl->callbacks.context = 0;

    // Allocate context
    m_pimpl->context = fmi_import_allocate_context(&(m_pimpl->callbacks));

    // Read version
    m_pimpl->version = fmi_import_get_fmi_version(m_pimpl->context, fmuAbsolutePath.c_str(), tmpPath.c_str());

    if (m_pimpl->version != fmi_version_2_0_enu) {
        gzerr << "gazebo_fmi: The code only supports FMI version 2.0." << std::endl;
        m_pimpl->cleanup();
        return false;
    }

    m_pimpl->fmuHandle = fmi2_import_parse_xml(m_pimpl->context, tmpPath.c_str(), 0);

    if (!m_pimpl->fmuHandle) {
        gzerr << "gazebo_fmi: Error parsing XML, exiting" << std::endl;
        m_pimpl->cleanup();
        return false;
    }

    if (fmi2_import_get_fmu_kind(m_pimpl->fmuHandle) == fmi2_fmu_kind_me) {
        gzerr << "gazebo_fmi:Only CS 2.0 is supported by this code." << std::endl;
        m_pimpl->cleanup();
        return false;
    }

    m_pimpl->callBackFunctions.logger = fmi2_log_forwarding;
    m_pimpl->callBackFunctions.allocateMemory = calloc;
    m_pimpl->callBackFunctions.freeMemory = free;
    m_pimpl->callBackFunctions.componentEnvironment = m_pimpl->fmuHandle;

    jm_status_enu_t status = fmi2_import_create_dllfmu(m_pimpl->fmuHandle, fmi2_fmu_kind_cs, &m_pimpl->callBackFunctions);
    if (status == jm_status_error) {
        gzerr << "gazebo_fmi: Could not create the DLL loading mechanism(C-API) (error: "
              << fmi2_import_get_last_error(m_pimpl->fmuHandle) << " )." << std::endl;
        m_pimpl->cleanup();
        return false;
    }

    // Create instance
    m_pimpl->instanceName = instanceName;
    bool createInstance = m_pimpl->createInstance(startTimeInSeconds);

    if (!createInstance)
    {
        return false;
    }

    m_pimpl->isLoaded = true;
    return true;
}

bool FMUCoSimulation::isLoaded()
{
    return m_pimpl->isLoaded;
}

bool FMUCoSimulation::resetInstance(const double resetTimeInSeconds)
{
    if (!isLoaded())
    {
        return false;
    }

    // Simple way of resetting the instance: destroy it and create a new one
    m_pimpl->deleteInstance();

    bool createInstance = m_pimpl->createInstance(resetTimeInSeconds);

    if (!createInstance)
    {
        return false;
    }

    return true;
}

bool FMUCoSimulation::doStep(const double currentTimeInSeconds, const double stepTimeInSeconds)
{
    if (!isLoaded()) {
        return false;
    }

    fmi2_status_t fmistatus = fmi2_import_do_step(m_pimpl->fmuHandle, currentTimeInSeconds, stepTimeInSeconds, fmi2_true);

    if (fmistatus != fmi2_status_ok) {
        gzerr << "gazebo_fmi: fmi2_import_do_step failed." << std::endl;
        return false;
    }

    return true;
}

bool FMUCoSimulation::getInputVariableRefs(const std::vector< std::string >& inputVariableNames,
                                           std::vector< fmi2_value_reference_t >& inputVariableReferences)
{
   inputVariableReferences.resize(inputVariableNames.size());

   for(size_t i=0; i < inputVariableNames.size(); i++)
   {
       fmi2_import_variable_t* var = fmi2_import_get_variable_by_name(m_pimpl->fmuHandle, inputVariableNames[i].c_str());
       if (!var) {
           gzerr << "gazebo_fmi: impossible to find variable of name \"" << inputVariableNames[i] << "\" in FMU." << std::endl;
           return false;
       }
       if (fmi2_causality_enu_input != fmi2_import_get_causality(var)) {
           gzerr << "gazebo_fmi: found variable of name " << inputVariableNames[i] << " in FMU, but causality is not input." << std::endl;
           return false;
       }
       if (fmi2_base_type_real != fmi2_import_get_variable_base_type(var)) {
           gzerr << "gazebo_fmi: found variable of name " << inputVariableNames[i] << " in FMU, but type is real." << std::endl;
           return false;
       }
       inputVariableReferences[i] = fmi2_import_get_variable_vr(var);
   }
   return true;
}

bool FMUCoSimulation::getOutputVariableRefs(const std::vector< std::string >& outputVariableNames,
                                            std::vector< fmi2_value_reference_t >& outputVariableReferences)
{
   outputVariableReferences.resize(outputVariableNames.size());

   for(size_t i=0; i < outputVariableReferences.size(); i++)
   {
       fmi2_import_variable_t* var = fmi2_import_get_variable_by_name(m_pimpl->fmuHandle, outputVariableNames[i].c_str());
       if (!var) {
           gzerr << "gazebo_fmi: impossible to find variable of name \"" << outputVariableNames[i] << "\" in FMU." << std::endl;
           return false;
       }
       if (fmi2_causality_enu_output != fmi2_import_get_causality(var)) {
           gzerr << "gazebo_fmi: found variable of name " << outputVariableNames[i] << " in FMU, but causality is not output." << std::endl;
           return false;
       }
       if (fmi2_base_type_real != fmi2_import_get_variable_base_type(var)) {
           gzerr << "gazebo_fmi: found variable of name " << outputVariableNames[i] << " in FMU, but type is real." << std::endl;
           return false;
       }
       outputVariableReferences[i] = fmi2_import_get_variable_vr(var);
   }
   return true;
}

bool FMUCoSimulation::getOutputVariables(const std::vector< fmi2_value_reference_t >& outputVariableReferences,
                                               std::vector< double >& outputVariables)
{
    outputVariables.resize(outputVariableReferences.size());

    fmi2_status_t fmistatus = fmi2_import_get_real(m_pimpl->fmuHandle, outputVariableReferences.data(),
                                                   outputVariables.size(), outputVariables.data());

    if (fmistatus != fmi2_status_ok) {
        gzerr << "gazebo_fmi: fmi2_import_get_real failed." << std::endl;
        return false;
    }

    return true;
}

bool FMUCoSimulation::setInputVariables(const std::vector< fmi2_value_reference_t >& inputVariableReferences,
                                        const std::vector< double >& inputVariables)
{
    if (inputVariableReferences.size() != inputVariables.size()) {
         gzerr << "gazebo_fmi: FMUCoSimulation::setInputVariables argument size mismatch." << std::endl;
        return false;
    }

    fmi2_status_t fmistatus = fmi2_import_set_real(m_pimpl->fmuHandle, inputVariableReferences.data(),
                                                   inputVariables.size(), inputVariables.data());

    if (fmistatus != fmi2_status_ok) {
        gzerr << "gazebo_fmi: fmi2_import_set_real failed." << std::endl;
        return false;
    }

    return true;
}



void FMUCoSimulation::unload()
{
    if (!this->isLoaded())
    {
        return;
    }

    m_pimpl->deleteInstance();

    // Free fmu
    fmi2_import_destroy_dllfmu(m_pimpl->fmuHandle);
    fmi2_import_free(m_pimpl->fmuHandle);
    fmi_import_free_context(m_pimpl->context);
    m_pimpl->fmuHandle = nullptr;
    m_pimpl->context = nullptr;
    m_pimpl->isLoaded = false;
}


}
