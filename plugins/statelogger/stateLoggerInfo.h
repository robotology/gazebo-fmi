/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef GAZEBO_FMI_STATELOGGERINFOPLUGIN_
#define GAZEBO_FMI_STATELOGGERINFOPLUGIN_

#include <gazebo_fmi/FMUCoSimulation.hh>    

class StateLoggerInfo
{
    public:
        StateLoggerInfo(const std::string& name, VariabileType type,const std::string& loggingFile,unsigned int samplingtime,const FMUCoSimulation& fmu):
            m_name(name),
            m_type(type),
            m_loggingFile(loggingFile),
            m_samplingtime(samplingtime),
            m_fmu(fmu)
        {};

        std::string m_name;
        VariabileType m_type;
        std::string m_loggingFile;
        unsigned int m_samplingtime{10};//In msec
        const FMUCoSimulation& fmu;
};

#endif