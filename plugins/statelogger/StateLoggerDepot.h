/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef GAZEBO_FMI_STATELOGGERDEPOTPLUGIN_
#define GAZEBO_FMI_STATELOGGERDEPOTPLUGIN_

#include "stateLoggerInfo.h"
class StateLoggerDepot
{
    public:
        StateLoggerDepot();
        bool Add(const StateLoggerInfo& info);
        /*
        bool Start();
        bool Stop();
        */
};


#endif