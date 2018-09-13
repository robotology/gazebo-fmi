# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
# 
# Licensed under either the GNU Lesser General Public License v3.0 : 
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

add_subdirectory(actuator)

if(gazebo_VERSION_MAJOR GREATER 7.0)
    add_subdirectory(single-body-fluid-dynamics)
endif()
