# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

add_executable(GazeboFMIUtilsComputeJointAccelerationTest GazeboFMIUtilsComputeJointAccelerationTest.cc)
target_include_directories(GazeboFMIUtilsComputeJointAccelerationTest PUBLIC ${GAZEBO_INCLUDE_DIRS})
find_library(GAZEBO_TEST_LIB NAMES gazebo_test_fixture HINTS ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(GazeboFMIUtilsComputeJointAccelerationTest PUBLIC ${GAZEBO_LIBRARIES} ${GAZEBO_TEST_LIB} GazeboFMIPrivateUtils gazebo_fmi_gtest)
target_compile_definitions(GazeboFMIUtilsComputeJointAccelerationTest PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME GazeboFMIUtilsComputeJointAccelerationTest COMMAND GazeboFMIUtilsComputeJointAccelerationTest)
