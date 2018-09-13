# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

include(FMIUtils)

omc_compile_mo_to_fmu(INPUT_MO ${CMAKE_CURRENT_SOURCE_DIR}/LinearFriction.mo
                      MODEL_NAME LinearFriction
                      OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

omc_compile_mo_to_fmu(INPUT_MO ${CMAKE_CURRENT_SOURCE_DIR}/NullFriction.mo
                      MODEL_NAME NullFriction
                      OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_executable(FMISingleBodyFluidDynamicsPluginTest FMISingleBodyFluidDynamicsPluginTest.cc)
target_include_directories(FMISingleBodyFluidDynamicsPluginTest PUBLIC ${GAZEBO_INCLUDE_DIRS})
find_library(GAZEBO_TEST_LIB NAMES  gazebo_test_fixture HINTS ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(FMISingleBodyFluidDynamicsPluginTest PUBLIC ${GAZEBO_LIBRARIES} ${GAZEBO_TEST_LIB} gazebo_fmi_gtest)
target_compile_definitions(FMISingleBodyFluidDynamicsPluginTest PRIVATE -DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(FMISingleBodyFluidDynamicsPluginTest PRIVATE -DCMAKE_CURRENT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(FMISingleBodyFluidDynamicsPluginTest PRIVATE -DFMI_PLUGIN_BUILD_DIR="$<TARGET_FILE_DIR:FMISingleBodyFluidDynamicsPlugin>")

# The test depends on the FMU
# see https://samthursfield.wordpress.com/2015/11/21/cmake-dependencies-between-targets-and-files-and-custom-commands/
add_custom_target(generate-fmu-single-body-fluid-dynamics-test DEPENDS LinearFriction.fmu NullFriction.fmu)
add_dependencies(FMISingleBodyFluidDynamicsPluginTest generate-fmu-single-body-fluid-dynamics-test)
add_test(NAME FMISingleBodyFluidDynamicsPluginTest COMMAND FMISingleBodyFluidDynamicsPluginTest)
