

# Unfortunatly, the Gazebo-provided ServerFixture hardcode
# the inclusion of gtest headers, so it is necessary to vendor
# exactly the same gtest version used by Gazebo
#
# see https://bitbucket.org/osrf/gazebo/pull-requests/1606
add_library(gazebo_fmi_gtest STATIC gtest/src/gtest-all.cc)
target_include_directories(gazebo_fmi_gtest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/gtest ${CMAKE_CURRENT_SOURCE_DIR}/gtest/include)
