/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <gazebo/physics/physics.hh>

#include <gazebo/test/ServerFixture.hh>
#include <gazebo/test/helper_physics_generator.hh>

#include <gazebo_fmi/GazeboFMIUtils.hh>

class GazeboFMIUtilsComputeJointAccelerationTest : public gazebo::ServerFixture,
                           public testing::WithParamInterface<const char*>
{
  public: void PluginTest(const std::string &_physicsEngine);

  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName);

};

void GazeboFMIUtilsComputeJointAccelerationTest::PluginTestHelper(const std::string &_physicsEngine,
                                             const std::string &worldName)
{
  bool worldPaused = true;
  std::string worldAbsPath = CMAKE_CURRENT_SOURCE_DIR"/" + worldName;
  Load(worldAbsPath, worldPaused, _physicsEngine);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gzdbg << "GazeboFMIUtilsComputeJointAccelerationTest: testing world " << worldName << std::endl;

  // Get model
#if GAZEBO_MAJOR_VERSION >=8
  auto model = world->ModelByName("pendulum_attached_to_world");
#else
  auto model = world->GetModel("pendulum_attached_to_world");
#endif

  // Run the world for a few steps
  for(int i=0; i < 10; i++)
  {
    world->Step(1);
  }

  auto joint = model->GetJoint("upper_joint");
  double finalJointAcceleration = gazebo_fmi::ComputeJointAcceleration(joint);

  gzdbg << "Final  acceleration " << finalJointAcceleration << std::endl;
  double tol = 0.1;
  EXPECT_NEAR(finalJointAcceleration, 0.0, tol);

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void GazeboFMIUtilsComputeJointAccelerationTest::PluginTest(const std::string &_physicsEngine)
{
  // We check that the acceleration estimation does not crashes if the "world" link is either parent or child
  this->PluginTestHelper(_physicsEngine, "test_ComputeJointAcceleration_WorldAsParent.world");

  // Simbody does not support for the world to be a child link
  if (_physicsEngine != "simbody")
  {
    this->PluginTestHelper(_physicsEngine, "test_ComputeJointAcceleration_WorldAsChild.world");
  }
}

/////////////////////////////////////////////////
TEST_P(GazeboFMIUtilsComputeJointAccelerationTest, PluginTest)
{
  PluginTest(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, GazeboFMIUtilsComputeJointAccelerationTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
