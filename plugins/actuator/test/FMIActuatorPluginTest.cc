/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
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

class FMIActuatorPluginTest : public gazebo::ServerFixture,
                              public testing::WithParamInterface<const char*>
{
  public: void PluginTest(const std::string &_physicsEngine);

  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const double expectedFinalPosition);

};

void FMIActuatorPluginTest::PluginTestHelper(const std::string &_physicsEngine,
                                             const std::string &worldName,
                                             const double expectedFinalPosition)
{
  bool worldPaused = true;
  std::string worldAbsPath = CMAKE_CURRENT_SOURCE_DIR"/" + worldName;
  Load(worldAbsPath, worldPaused, _physicsEngine);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  gazebo::physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Get model
  auto model = world->ModelByName("pendulum_with_base");


  // Set the desired position
  auto jointController = model->GetJointController();

  // Set PID gains and target
  gazebo::common::PID pid(100.0, 0.1, 0.0,      // P I D gain
                          10000.0, -10000.0,    // integral min max
                          10000.0, -10000.0); // output min max

  std::string jointName = "upper_joint";

  // Assume gravity on z axis
  auto grav = world->Gravity();
  EXPECT_NEAR(grav[0], 0, 0.1);
  EXPECT_NEAR(grav[1], 0, 0.1);

  // Get joint
  auto joint = model->GetJoint(jointName);
  std::string jointScopedName = joint->GetScopedName();

  jointController->SetPositionPID(jointScopedName, pid);
  jointController->SetPositionTarget(jointScopedName, 0.0);

  for(int i=0; i < 3000; i++)
  {
    //gzdbg << "Calling world->Step for i " << i << std::endl;
    gzerr << "Calling step" << std::endl;
    world->Step(1);
    // Measured position
    //gzdbg << "Measured  position " << joint->Position(0u) << std::endl;
  }

  gzdbg << "Final  position " << joint->Position(0u) << std::endl;
  EXPECT_NEAR(joint->Position(0u), expectedFinalPosition, 0.1);
}

/////////////////////////////////////////////////////////////////////
void FMIActuatorPluginTest::PluginTest(const std::string &_physicsEngine)
{
  // Defined by CMake
  std::string pluginDir = FMI_ACTUATOR_PLUGIN_BUILD_DIR;
  std::string fmuPath   = CMAKE_CURRENT_BINARY_DIR;
  gzdbg << "Adding " << pluginDir << " to the GAZEBO_PLUGIN_PATH" << std::endl;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  gzdbg << "Adding " << fmuPath << " to the GAZEBO_RESOURCE_PATH" << std::endl;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(fmuPath);

  // With the null trasmission, the PID loop should not be effective
  this->PluginTestHelper(_physicsEngine, "test_NullTrasmission.world", -1.5708);

  // With the identity trasmission, the PID loop should work fine
  this->PluginTestHelper(_physicsEngine, "test_IdentityTrasmission.world", 0.0);
}

/////////////////////////////////////////////////
TEST_P(FMIActuatorPluginTest, PluginTest)
{
  PluginTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FMIActuatorPluginTest, ::testing::Values("ode"));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
