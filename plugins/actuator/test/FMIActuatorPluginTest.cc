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

  struct PluginTestHelperOptions
  {
      double expectedFinalPosition{0.0};
      bool finalPositionIsReached{true};
  };

  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const PluginTestHelperOptions& options);

};

void FMIActuatorPluginTest::PluginTestHelper(const std::string &_physicsEngine,
                                             const std::string &worldName,
                                             const PluginTestHelperOptions& options)
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

  gzdbg << "FMIActuatorPluginTest: testing world " << worldName << std::endl;

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
    world->Step(1);
  }

  gzdbg << "Final  position " << joint->Position(0u) << std::endl;
  double tol = 0.1;
  if (options.finalPositionIsReached)
  {
    EXPECT_NEAR(joint->Position(0u), options.expectedFinalPosition, tol);
  }
  else
  {
    EXPECT_TRUE(std::abs(joint->Position(0u)-options.expectedFinalPosition) > tol);
  }

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void FMIActuatorPluginTest::PluginTest(const std::string &_physicsEngine)
{
  if(_physicsEngine == "simbody")
  {
      gzlog << "Aborting test: the FMIActuatorPluginTest is not workign correctly for Simbody, see https://github.com/robotology-playground/gazebo-fmi/issues/4 ." << std::endl;
      return;
  }

  // Defined by CMake
  std::string pluginDir = FMI_ACTUATOR_PLUGIN_BUILD_DIR;
  std::string fmuPath   = CMAKE_CURRENT_BINARY_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(fmuPath);

  // With the null trasmission, the PID loop should not be effective at all
  PluginTestHelperOptions options;
  options.expectedFinalPosition = -1.5708;
  options.finalPositionIsReached = true;
  this->PluginTestHelper(_physicsEngine, "test_NullTransmission.world", options);

  // With the identity trasmission, the PID loop should work fine
  options.expectedFinalPosition = 0.0;
  options.finalPositionIsReached = true;
  this->PluginTestHelper(_physicsEngine, "test_IdentityTransmission.world", options);

  // With the stiff trasmission, the PID loop should work fine
  options.expectedFinalPosition = 0.0;
  options.finalPositionIsReached = true;
  this->PluginTestHelper(_physicsEngine, "test_StiffTransmission.world", options);

  // With the compliant trasmission, the PID loop should work fine as well, given that
  // the spring is stiff enough
  options.expectedFinalPosition = 0.0;
  options.finalPositionIsReached = true;
  this->PluginTestHelper(_physicsEngine, "test_CompliantTransmission.world", options);

  // With the soft trasmission, the trasmission spring is too soft and the
  // PID is not effective
  options.expectedFinalPosition = 0.0;
  options.finalPositionIsReached = false;
  this->PluginTestHelper(_physicsEngine, "test_SoftTransmission.world", options);
}

/////////////////////////////////////////////////
TEST_P(FMIActuatorPluginTest, PluginTest)
{
  PluginTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FMIActuatorPluginTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
