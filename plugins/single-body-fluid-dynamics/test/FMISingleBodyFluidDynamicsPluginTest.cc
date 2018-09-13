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

class FMISingleBodyFluidDynamicsPluginTest : public gazebo::ServerFixture,
                                             public testing::WithParamInterface<const char*>
{
  public: void PluginTest(const std::string &_physicsEngine);

  struct PluginTestHelperOptions
  {
      double expectedFinalPositionMinValue{0.0};
      double expectedFinalPositionMaxValue{0.0};
      int nrOfSteps{2000};
      double initialVel{1.0};
  };

  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const PluginTestHelperOptions& options);

};

void FMISingleBodyFluidDynamicsPluginTest::PluginTestHelper(const std::string &_physicsEngine,
                                                            const std::string &worldName,
                                                            const PluginTestHelperOptions& options)
{
  bool worldPaused = true;
  std::string worldAbsPath = CMAKE_CURRENT_SOURCE_DIR"/" + worldName;
  // Bullet is not compatible with FMISingleBodyFluidDynamicsPlugin
  // due to https://bitbucket.org/osrf/gazebo/issues/1476/implement-addxxxforce-for-bullet
  // Just  skip the test in that case
  if (_physicsEngine == "bullet") {
      return;
  }
  gzdbg << "FMISingleBodyFluidDynamicsPluginTest: testing world " << worldName << std::endl;
  Load(worldAbsPath, worldPaused, _physicsEngine);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  gazebo::physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);


  // Get model
  auto model = world->ModelByName("floating_cube");

  // Make sure that there is no gravity
  auto grav = world->Gravity();
  EXPECT_NEAR(grav[0], 0, 0.001);
  EXPECT_NEAR(grav[1], 0, 0.001);
  EXPECT_NEAR(grav[2], 0, 0.001);

  // Get link
  auto link = model->GetLink("link");

  // Set initial velocity of 1 m/s in the x directory
  ignition::math::Vector3d initialVel(options.initialVel, 0.0, 0.0);
  link->SetLinearVel(initialVel);

  // Run for two seconds
  for(int i=0; i < options.nrOfSteps; i++)
  {
    world->Step(1);
  }

  double posX = link->WorldPose().Pos()[0];
  EXPECT_GT(posX, options.expectedFinalPositionMinValue);
  EXPECT_LT(posX, options.expectedFinalPositionMaxValue);

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void FMISingleBodyFluidDynamicsPluginTest::PluginTest(const std::string &_physicsEngine)
{
  // Defined by CMake
  std::string pluginDir = FMI_PLUGIN_BUILD_DIR;
  std::string fmuPath   = CMAKE_CURRENT_BINARY_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(fmuPath);

  // With the null friction, the cube continues to go ahead and its velocity remains constant
  double timestep = 0.001;
  PluginTestHelperOptions options;
  double tol = timestep*options.initialVel;
  double finalPos = options.initialVel*options.nrOfSteps*timestep;
  options.expectedFinalPositionMinValue = finalPos-tol;
  options.expectedFinalPositionMaxValue = finalPos+tol;
  this->PluginTestHelper(_physicsEngine, "test_NullFriction.world", options);

  // With the linear friction, the cube eventually stops
  options.expectedFinalPositionMinValue = 0.096;
  options.expectedFinalPositionMaxValue = 0.1;
  this->PluginTestHelper(_physicsEngine, "test_LinearFriction.world", options);
}

/////////////////////////////////////////////////
TEST_P(FMISingleBodyFluidDynamicsPluginTest, PluginTest)
{
  PluginTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FMISingleBodyFluidDynamicsPluginTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
