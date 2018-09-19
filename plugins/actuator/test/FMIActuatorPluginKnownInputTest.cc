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

#include <gazebo_fmi/GazeboFMIUtils.hh>

#include <matio.h>


inline void writeVectorToMat(std::string name, std::vector< double > & vector, mat_t* mat)
{
    size_t nrOfSamples = vector.size();
    size_t dims[2] = {nrOfSamples,1};
    matvar_t *matvar = Mat_VarCreate(name.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, vector.data(), 0);
    Mat_VarWrite(mat, matvar, MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);
}

/**
 * @brief Test the effect of FMU transmission models, by applying a constant input to the transmission.
 *
 */
class FMIActuatorPluginKnownInputTest : public gazebo::ServerFixture,
                                                public testing::WithParamInterface<const char*>
{
  public: void PluginTest(const std::string &_physicsEngine);

  struct PluginTestHelperOptions
  {
  };

  public: gazebo::event::ConnectionPtr updateConnection;

  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const PluginTestHelperOptions& options);

  public: void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

  public: gazebo::physics::Joint* m_joint;
  public: double m_setForceBuffer;

};

void FMIActuatorPluginKnownInputTest::onUpdate(const gazebo::common::UpdateInfo & info)
{
    m_joint->SetForce(0, m_setForceBuffer);
}

void FMIActuatorPluginKnownInputTest::PluginTestHelper(const std::string &_physicsEngine,
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

  gzdbg << "FMIActuatorPluginKnownInputTest: testing world " << worldName << std::endl;

  // Get model
  auto model = world->ModelByName("pendulum_with_base");

  std::string jointName = "upper_joint";

  // Set zero gravity to simplify the analysis
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Get joint
  auto joint = model->GetJoint(jointName);
  std::string jointScopedName = joint->GetScopedName();

  // First test: constant input
  std::vector<double> inputActuator;
  std::vector<double> outputActuator;
  std::vector<double> jointPosition;
  std::vector<double> jointVelocity;
  std::vector<double> jointAcceleration;
  std::vector<double> simTime;

  // It is not obvious, but basically we cannot call SetForce outside the step,
  // or all the cumulative setForce logic gets skewed up due to the fact that
  // the logic in the step (including the actuator hijacking) and the setForce
  // happen at different timestamps. For this reason we actually call setForce
  // in a dedicated callback
  this->updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin (boost::bind ( &FMIActuatorPluginKnownInputTest::onUpdate, this, _1 ) );

  m_joint = joint.get();

  double input = 10;
  for(int i=0; i < 1000; i++)
  {
    // Set input
    double curr_inputActuator = input;
    inputActuator.push_back(curr_inputActuator);

    // The actual setForce is done in the onUpdate callback
    m_setForceBuffer = input;

    // Run simulation
    world->Step(1);

    // Save data
    double curr_outputActuator     = joint->GetForce(0u);
    double curr_jointPosition      = joint->Position(0u);
    double curr_jointVelocity      = joint->GetVelocity(0u);
    double curr_jointAcceleration  = gazebo_fmi::ComputeJointAcceleration(joint);
    double curr_simTime            = world->SimTime().Double();
    outputActuator.push_back(curr_outputActuator);
    jointPosition.push_back(curr_jointPosition);
    jointVelocity.push_back(curr_jointVelocity);
    jointAcceleration.push_back(curr_jointAcceleration);
    simTime.push_back(curr_simTime);
  }

  // Write the experiment data to a .mat file
  std::string output_mat = CMAKE_CURRENT_BINARY_DIR"/" + worldName + "_" + _physicsEngine +  "_constantInput.mat";
  mat_t *mat = Mat_Create(output_mat.c_str(), NULL);
  writeVectorToMat("inputActuator", inputActuator, mat);
  writeVectorToMat("outputActuator", outputActuator, mat);
  writeVectorToMat("jointPosition", jointPosition, mat);
  writeVectorToMat("jointVelocity", jointVelocity, mat);
  writeVectorToMat("jointAcceleration", jointAcceleration, mat);
  writeVectorToMat("simTime", simTime, mat);
  Mat_Close(mat);

  gzdbg << "FMIActuatorPluginKnownInputTest: saving sim result to " << output_mat << std::endl;

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void FMIActuatorPluginKnownInputTest::PluginTest(const std::string &_physicsEngine)
{
  // Defined by CMake
  std::string pluginDir = FMI_ACTUATOR_PLUGIN_BUILD_DIR;
  std::string fmuPath   = CMAKE_CURRENT_BINARY_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(fmuPath);

  PluginTestHelperOptions options;
  this->PluginTestHelper(_physicsEngine, "test_NoTransmission.world", options);
  this->PluginTestHelper(_physicsEngine, "test_NullTransmission.world", options);
  this->PluginTestHelper(_physicsEngine, "test_IdentityTransmission.world", options);
  this->PluginTestHelper(_physicsEngine, "test_DelayTransmission.world", options);

  // Uncommented to simulate using different worlds
  // this->PluginTestHelper(_physicsEngine, "test_StiffTransmission.world", options);
  // this->PluginTestHelper(_physicsEngine, "test_CompliantTransmission.world", options);
  // this->PluginTestHelper(_physicsEngine, "test_SoftTransmission.world", options);
}

/////////////////////////////////////////////////
TEST_P(FMIActuatorPluginKnownInputTest, PluginTest)
{
  PluginTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FMIActuatorPluginKnownInputTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
