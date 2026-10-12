#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

TEST(MuJoCo, HeaderMatchesLibraryVersion)
{
  EXPECT_EQ(mj_version(), mjVERSION_HEADER);
}

TEST(H1Model, LoadsWithExpectedJointsAndActuators)
{
  char error[1000] = "";
  mjModel * model = mj_loadXML(H1_SCENE, nullptr, error, sizeof(error));
  ASSERT_NE(model, nullptr) << error;

  EXPECT_EQ(model->nu, 19);
  EXPECT_EQ(model->njnt, 20);
  EXPECT_EQ(model->jnt_type[0], mjJNT_FREE);
  EXPECT_EQ(std::string(mj_id2name(model, mjOBJ_JOINT, 1)), "left_hip_yaw");
  EXPECT_EQ(std::string(mj_id2name(model, mjOBJ_JOINT, 2)), "left_hip_roll");
  EXPECT_DOUBLE_EQ(model->opt.timestep, 0.002);

  mj_deleteModel(model);
}