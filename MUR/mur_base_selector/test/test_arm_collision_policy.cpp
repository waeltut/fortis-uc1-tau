#include "arm_collision_policy.hpp"
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

class ArmCollisionPolicy : public ::testing::Test {
protected:
  moveit::core::RobotModelPtr model;
  std::unique_ptr<planning_scene::PlanningScene> scene;
  std::array<std::vector<std::string>,2> branches;
  void SetUp() override {
    const std::string box="<collision><geometry><box size='0.2 0.2 0.2'/></geometry></collision>";
    const std::string urdf="<robot name='test'>"
      "<link name='body'>"+box+"</link>"
      "<link name='mount'>"+box+"</link>"
      "<joint name='mount_fixed' type='fixed'><parent link='body'/><child link='mount'/><origin xyz='-2 0 0'/></joint>"
      "<link name='left'>"+box+"</link>"
      "<joint name='left_slide' type='prismatic'><parent link='body'/><child link='left'/><axis xyz='1 0 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
      "<link name='elbow'>"+box+"</link>"
      "<joint name='elbow_slide' type='prismatic'><parent link='left'/><child link='elbow'/><axis xyz='1 0 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
      "<link name='left_tip'/>"
      "<joint name='tip_fixed' type='fixed'><parent link='elbow'/><child link='left_tip'/></joint>"
      "<link name='left_hand'>"+box+"</link>"
      "<joint name='hand_fixed' type='fixed'><parent link='left_tip'/><child link='left_hand'/><origin xyz='0.5 0 0'/></joint>"
      "<link name='right'>"+box+"</link>"
      "<joint name='right_slide' type='prismatic'><parent link='body'/><child link='right'/><axis xyz='1 0 0'/><limit lower='-3' upper='3' effort='1' velocity='1'/></joint>"
      "<link name='right_tool'>"+box+"</link>"
      "<joint name='tool_fixed' type='fixed'><parent link='right'/><child link='right_tool'/><origin xyz='0.5 0 0'/></joint>"
      "</robot>";
    auto description=urdf::parseURDF(urdf); ASSERT_TRUE(description);
    auto semantic=std::make_shared<srdf::Model>();
    ASSERT_TRUE(semantic->initString(*description,"<robot name='test'><group name='left_arm'><chain base_link='body' tip_link='left_tip'/></group><group name='right_arm'><chain base_link='body' tip_link='right'/></group></robot>"));
    model=std::make_shared<moveit::core::RobotModel>(description,semantic);
    scene=std::make_unique<planning_scene::PlanningScene>(model);
    branches={mr::arm_branch(*model,"left_arm"),mr::arm_branch(*model,"right_arm")};
  }
  bool collides(unsigned arm,double left,double elbow,double right) {
    moveit::core::RobotState state(model);state.setToDefaultValues();
    state.setVariablePosition("left_slide",left);state.setVariablePosition("elbow_slide",elbow);
    state.setVariablePosition("right_slide",right);state.update();
    auto acm=mr::arm_collision_matrix(*scene,branches[1-arm]);
    collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
    scene->checkSelfCollision(request,result,state,acm);return result.collision;
  }
};
TEST_F(ArmCollisionPolicy, BranchIncludesToolsButNotFixedStructure) {
  EXPECT_NE(std::find(branches[0].begin(),branches[0].end(),"left_hand"),branches[0].end());
  EXPECT_NE(std::find(branches[1].begin(),branches[1].end(),"right_tool"),branches[1].end());
  for(const auto& branch:branches) {
    EXPECT_EQ(std::find(branch.begin(),branch.end(),"body"),branch.end());
    EXPECT_EQ(std::find(branch.begin(),branch.end(),"mount"),branch.end());
  }
}
TEST_F(ArmCollisionPolicy, OppositePostureAndItsBodyCollisionsDoNotMatter) {
  for(double right:{0.,1.,1.5,2.,-2.}) EXPECT_FALSE(collides(0,1.,.5,right));
}
TEST_F(ArmCollisionPolicy, SelectedArmStillCollidesWithBodyAndMount) {
  EXPECT_TRUE(collides(0,0.,.5,2.5));
  EXPECT_TRUE(collides(0,-2.,.5,2.5));
}
TEST_F(ArmCollisionPolicy, SelectedArmSelfCollisionIsRejected) {
  EXPECT_TRUE(collides(0,1.,0.,2.5));
}
TEST_F(ArmCollisionPolicy, OwnToolCollisionIsRejectedOutsideIkGroup) {
  EXPECT_TRUE(collides(0,-1.,.5,2.5)); // hand at body origin
}
TEST_F(ArmCollisionPolicy, RightWorkerIgnoresLeftButKeepsBodyChecks) {
  EXPECT_FALSE(collides(1,0.,0.,1.));
  EXPECT_TRUE(collides(1,1.,.5,0.));
}
