// mppi::ParametersHandler on a plain node: which runtime sets reach MPPI's
// post callbacks (the Optimizer's re-apply / reset) and which are refused.
// Upstream ran the post callbacks for every set on the node, so a console
// change of another plugin's or the server's parameter reset the optimizer
// mid-goal.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_mppi_controller/tools/parameters_handler.hpp"

namespace
{
// getParam() is protected; the plugins reach it through getParamGetter().
class Fixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_parameters_handler");
    // Another plugin's and the server's parameters, declared as they are.
    node_->declare_parameter("FollowPathRPP3D.desired_linear_vel", 0.8);
    node_->declare_parameter("failure_tolerance", 3.0);
    handler_ = std::make_unique<mppi::ParametersHandler>(node_);
    auto getParam = handler_->getParamGetter("FollowPath");
    getParam(vx_max_, "vx_max", 0.5);
    getParam(model_dt_, "model_dt", 0.05, mppi::ParameterType::Static);
    getParam(critics_, "critics", std::string("CostCritic"), mppi::ParameterType::Static);
    handler_->addReadOnly("FollowPath.model_dt", "startup only");
    handler_->addPostCallback([this]() { post_calls_++; });
    handler_->start();
  }

  bool set(const rclcpp::Parameter & p) { return node_->set_parameter(p).successful; }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<mppi::ParametersHandler> handler_;
  double vx_max_{0.0};
  double model_dt_{0.0};
  std::string critics_;
  int post_calls_{0};
};
}  // namespace

TEST_F(Fixture, OwnDynamicParameterIsAppliedAndRunsThePostCallback)
{
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.vx_max", 0.6)));
  EXPECT_DOUBLE_EQ(vx_max_, 0.6);
  EXPECT_EQ(post_calls_, 1);
}

TEST_F(Fixture, AnotherPluginsOrTheServersParameterLeavesMppiAlone)
{
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPathRPP3D.desired_linear_vel", 0.5)));
  EXPECT_TRUE(set(rclcpp::Parameter("failure_tolerance", 2.0)));
  EXPECT_EQ(post_calls_, 0);
}

TEST_F(Fixture, ReadOnlyParameterIsRefusedAndUntouched)
{
  EXPECT_FALSE(set(rclcpp::Parameter("FollowPath.model_dt", 0.1)));
  EXPECT_DOUBLE_EQ(model_dt_, 0.05);
  EXPECT_DOUBLE_EQ(node_->get_parameter("FollowPath.model_dt").as_double(), 0.05);
  EXPECT_EQ(post_calls_, 0);
}

TEST_F(Fixture, StartupOnlyParameterIsAcceptedButNotApplied)
{
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.critics", std::string("GoalCritic"))));
  EXPECT_EQ(critics_, "CostCritic");
  EXPECT_EQ(post_calls_, 0);
}

TEST_F(Fixture, ARefusalInABatchAppliesNothing)
{
  const auto results = node_->set_parameters_atomically(
    {rclcpp::Parameter("FollowPath.vx_max", 0.7), rclcpp::Parameter("FollowPath.model_dt", 0.1)});
  EXPECT_FALSE(results.successful);
  EXPECT_DOUBLE_EQ(vx_max_, 0.5);
  EXPECT_EQ(post_calls_, 0);
}

TEST_F(Fixture, ASecondReaderOfAParameterFollowsItToo)
{
  // ConstraintCritic beside the Optimizer: getParam() binds a name once, the
  // second reader adds a callback of its own.
  double critic_copy = 0.5;
  handler_->addDynamicParamCallback(
    "FollowPath.vx_max",
    [&critic_copy](const rclcpp::Parameter & p) { critic_copy = p.as_double(); });
  EXPECT_TRUE(set(rclcpp::Parameter("FollowPath.vx_max", 0.65)));
  EXPECT_DOUBLE_EQ(vx_max_, 0.65);
  EXPECT_DOUBLE_EQ(critic_copy, 0.65);
  EXPECT_EQ(post_calls_, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
