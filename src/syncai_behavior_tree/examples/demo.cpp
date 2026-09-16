#include <fstream>
#include <iostream>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message", "Message to say")};
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("message");

    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

struct Position2D
{
  double x;
  double y;
};

// When the blackboard holds a string (e.g. SetBlackboard value="-1;3") and some
// InputPort<Position2D> wants to read it, BT calls this specialisation to parse the string into
// the struct.
// Note: it must be placed inside namespace BT.
namespace BT
{
template <>
Position2D convertFromString(StringView str)
{
  // Expected format "x;y"
  const auto parts = splitString(str, ';');
  if (parts.size() != 2) {
    throw RuntimeError("invalid input for Position2D, expected \"x;y\"");
  }

  Position2D out;
  out.x = convertFromString<double>(parts[0]);
  out.y = convertFromString<double>(parts[1]);
  return out;
}
}  // namespace BT

// Writer side: write a custom struct (Position2D) into the blackboard.
class SetGoal : public BT::SyncActionNode
{
public:
  SetGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<Position2D>("goal", "Print goal")};
  }

  BT::NodeStatus tick() override
  {
    Position2D goal = {1.1, 2.3};
    setOutput<Position2D>("goal", goal);
    std::cout << "SetGoal: wrote goal {" << goal.x << ", " << goal.y << "}" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// Reader side: read the same struct back from the blackboard through InputPort<Position2D>.
// The value is stored on the blackboard as a whole Position2D, so no convertFromString is needed.
class ShowGoal : public BT::SyncActionNode
{
public:
  ShowGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Position2D>("goal", "Goal read from the blackboard")};
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<Position2D> goal = getInput<Position2D>("goal");
    if (!goal) {
      throw BT::RuntimeError("missing required input [goal]: ", goal.error());
    }

    std::cout << "ShowGoal: read goal {" << goal->x << ", " << goal->y << "}" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// Derives directly from ActionNodeBase: both tick() and halt() must be implemented by hand.
// This simulates an action that needs several ticks to finish, so it returns RUNNING, tracks
// its own progress in counter_, and halt() does the cleanup when it is interrupted.
class DoSomething : public BT::ActionNodeBase
{
public:
  DoSomething(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("answer", "Answer to the question"),
      // OutputPort: declares that this node writes a value out to the blackboard.
      // Note: "text" here is the port name, unrelated to the blackboard key.
      // Writing text="{message}" in the XML binds port "text" to the blackboard key "message".
      BT::OutputPort<std::string>("text", "Message written to the blackboard")};
  }

  // Because it returns RUNNING, tick() has to be implemented by hand
  BT::NodeStatus tick() override
  {
    // First tick: initialise the progress
    if (status() == BT::NodeStatus::IDLE) {
      counter_ = 0;
      std::cout << "DoSomething: start working..." << std::endl;
    }

    ++counter_;
    std::cout << "DoSomething: working... (" << counter_ << "/3)" << std::endl;

    // Not done yet -> return RUNNING; the framework ticks us again next round
    if (counter_ < 3) {
      return BT::NodeStatus::RUNNING;
    }

    BT::Optional<std::string> answer = getInput<std::string>("answer");
    if (!answer) {
      throw BT::RuntimeError("missing required input [answer]: ", answer.error());
    }

    std::cout << "DoSomething: done, answer is \"" << answer.value() << "\"" << std::endl;

    // The first argument of setOutput is the port name and must match providedPorts() -> "text".
    // Through the XML binding text="{message}" the value lands in the blackboard key "message",
    // where <SaySomething message="{message}"/> can then read it.
    setOutput("text", "The answer is " + answer.value());

    return BT::NodeStatus::SUCCESS;
  }

  // A node that returns RUNNING must implement halt(): reset progress when interrupted from above
  void halt() override
  {
    std::cout << "DoSomething: halted, resetting progress." << std::endl;
    counter_ = 0;
  }

private:
  int counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<DoSomething>("DoSomething");
  factory.registerNodeType<SetGoal>("SetGoal");
  factory.registerNodeType<ShowGoal>("ShowGoal");

  // tutorial 07: split the GoalChecker subtree into its own XML file and pull it into the main
  // tree with <include>. First write the subtree to a file at an absolute path.
  // Note: createTreeFromText sets no base directory for relative includes, so the include must
  // use an absolute path.
  const std::string subtree_path = "/tmp/goal_checker_subtree.xml";
  {
    std::ofstream ofs(subtree_path);
    ofs << R"(
    <root>
      <!-- Subtree: internally it only knows "target" on its own blackboard. -->
      <BehaviorTree ID="GoalChecker">
        <Sequence>
          <SaySomething message="--- inside subtree GoalChecker (from included file) ---"/>
          <ShowGoal goal="{target}"/>
        </Sequence>
      </BehaviorTree>
    </root>
    )";
  }

  // Main tree: pull in the tree definition from the file above with <include path="..."/>, so
  // that <SubTree ID="GoalChecker"> can be resolved afterwards.
  static const char * xml_text = R"(
  <root main_tree_to_execute="MainTree">

    <include path="/tmp/goal_checker_subtree.xml"/>

    <BehaviorTree ID="MainTree">
      <Sequence name="root">
        <SetGoal goal="{the_goal}"/>
        <!-- classic <SubTree> remap: NO braces around the value, just the parent tree's key. -->
        <SubTree ID="GoalChecker" target="the_goal"/>
      </Sequence>
    </BehaviorTree>

  </root>
  )";

  auto tree = factory.createTreeFromText(xml_text);

  // DoSomething returns RUNNING, so loop with tickRootWhileRunning until the tree completes
  tree.tickRootWhileRunning();

  rclcpp::shutdown();
  return 0;
}