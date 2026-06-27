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

// 當 blackboard 上存的是「字串」(例如 SetBlackboard value="-1;3")，
// 而某個 InputPort<Position2D> 要去讀它時，BT 會呼叫這個特化把字串 parse 成 struct。
// 注意：一定要放在 namespace BT 裡。
namespace BT
{
template <>
Position2D convertFromString(StringView str)
{
  // 預期格式 "x;y"
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

// 寫入端：把一個自訂 struct (Position2D) 寫進 blackboard。
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

// 讀取端：用 InputPort<Position2D> 把同一個 struct 從 blackboard 讀回來。
// 因為值是以 Position2D 型別整個存在 blackboard 上，這裡不需要 convertFromString。
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

// 直接繼承 ActionNodeBase：tick() 與 halt() 都必須自己實作。
// 這裡模擬一個「需要好幾個 tick 才完成」的動作，所以會回傳 RUNNING，
// 並用內部 counter_ 自己管理進度，halt() 負責被中斷時的清理。
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
      // OutputPort：宣告這個節點會「寫出」一個值到 blackboard
      // 注意：這裡的 "text" 是「port 名稱」，跟 blackboard key 無關。
      // XML 寫 text="{message}" → port「text」綁到 blackboard 的「message」key。
      BT::OutputPort<std::string>("text", "Message written to the blackboard")};
  }

  // 因為會回傳 RUNNING，必須自己實作 tick()
  BT::NodeStatus tick() override
  {
    // 第一次被 tick：初始化進度
    if (status() == BT::NodeStatus::IDLE) {
      counter_ = 0;
      std::cout << "DoSomething: start working..." << std::endl;
    }

    ++counter_;
    std::cout << "DoSomething: working... (" << counter_ << "/3)" << std::endl;

    // 還沒做完 → 回傳 RUNNING，框架下一輪會再 tick 進來
    if (counter_ < 3) {
      return BT::NodeStatus::RUNNING;
    }

    BT::Optional<std::string> answer = getInput<std::string>("answer");
    if (!answer) {
      throw BT::RuntimeError("missing required input [answer]: ", answer.error());
    }

    std::cout << "DoSomething: done, answer is \"" << answer.value() << "\"" << std::endl;

    // setOutput 的第一個參數是「port 名稱」，要跟 providedPorts() 宣告的一致 → "text"。
    // 值會經由 XML 的 text="{message}" 綁定，存進 blackboard 的 "message" key，
    // 之後 <SaySomething message="{message}"/> 就能讀到。
    setOutput("text", "The answer is " + answer.value());

    return BT::NodeStatus::SUCCESS;
  }

  // 因為是 RUNNING 節點，必須自己實作 halt()：被上層中斷時重置進度
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

  // tutorial 07：把子樹 GoalChecker 拆成獨立的 XML 檔，
  // 再用 <include> 從主樹拉進來。先把子樹寫到一個絕對路徑的檔案。
  // 注意：createTreeFromText 不會設定 include 的相對路徑基準，所以 include 要用絕對路徑。
  const std::string subtree_path = "/tmp/goal_checker_subtree.xml";
  {
    std::ofstream ofs(subtree_path);
    ofs << R"(
    <root>
      <!-- 子樹：內部只認得自己 blackboard 上的 "target"。 -->
      <BehaviorTree ID="GoalChecker">
        <Sequence>
          <SaySomething message="--- inside subtree GoalChecker (from included file) ---"/>
          <ShowGoal goal="{target}"/>
        </Sequence>
      </BehaviorTree>
    </root>
    )";
  }

  // 主樹：用 <include path="..."/> 把上面那個檔案的樹定義拉進來，
  // 之後 <SubTree ID="GoalChecker"> 就能解析到。
  static const char * xml_text = R"(
  <root main_tree_to_execute="MainTree">

    <include path="/tmp/goal_checker_subtree.xml"/>

    <BehaviorTree ID="MainTree">
      <Sequence name="root">
        <SetGoal goal="{the_goal}"/>
        <!-- classic <SubTree> 的 remap:值「不要」加大括號,直接寫父樹的 key 名稱。 -->
        <SubTree ID="GoalChecker" target="the_goal"/>
      </Sequence>
    </BehaviorTree>

  </root>
  )";

  auto tree = factory.createTreeFromText(xml_text);

  // DoSomething 會回傳 RUNNING，要用 tickRootWhileRunning 迴圈跑到完成
  tree.tickRootWhileRunning();

  rclcpp::shutdown();
  return 0;
}