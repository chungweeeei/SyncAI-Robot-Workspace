// Demo of syncai_behavior_tree::BehaviorTreeEngine.
//
// Shows the three steps every BT-based node (e.g. a future syncai_bt_navigator)
// goes through:
//   1. register node types with the factory
//   2. create a tree from an XML description + a shared blackboard
//   3. run() — tick the tree at a fixed rate until SUCCESS/FAILURE/cancel
//
// Run with:
//   ros2 run syncai_behavior_tree engine_demo
// or directly:
//   ./install/syncai_behavior_tree/lib/syncai_behavior_tree/engine_demo

#include <chrono>
#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/behavior_tree_engine.hpp"

class SaySomething : public BT::SyncActionNode
{
public:
  /**
   * BT::SyncActionNode 是 BehaviorTree.CPP 提供的最簡單的 ActionNode 基底類別，專門給「在此 tick 就能完成的任務」的同步動作用。
   * 核心 Constraint 只有一條：
   * tick() 必須立刻完成並回傳 SUCCESS 或 FAILURE，不允許回傳 RUNNING。
   * 
   * 在BehaviorTree.CPP v3 的 action 基底類別主要有三種，差別就在「動作要花多久」：
   * 1. SyncActionNode - 瞬間完成的動作                | 一次tick()直接回傳 SUCCESS/FAILURE
   * 2. StatefulActionNode - 長時間、需要多次tick的動作 | 拆成多個 step， onStart() / onRunning() / onHalted()，可回傳RUNNING
   * 3. AsyncActionNode - 自己的執行緒裡跑的動作        | tick() 在background thread 執行 (v3 中較少建議使用)
   */

  SaySomething(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  /**
   * providedPorts() 主要是Node向factory宣告「自己需要哪些輸入/輸出」。
   * 可以把它理解為這個節點的「介面規格表」
   */
  static BT::PortsList providedPorts()
  {
    return {/** 
       *  以 SaySomething 這個 SyncActionNode 為例
       *  <SaySomething message="leaving dock"/>  => 靜態字串 "leaving dock" 會直接傳給 message port
       *  <SaySomething message="{goal_name}"/>   => 從 blackboard 讀取 key "goal_name" 的值傳給 message
       */
            BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {
    // getInput() 是 BT::TreeNode 提供的 helper function，會根據 providedPorts() 的定義從 XML 或 blackboard 讀取輸入值。
    auto msg = getInput<std::string>("message");
    if (!msg) {
      std::cout << "[Say] missing port: " << msg.error() << "\n";
      return BT::NodeStatus::FAILURE;
    }
    std::cout << "[Say] " << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// A long-running node, like FollowPath in a real navigation tree: it keeps
// returning RUNNING, so the engine ticks it again on every loop until it
// finally reports SUCCESS. If the tree is cancelled mid-way the engine
// halts it and onHalted() runs.
class MoveToGoal : public BT::StatefulActionNode
{
public:
  MoveToGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    // There are three arguments to InputPort() helper function:
    // 1. port name (matches XML attribute)
    // 2. default value (optional; used if XML doesn't specify this port)
    // 3. description (optional; used for documentation)
    return {BT::InputPort<int>("num_ticks", 5, "ticks needed to reach the goal")};
  }

  /**
   * StatefulActionNode 的 tick() 已經在 BehaviorTree.CPP 實做好了（不能override它），內部會根據Node當前的State來分派呼叫。
   * 被tick時的狀態
   * IDLE => 呼叫 onStart() 也就是說當Node還沒開始過，或上次回傳SUCCESS/FAILURE後被reset
   * RUNNING => 呼叫 onRunning() │ 上一次 tick 回了 RUNNING
   * -       => onHalted()  │ 不是由 tick 觸發，而是父節點/engine 呼叫 halt()
   */
  BT::NodeStatus onStart() override
  {
    progress_ = 0;
    getInput("num_ticks", num_ticks_);
    std::cout << "[Move] start, needs " << num_ticks_ << " ticks\n";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override  // called once per engine tick
  {
    if (++progress_ >= num_ticks_) {
      std::cout << "[Move] arrived\n";
      return BT::NodeStatus::SUCCESS;
    }
    std::cout << "[Move] progress " << progress_ << "/" << num_ticks_ << "\n";
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { std::cout << "[Move] halted before reaching the goal!\n"; }

private:
  int progress_{0};
  int num_ticks_{5};
};

// The engine's factory_ is protected: nav2 normally fills it by loading
// plugin .so files passed to the constructor (each one self-registers via
// the BT_REGISTER_NODES macro). For a self-contained demo we subclass the
// engine and register our node types directly.
class DemoEngine : public syncai_behavior_tree::BehaviorTreeEngine
{
public:
  DemoEngine() : BehaviorTreeEngine({})  // empty plugin list: nothing loaded dynamically
  {
    // 直接透過 BT::BehaviorTreeFactory註冊一個C++ class
    factory_.registerNodeType<SaySomething>("SaySomething");
    factory_.registerNodeType<MoveToGoal>("MoveToGoal");

    /**
     * 其他兩種方式可以透過 BT::BehaviorTreeFactory註冊 node.
     * 當然也可以透過 lambda function 註冊簡單的Condition
     * factory_.registerSimpleCondition("IsBatteryOK", ...);
     * 從動態函式庫（.so）載入 
     * factory_.registerFromPlugin(loader.getOSName(p));
     */
  }
};

// The same tree a navigator would load from a .xml file with
// createTreeFromFile(). {goal_name} is a blackboard entry: the XML reads
// whatever main() stored under that key, the way nav2 passes `goal` and
// `path` between nodes.
static const char * kTreeXml = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <SaySomething message="leaving dock"/>
      <MoveToGoal num_ticks="5"/>
      <SaySomething message="{goal_name}"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char ** argv)
{
  // run() uses rclcpp::ok() and rclcpp::WallRate, so rclcpp must be up.
  rclcpp::init(argc, argv);

  DemoEngine engine;

  // 1. The blackboard is shared by every node in the tree. nav2 stores the
  //    ROS node handle, the TF buffer and the current goal here.
  auto blackboard = BT::Blackboard::create();
  blackboard->set<std::string>("goal_name", "reached goal: kitchen");

  // 2. Parse the XML and instantiate the tree.
  BT::Tree tree = engine.createTreeFromText(kTreeXml, blackboard);

  int tick_count = 0;

  // 3. Tick the root every 100 ms until the tree finishes.
  //    - onLoop:          runs after every tick (nav2 publishes action
  //                       feedback here)
  //    - cancelRequested: checked before every tick; returning true halts
  //                       the whole tree (nav2 checks the action server).
  //                       Try `return tick_count >= 3;` to see [Move] halt.
  // engine.run() 裡面分別有4個參數：
  // 1. &tree: 要執行的BT Tree
  // 2. onLoop: 每次tick後執行的callback function。
  // 3. cancelRequested: 每次tick前詢問「要取消嗎」
  // 4. loopTimeout: 每次tick的時間間隔
  syncai_behavior_tree::BtStatus result = engine.run(
    &tree,
    [&tick_count]() {
      ++tick_count;
      std::cout << "  (onLoop: tick " << tick_count << " done)\n";
    },
    []() { return false; }, std::chrono::milliseconds(100));

  switch (result) {
    case syncai_behavior_tree::BtStatus::SUCCEEDED:
      std::cout << "tree finished: SUCCEEDED after " << tick_count << " ticks\n";
      break;
    case syncai_behavior_tree::BtStatus::FAILED:
      std::cout << "tree finished: FAILED\n";
      break;
    case syncai_behavior_tree::BtStatus::CANCELED:
      std::cout << "tree finished: CANCELED\n";
      break;
  }

  rclcpp::shutdown();
  return 0;
}
