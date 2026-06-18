// 對比範例：AsyncActionNode（注意：它是 action node / leaf，不是 control node）。
//
// 跟 StatefulActionNode 的本質差異：
//   StatefulActionNode → 工作跑在「tree thread」上，你寫的 onRunning() 必須「秒回」，
//                        靠每 tick 回 RUNNING 來分段查進度。
//   AsyncActionNode    → 框架幫你「另開一條 background thread」跑你的 tick()，
//                        所以 tick() 裡【可以阻塞 / 跑迴圈 / sleep】而不會卡住整棵樹。
//
// 機制（見 third-party/.../src/action_node.cpp 的 AsyncActionNode::executeTick）：
//   第一次 tick(IDLE) → 設 RUNNING、用 std::async 開 thread 跑你的 tick()，馬上回 RUNNING；
//   之後每次 tick     → 只回傳目前 status；背景 thread 跑完才把 status 改成 SUCCESS；
//   halt()            → 設 halt flag 並 wait() 等背景 thread 結束，所以 tick() 內要
//                        定期檢查 isHaltRequested() 主動跳出。
//
// 觀察重點：log 會看到「worker thread 的進度」和「tree thread 的 run loop #」
//          兩條訊息【交錯出現】—— 這就是兩條 thread 同時在跑的證據。
//
// Run with:
//   ros2 run syncai_behavior_tree async_action_demo

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/behavior_tree_engine.hpp"

// ---------------------------------------------------------------------------
// SyncActionNode：瞬間完成（同前範例，當對照組）
// ---------------------------------------------------------------------------
class PrintMessage : public BT::SyncActionNode
{
public:
  PrintMessage(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message", "text to print")};
  }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg) {
      std::cout << "[PrintMessage] missing port: " << msg.error() << "\n";
      return BT::NodeStatus::FAILURE;
    }
    std::cout << "[PrintMessage] " << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// AsyncActionNode：tick() 在框架另開的 thread 上跑，因此「可以阻塞」
// ---------------------------------------------------------------------------
class HeavyWork : public BT::AsyncActionNode
{
public:
  HeavyWork(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("steps", 5, "要做幾步重工作")};
  }

  // 注意：這個 tick() 不是在 tree thread 上跑，而是在框架用 std::async 開的
  // background thread 上跑。所以這裡【刻意 sleep】來模擬耗時計算是 OK 的，
  // 不會卡住 run() 迴圈 —— 這正是 AsyncActionNode 跟 Sync/Stateful 的關鍵差別。
  BT::NodeStatus tick() override
  {
    int steps = 5;
    getInput("steps", steps);
    std::cout << "    [HeavyWork] (worker thread) 開始，共 " << steps << " 步\n";

    for (int i = 1; i <= steps; ++i) {
      // 模擬一步耗時工作（阻塞），但因為在自己的 thread 上，沒關係。
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // 支援取消：halt() 會把 isHaltRequested() 設成 true，這裡必須主動檢查並跳出，
      // 否則 halt() 會一直 wait() 等到迴圈自然結束。
      if (isHaltRequested()) {
        std::cout << "    [HeavyWork] (worker thread) 收到 halt，提前中止於第 " << i << " 步\n";
        return BT::NodeStatus::FAILURE;
      }

      std::cout << "    [HeavyWork] (worker thread) 完成第 " << i << "/" << steps << " 步\n";
    }

    std::cout << "    [HeavyWork] (worker thread) 全部完成\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// Engine subclass：註冊兩種 node
// ---------------------------------------------------------------------------
class DemoEngine : public syncai_behavior_tree::BehaviorTreeEngine
{
public:
  DemoEngine() : BehaviorTreeEngine({})
  {
    factory_.registerNodeType<PrintMessage>("PrintMessage");
    factory_.registerNodeType<HeavyWork>("HeavyWork");
  }
};

// ---------------------------------------------------------------------------
// XML：Sync → Async → Sync
// ---------------------------------------------------------------------------
static const char * kTreeXml = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <PrintMessage message="start"/>
      <HeavyWork steps="5"/>
      <PrintMessage message="done"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  DemoEngine engine;
  auto blackboard = BT::Blackboard::create();
  BT::Tree tree = engine.createTreeFromText(kTreeXml, blackboard);

  int loop_count = 0;

  // tree thread 在 HeavyWork 跑的期間【完全沒被卡住】：每 100ms 照常 tick 一圈、
  // 拿到 RUNNING、印一行 run loop #。你會看到它跟 worker thread 的進度交錯出現。
  syncai_behavior_tree::BtStatus result = engine.run(
    &tree,
    [&loop_count]() {
      ++loop_count;
      std::cout << "(tree thread) run loop #" << loop_count << "\n";
    },                              // onLoop：每次 tick 後執行
    []() { return false; },         // cancelRequested：永不取消
    std::chrono::milliseconds(100)  // loopTimeout: 每次 tick 的時間，這裡設 100ms
  );

  switch (result) {
    case syncai_behavior_tree::BtStatus::SUCCEEDED:
      std::cout << "tree finished: SUCCEEDED (tree thread 共轉了 " << loop_count << " 圈)\n";
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
