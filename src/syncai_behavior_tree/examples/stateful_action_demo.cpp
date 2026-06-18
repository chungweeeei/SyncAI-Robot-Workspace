// 對比範例：SyncActionNode vs StatefulActionNode。
//
// 一棵 Sequence 裡放三個 child：
//   <PrintMessage>  (Sync)     ── 瞬間完成，一個 tick 回 SUCCESS
//   <CountDown>     (Stateful) ── 長動作，要跑 N 個 tick 才完成，中間一直回 RUNNING
//   <PrintMessage>  (Sync)     ── 等 CountDown 完成後才會輪到它
//
// 觀察重點（對照 docs/behavior_tree_tick_notes.md 情境 B）：
//   1. SyncAction 在同一個 tickRoot() 裡秒回 SUCCESS；
//      StatefulAction 會跨「多圈」run() 迴圈，每圈被重 tick 一次。
//   2. Sequence 卡在 CountDown 那個 idx，不會回頭重跑前面已 SUCCESS 的 PrintMessage。
//   3. loopTimeout(100ms) 在這裡真的造成等待 —— 每次 onRunning() 之間相隔約 100ms。
//
// Run with:
//   ros2 run syncai_behavior_tree stateful_action_demo

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/behavior_tree_engine.hpp"

// ---------------------------------------------------------------------------
// SyncActionNode：瞬間完成（同前一個範例）
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
    return BT::NodeStatus::SUCCESS;  // Sync：當下就完成
  }
};

// ---------------------------------------------------------------------------
// StatefulActionNode：長動作，要跑 N 個 tick 才完成
// ---------------------------------------------------------------------------
// 重點：StatefulActionNode 的 tick() 已由框架實作好（不能 override），它會依
// node 當前狀態自動分派：
//   IDLE    → onStart()     (還沒開始、或上次跑完被 reset)
//   RUNNING → onRunning()   (上一次 tick 回了 RUNNING)
//   halt 時 → onHalted()    (被父節點/engine 呼叫 halt()，不是由 tick 觸發)
//
// 我們只覆寫這三個。注意：每個 onRunning() 都「秒回」—— 它不在 tick 裡 sleep，
// 而是回 RUNNING 把控制權交還給 run() 迴圈，下一圈再被叫進來查進度。
class CountDown : public BT::StatefulActionNode
{
public:
  CountDown(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    // InputPort(name, default, description)：XML 沒給 ticks 時用預設 5
    return {BT::InputPort<int>("ticks", 5, "需要幾個 tick 才完成")};
  }

  // 第一次被 tick（IDLE → 啟動工作）。這裡只做初始化，立刻回 RUNNING。
  BT::NodeStatus onStart() override
  {
    progress_ = 0;
    getInput("ticks", ticks_);
    std::cout << "[CountDown] onStart: 需要 " << ticks_ << " 個 tick\n";
    return BT::NodeStatus::RUNNING;  // 還沒完 → 交還控制權
  }

  // 之後每一圈 run() 迴圈都會叫一次（RUNNING → 查進度）。
  BT::NodeStatus onRunning() override
  {
    ++progress_;
    if (progress_ >= ticks_) {
      std::cout << "[CountDown] onRunning: " << progress_ << "/" << ticks_ << " → 完成\n";
      return BT::NodeStatus::SUCCESS;  // 做完了 → Sequence 才會往下一個 child
    }
    std::cout << "[CountDown] onRunning: " << progress_ << "/" << ticks_ << "\n";
    return BT::NodeStatus::RUNNING;  // 還沒完 → 下一圈再來
  }

  // 被 halt（例如 cancelRequested 回 true、或父節點失敗）時呼叫，用來清理。
  void onHalted() override { std::cout << "[CountDown] onHalted: 還沒完成就被中斷了\n"; }

private:
  int progress_{0};
  int ticks_{5};
};

// ---------------------------------------------------------------------------
// Engine subclass：把兩種 node 都註冊進 factory_
// ---------------------------------------------------------------------------
class DemoEngine : public syncai_behavior_tree::BehaviorTreeEngine
{
public:
  DemoEngine() : BehaviorTreeEngine({})
  {
    factory_.registerNodeType<PrintMessage>("PrintMessage");
    factory_.registerNodeType<CountDown>("CountDown");
  }
};

// ---------------------------------------------------------------------------
// XML：Sync → Stateful → Sync
// ---------------------------------------------------------------------------
static const char * kTreeXml = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <PrintMessage message="start"/>
      <CountDown ticks="5"/>
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

  // onLoop 印出迴圈圈數，讓你看到 CountDown 橫跨了好幾圈 run() 迴圈。
  // cancelRequested 先固定回 false；想看 onHalted()，把它改成
  //   [&loop_count]() { return loop_count >= 3; }
  // 就會在第 3 圈中斷 CountDown，觸發 onHalted()，整棵樹回 CANCELED。
  syncai_behavior_tree::BtStatus result = engine.run(
    &tree,
    [&loop_count]() {
      ++loop_count;
      std::cout << "  (run loop #" << loop_count << " 完成一次 tickRoot)\n";
    },                                           // onLoop：每次 tick 後執行
    [&loop_count]() { return loop_count > 2; },  // cancelRequested：永不取消
    std::chrono::milliseconds(100)               // loopTimeout: 每次 tick 的時間，這裡設 100ms
  );

  switch (result) {
    case syncai_behavior_tree::BtStatus::SUCCEEDED:
      std::cout << "tree finished: SUCCEEDED (共 " << loop_count << " 圈)\n";
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
