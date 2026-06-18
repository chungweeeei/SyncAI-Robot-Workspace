// 最小可跑範例：自己實作一個繼承 BT::SyncActionNode 的 behavior。
//
// SyncActionNode 是最簡單的 ActionNode 基底類別，專門給「在這一次 tick
// 就能完成」的同步動作用。唯一的限制是：
//   tick() 必須立刻回傳 SUCCESS 或 FAILURE，不允許回傳 RUNNING。
//
// 這個範例只做一件事：實作一個 PrintMessage node，從 port 讀一段文字並印出來。
//
// Run with:
//   ros2 run syncai_behavior_tree sync_action_demo

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/behavior_tree_engine.hpp"

// ---------------------------------------------------------------------------
// Step 1. 繼承 BT::SyncActionNode 實作一個 behavior
//         BT::SyncActionNode 是瞬間完成，一個 tick 回 SUCCESS / FAILURE (不可 RUNNING)
// ---------------------------------------------------------------------------
class PrintMessage : public BT::SyncActionNode
{
public:
  // 每個 BT node 的 constructor 都固定收這兩個參數 (name, config)，
  // 然後原封不動往上丟給基底類別。
  PrintMessage(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  // providedPorts() 向 factory 宣告「這個 node 有哪些輸入/輸出」。
  // 這裡宣告一個 input port "message"，型別是 std::string。
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message", "text to print")};
  }

  // SyncActionNode 只要覆寫 tick()。被 tick 時必須當下回傳 SUCCESS / FAILURE。
  BT::NodeStatus tick() override
  {
    // getInput() 會根據 providedPorts() 的宣告，從 XML 屬性或 blackboard 取值。
    auto msg = getInput<std::string>("message");
    if (!msg) {
      // 讀不到 port（例如 XML 漏寫 message=）就回 FAILURE。
      std::cout << "[PrintMessage] missing port: " << msg.error() << "\n";
      return BT::NodeStatus::FAILURE;
    }
    std::cout << "[PrintMessage] " << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// Step 2. 用一個 engine subclass 把 node 註冊進 factory_
// ---------------------------------------------------------------------------
// BehaviorTreeEngine::factory_ 是 protected，正式用法是透過 plugin .so 載入；
// 在這個自包含範例裡，我們 subclass engine 直接呼叫 registerNodeType 註冊。
class DemoEngine : public syncai_behavior_tree::BehaviorTreeEngine
{
public:
  DemoEngine() : BehaviorTreeEngine({})  // 空的 plugin list：不動態載入任何東西
  {
    factory_.registerNodeType<PrintMessage>("PrintMessage");
  }
};

// ---------------------------------------------------------------------------
// Step 3. 描述這棵樹的 XML
// ---------------------------------------------------------------------------
// message="..." 是靜態字串；message="{some_key}" 則會去 blackboard 讀 key。
static const char * kTreeXml = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <PrintMessage message="hello behavior tree 1"/>
      <PrintMessage message="hello behavior tree 2"/>
      <PrintMessage message="hello behavior tree 3"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

// ---------------------------------------------------------------------------
// Step 4. main()：建 blackboard → 建樹 → run()
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  // engine.run() 內部用到 rclcpp::ok() 與 rclcpp::WallRate，所以要先 init。
  rclcpp::init(argc, argv);

  // create DemoEngine instance
  DemoEngine engine;

  // blackboard 由整棵樹共用，這個最小範例其實還沒用到它。
  auto blackboard = BT::Blackboard::create();

  // create behavior tree instance from XML string
  BT::Tree tree = engine.createTreeFromText(kTreeXml, blackboard);

  // run() 固定頻率 tick 整棵樹，直到回 SUCCESS/FAILURE 或 cancelRequested 為 true。
  // run function 裡需要帶入的參數說明：
  // - onLoop: 每次 tick 後執行，這裡先預留空值
  // - cancelRequested: 這裡先寫永不取消
  // - loopTimeout: 每次 tick 的時間間隔，這裡設 100ms
  syncai_behavior_tree::BtStatus result = engine.run(
    &tree,                          // tree pointer
    []() {},                        // onLoop：每次 tick 後執行，這裡先留空
    []() { return false; },         // cancelRequested：永不取消
    std::chrono::milliseconds(100)  // loopTimeout: 每次 tick 的時間，這裡設 100ms
  );

  switch (result) {
    case syncai_behavior_tree::BtStatus::SUCCEEDED:
      std::cout << "tree finished: SUCCEEDED\n";
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
