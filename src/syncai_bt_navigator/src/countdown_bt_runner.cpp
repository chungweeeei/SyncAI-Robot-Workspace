// Minimal runner that exercises the Countdown BT node against test_action_server.
//
// 它走的就是 nav2 bt_navigator 的精簡版三步驟：
//   1. 用 BehaviorTreeEngine 載入 Countdown plugin (.so)，把 "Countdown" tag 註冊進 factory
//   2. 從 share 目錄讀 countdown.xml，配一塊放好必要 key 的 blackboard，建出 tree
//   3. run() —— 固定頻率 tick 整棵樹直到 SUCCESS/FAILURE
//
// 跑法：
//   terminal 1: ros2 run syncai_test_action test_action_server
//   terminal 2: ros2 run syncai_bt_navigator countdown_bt_runner

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/behavior_tree_engine.hpp"

using namespace std::chrono_literals;  // NOLINT

int main(int argc, char ** argv)
{
  // run() 內部用 rclcpp::ok() / WallRate，所以 rclcpp 必須先 init。
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("countdown_bt_runner");

  // 1. 載入 Countdown plugin（.so 名稱對到 syncai_behavior_tree 的 add_library 目標）。
  //    載入時 BT_REGISTER_NODES 會自動把 "Countdown" 註冊進 factory。
  const std::vector<std::string> plugin_libs = {"syncai_countdown_action_bt_node"};
  syncai_behavior_tree::BehaviorTreeEngine engine(plugin_libs);

  // 2. blackboard：BtActionNode 的 constructor 會直接讀這幾個 key，少一個就丟例外。
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 100ms);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 2000ms);

  // 從 install 後的 share 目錄找到 XML。
  const std::string xml_path =
    ament_index_cpp::get_package_share_directory("syncai_bt_navigator") +
    "/behavior_trees/countdown.xml";
  RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", xml_path.c_str());

  try {
    // createTreeFromFile 會實例化 <Countdown> node；此時 BtActionNode constructor 會
    // 嘗試連上 /countdown action server，連不到（wait_for_service_timeout）就丟例外。
    BT::Tree tree = engine.createTreeFromFile(xml_path, blackboard);

    int tick_count = 0;

    // 3. 以 100ms 為週期 tick 整棵樹。
    //    - onLoop:          每個 tick 後執行（這裡只計數）
    //    - cancelRequested: 每個 tick 前詢問是否取消（固定 false；Ctrl+C 會讓 rclcpp::ok()
    //                       變 false 而結束）
    syncai_behavior_tree::BtStatus result = engine.run(
      &tree, [&tick_count]() { ++tick_count; }, []() { return false; }, 100ms);

    switch (result) {
      case syncai_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Tree SUCCEEDED after %d ticks", tick_count);
        break;
      case syncai_behavior_tree::BtStatus::FAILED:
        RCLCPP_WARN(node->get_logger(), "Tree FAILED after %d ticks", tick_count);
        break;
      case syncai_behavior_tree::BtStatus::CANCELED:
        RCLCPP_WARN(node->get_logger(), "Tree CANCELED after %d ticks", tick_count);
        break;
    }

    // 讀回 Countdown 寫到 blackboard 的 result。
    bool success = false;
    std::string message;
    blackboard->get("countdown_success", success);
    blackboard->get("countdown_message", message);
    RCLCPP_INFO(
      node->get_logger(), "Result from blackboard -> success: %s, message: \"%s\"",
      success ? "true" : "false", message.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "BT runner failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
