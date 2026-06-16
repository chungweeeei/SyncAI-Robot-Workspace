// 「沒有 Navigator」的最小 BT action node 骨架。
//
// 它證明的事：真正在跑 BT 的引擎是 BtActionServer（內部用 BehaviorTreeEngine），
// Navigator / BtNavigator / Muxer / TF 全部都是可以拿掉的外殼。
//
// 架構（層級 1）：
//   [ 外部 client ] --(askmodel action goal)--> AskModelBtRunner
//        AskModelBtRunner 持有一個 BtActionServer<Askmodel>
//          - goal 進來 -> onGoalReceived()：把 question 寫進 blackboard
//          - 每個 tick -> onLoop()：發 feedback（THINKING…）
//          - 樹跑完  -> onCompletion()：從 blackboard 撈 answer 填進 result
//        樹「裡面」的葉節點（例如 <AskModel> action client、<Printout> service client）
//        才是去呼叫別人的 client，跟這個 server 是兩回事。
//
// 跑法：
//   ros2 run syncai_bt_navigator askmodel_bt_runner
//   ros2 action send_goal /askmodel syncai_test_bt_action_msgs/action/Askmodel "{question: 'hi'}"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_action_server.hpp"
#include "syncai_test_bt_action_msgs/action/askmodel.hpp"

namespace syncai_bt_navigator
{

class AskModelBtRunner
{
public:
  using ActionT = syncai_test_bt_action_msgs::action::Askmodel;

  explicit AskModelBtRunner(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    logger_ = node->get_logger();

    // 1. 樹裡面會用到的 BT plugin（.so 名稱）。XML 裡每個自訂 tag 都要有對應的 plugin
    //    被載入、註冊進 factory，否則 loadBehaviorTree 會丟例外。
    //    TODO: 等你寫好對應的 BT 葉節點後，把它們的 library 名稱填進來，例如：
    //      "syncai_askmodel_action_bt_node"   // BT::RosActionNode<Askmodel>，去呼叫 /askmodel_backend
    //      "syncai_printout_service_bt_node"  // BT::RosServiceNode<Printout>，去呼叫 /printout
    const std::vector<std::string> plugin_libs = {
      "syncai_askmodel_action_bt_node",
      "syncai_printout_service_bt_node",
    };

    // 2. default BT XML：放在 share/<pkg>/behavior_trees/askmodel.xml
    const std::string default_bt_xml =
      ament_index_cpp::get_package_share_directory("syncai_bt_navigator") +
      "/behavior_trees/askmodel.xml";

    // 3. 直接 new 一個 BtActionServer<Askmodel>。注意它完全沒有依賴 Navigator——
    //    是 Navigator 依賴它，不是反過來。constructor 會：建 action server + 載入
    //    default BT + activate，失敗會丟例外。
    bt_action_server_ = std::make_unique<syncai_behavior_tree::BtActionServer<ActionT>>(
      node, "askmodel", plugin_libs, default_bt_xml,
      std::bind(&AskModelBtRunner::onGoalReceived, this, std::placeholders::_1),
      std::bind(&AskModelBtRunner::onLoop, this),
      std::bind(&AskModelBtRunner::onPreempt, this, std::placeholders::_1),
      std::bind(
        &AskModelBtRunner::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger_, "AskModelBtRunner ready, action server: /askmodel");
  }

private:
  // goal 一進來就被呼叫一次。回傳 false 會直接拒絕 goal。
  // 主要工作：把 goal 的內容寫進 blackboard，讓樹裡的葉節點讀得到。
  bool onGoalReceived(ActionT::Goal::ConstSharedPtr goal)
  {
    RCLCPP_INFO(logger_, "Received question: \"%s\"", goal->question.c_str());

    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::string>("question", goal->question);  // 樹裡用 {question} 取用

    // TODO: 若之後 goal 想帶不同的 BT 檔名，可在這裡呼叫
    //       bt_action_server_->loadBehaviorTree(goal->some_bt_field);
    return true;
  }

  // bt_->run() 的 tick loop 裡每 tick 一次被呼叫，直到樹跑完。
  // 典型用途：發 action feedback。
  void onLoop()
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->state = ActionT::Feedback::THINKING;
    bt_action_server_->publishFeedback(feedback);
  }

  // 有新的 goal 搶佔當前 goal 時被呼叫。骨架先簡單接受 pending goal。
  void onPreempt(ActionT::Goal::ConstSharedPtr /*goal*/)
  {
    RCLCPP_INFO(logger_, "Preemption requested, accepting pending goal");
    auto pending = bt_action_server_->acceptPendingGoal();
    bt_action_server_->getBlackboard()->set<std::string>("question", pending->question);
  }

  // 樹跑完（SUCCEEDED / FAILED / CANCELED）後被呼叫。
  // 工作：把樹寫到 blackboard 的結果，搬進 action result。
  void onCompletion(
    ActionT::Result::SharedPtr result, const syncai_behavior_tree::BtStatus final_bt_status)
  {
    auto blackboard = bt_action_server_->getBlackboard();

    // TODO: 換成你的葉節點實際寫回 blackboard 的 key
    std::string answer;
    blackboard->get("answer", answer);  // key 不存在時 answer 維持空字串

    result->answer = answer;
    result->success = (final_bt_status == syncai_behavior_tree::BtStatus::SUCCEEDED);

    RCLCPP_INFO(
      logger_, "Tree finished, success=%s answer=\"%s\"", result->success ? "true" : "false",
      result->answer.c_str());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AskModelBtRunner")};
  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
};

}  // namespace syncai_bt_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // BtActionServer 需要一個已存在的 node SharedPtr（它內部存 WeakPtr，所以 node 要活著）。
  auto node = std::make_shared<rclcpp::Node>("askmodel_bt_runner");
  auto runner = std::make_shared<syncai_bt_navigator::AskModelBtRunner>(node);

  // spin 主 node 才能收到 action goal；BT 葉節點的 client_node 由葉節點自己 spin。
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
