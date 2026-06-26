# `syncai_task_runner` — Class / Object 架構

這份文件用來 trace `syncai_task_runner` 這個 node package 的 class 結構與物件關係，方便後續閱讀與擴充。

> TL;DR：`task_runner` 是一個**非 lifecycle** 的 `rclcpp::Node`，它本身不跑導航邏輯，而是「host」一群 **Navigator**。每個 Navigator 綁定一種 nav2 action（`NavigateToPose` / `NavigateThroughPoses`），內部各自持有一個 `BtActionServer`，由 Behavior Tree 真正執行導航。一個 `NavigatorMutex` 確保同一時間只有一個 Navigator 在驅動機器人。

---

## 1. 檔案地圖

```
syncai_task_runner/
├── include/syncai_task_runner/
│   ├── syncai_task_runner.hpp          # TaskRunner（host node）
│   ├── navigator.hpp                   # FeedbackUtils / NavigatorMutex / Navigator<ActionT>（base）
│   └── navigators/
│       ├── navigate_to_pose.hpp        # NavigateToPoseNavigator（衍生類別）
│       └── navigate_through_poses.hpp  # NavigateThroughPosesNavigator（衍生類別）
├── src/
│   ├── main.cpp                        # 進入點：建 node → initialize() → spin → cleanup()
│   ├── syncai_task_runner.cpp          # TaskRunner 實作
│   └── navigators/
│       ├── navigate_to_pose.cpp
│       └── navigate_through_poses.cpp
├── behavior_trees/
│   ├── move.xml                        # navigate_to_pose 預設 BT
│   └── patrol.xml                      # navigate_through_poses 預設 BT
└── params/
    └── task_runner_params.yaml         # frames / topics / BT loop 參數 / plugin 清單
```

外部相依（同 workspace 其他 package）：
- `syncai_behavior_tree::BtActionServer<ActionT>` — 把 action server + BT engine 包在一起的整合層。
- `syncai_util` — `OdomSmoother`、`getCurrentPose`、`geometry_utils`、`node_utils` 等工具。

---

## 2. Class 一覽

| Class / Struct | 定義位置 | 角色 |
|---|---|---|
| `TaskRunner` | `syncai_task_runner.hpp` | Host node，繼承 `rclcpp::Node`。擁有 TF / odom / mutex，註冊並初始化所有 navigator。 |
| `Navigator<ActionT>` | `navigator.hpp` | 所有 navigator 的 **template base class**。封裝「建 BtActionServer + mutex 互斥 + blackboard 初始化」的共通流程。 |
| `NavigateToPoseNavigator` | `navigators/navigate_to_pose.hpp` | 衍生類別，綁定 `nav2_msgs::action::NavigateToPose`（單一目標點）。 |
| `NavigateThroughPosesNavigator` | `navigators/navigate_through_poses.hpp` | 衍生類別，綁定 `nav2_msgs::action::NavigateThroughPoses`（多目標點 / 巡邏）。 |
| `NavigatorMutex` | `navigator.hpp` | 跨 navigator 的互斥鎖，保證同時只有一個 navigator 在導航。 |
| `FeedbackUtils` | `navigator.hpp` | 純資料 struct，把 TF/frame/tolerance 從 host node 傳給 navigator。 |
| `BtActionServer<ActionT>` | `syncai_behavior_tree`（外部） | 每個 navigator 內部持有，真正承載 action server + BT 執行。 |

---

## 3. 物件擁有關係（Ownership）

```
main()
 └─ shared_ptr<TaskRunner>                         (rclcpp::Node)
     ├─ NavigatorMutex            plugin_mutex_     (值；位址被傳給每個 navigator)
     ├─ shared_ptr<OdomSmoother>  odom_smoother_
     ├─ shared_ptr<tf2_ros::Buffer>            tf_
     ├─ shared_ptr<tf2_ros::TransformListener> tf_listener_
     ├─ unique_ptr<Navigator<NavigateToPose>>      pose_navigator_
     │    └─ (實體 = NavigateToPoseNavigator)
     │         └─ unique_ptr<BtActionServer<NavigateToPose>>  bt_action_server_
     │              ├─ SimpleActionServer<NavigateToPose>
     │              ├─ BehaviorTreeEngine
     │              └─ BT::Blackboard  (tf_buffer / odom_smoother / number_recoveries / goal / path ...)
     └─ unique_ptr<Navigator<NavigateThroughPoses>> poses_navigator_
          └─ (實體 = NavigateThroughPosesNavigator)
               └─ unique_ptr<BtActionServer<NavigateThroughPoses>> bt_action_server_
```

重點觀察：
- `TaskRunner` 以 `unique_ptr` 持有 navigator，但用 **base type** `Navigator<ActionT>` 的指標持有，靠 virtual function 呼到衍生類別（典型 polymorphism）。
- `plugin_mutex_` 是 `TaskRunner` 的**成員值**，初始化時把 `&plugin_mutex_`（裸指標）傳進每個 navigator，所以**兩個 navigator 共用同一把鎖**。
- navigator 對 host node 只持 `rclcpp::Node::WeakPtr`，避免循環參照。

---

## 4. Base class：`Navigator<ActionT>`（核心）

這是整個 package 最關鍵的設計，是一個 **CRTP 風格之外的 template + virtual 混合的 base class**。

### 4.1 為什麼用 template
每種 navigator 綁定不同的 action type（`NavigateToPose` vs `NavigateThroughPoses`），而 `BtActionServer` 也是 templated。用 `template <class ActionT>` 讓 base 一次寫好、衍生類別只要指定 `ActionT` 即可。

### 4.2 生命週期方法（非 virtual，框架流程）
| 方法 | 做什麼 |
|---|---|
| `on_initialize(...)` | 框架入口。lock parent node → 取 logger/clock → 存 `FeedbackUtils` 與 mutex 指標 → 取得預設 BT 路徑 → **建立 `BtActionServer`**（綁四個 callback）→ 初始化 blackboard（tf_buffer / initial_pose_received / number_recoveries / odom_smoother）→ 呼叫衍生類別的 `configure()`。建立 `BtActionServer` 用 try/catch 包住，失敗回 `false`。 |
| `on_cleanup()` | 呼叫衍生 `cleanup()`，再 `reset()` 掉 `bt_action_server_`。 |
| `getActionServer()` | 回傳內部 `BtActionServer` 的 unique_ptr 參考。 |

### 4.3 中介 callback（base 實作，**包含 mutex 邏輯**）
`BtActionServer` 收到的不是衍生類別的 callback，而是 base 的這兩個中介層，base 在這裡塞入互斥邏輯後再轉呼衍生類別：

```
onGoalReceived(goal)          ← 綁給 BtActionServer
   ├─ 若 plugin_mutex_->isNavigating() → 拒絕（return false）
   ├─ goal_accepted = goalReceived(goal)        // 純虛擬，衍生類別決定
   └─ 若 accepted → plugin_mutex_->startNavigating(getName())

onCompletion(result, status)  ← 綁給 BtActionServer
   ├─ plugin_mutex_->stopNavigating()
   └─ goalCompleted(result, status)             // 純虛擬
```

> 換句話說：**互斥的責任在 base，業務邏輯在衍生類別**。新增 navigator 時不必再處理鎖。

### 4.4 衍生類別必須實作的介面

| 介面 | 純虛擬? | 用途 |
|---|---|---|
| `getName()` | ✅ pure | navigator 名稱，同時是 action name。 |
| `getDefaultBTFilepath(node)` | ✅ pure | 回傳預設 BT XML 路徑。 |
| `goalReceived(goal)` | ✅ pure | 驗證 goal、載入 BT、寫入 blackboard。 |
| `goalCompleted(result, status)` | ✅ pure | 收尾（目前兩者皆空實作）。 |
| `onLoop()` | ✅ pure | 每個 BT tick 週期算 feedback 並 publish。 |
| `onPreempt(goal)` | ✅ pure | 處理 preemption（同一支 BT 才接受）。 |
| `configure(node, odom)` | virtual（預設回 true） | 註冊訂閱 / action client / 讀參數。 |
| `cleanup()` | virtual（預設回 true） | 釋放訂閱 / client。 |

### 4.5 受保護成員
```
bt_action_server_   unique_ptr<BtActionServer<ActionT>>   // 內部 action server + BT
logger_ / clock_                                          // 從 parent node 取得
feedback_utils_     FeedbackUtils                         // TF / frame / tolerance
plugin_mutex_       NavigatorMutex*                       // 指向 host node 共用鎖
```

---

## 5. 衍生類別差異對照

| 項目 | `NavigateToPoseNavigator` | `NavigateThroughPosesNavigator` |
|---|---|---|
| Action | `NavigateToPose`（單點） | `NavigateThroughPoses`（多點） |
| `getName()` | `"navigate_to_pose"` | `"navigate_through_poses"` |
| 預設 BT | `behavior_trees/move.xml` | `behavior_trees/patrol.xml` |
| 預設 BT 參數名 | `default_bt_xml` | `default_nav_through_poses_bt_xml` |
| Blackboard goal key | `goal_blackboard_id_`（預設 `"goal"`，單一 `PoseStamped`） | `goals_blackboard_id_`（預設 `"goals"`，`vector<PoseStamped>`） |
| 額外訂閱 | `goal_sub_`：訂 `goal_pose`（接 RViz 的單點目標） | 無 |
| 額外 client | `self_client_`：把 `goal_pose` 轉成 action goal 自送 | 無 |
| Feedback 額外欄位 | — | `number_of_poses_remaining` |

`onLoop()` 兩者邏輯幾乎相同：取目前位置 → 在 path 上找最近點 → 算剩餘距離 → 用 `OdomSmoother` 的速度估剩餘時間 → 填 `number_of_recoveries` / `current_pose` / `navigation_time` → publish feedback。差別只在 through_poses 多回報剩餘點數，且 goals 為空時直接 publish 空 feedback 返回。

---

## 6. 執行流程（Sequence）

### 6.1 啟動
```
main()
 1. rclcpp::init
 2. node = make_shared<TaskRunner>()        // ctor 只宣告參數，不做初始化
 3. node->initialize()                       // 必須在 shared_ptr 之後（用 shared_from_this()）
      ├─ 建 tf buffer + listener
      ├─ 讀 global_frame / base_frame / transform_tolerance / odom_topic / plugin_lib_names
      ├─ new NavigateToPoseNavigator / NavigateThroughPosesNavigator
      ├─ 填 FeedbackUtils
      ├─ 建 OdomSmoother
      └─ 對兩個 navigator 各呼 on_initialize(...)   // 內部建 BtActionServer + configure()
 4. rclcpp::spin(node)
 5. node->cleanup()                          // 反向釋放：先 listener 後 buffer，再 navigator
 6. rclcpp::shutdown
```

> ⚠️ 為什麼 `initialize()` 不放在 constructor：因為它呼叫 `this->shared_from_this()`，而物件必須先被 `shared_ptr` 持有後才能用。constructor 內 `shared_from_this()` 會丟例外。

### 6.2 一次導航（以 navigate_to_pose 為例）
```
外部 action client（或 RViz goal_pose → self_client_）送 NavigateToPose goal
        │
        ▼
BtActionServer 收到 goal
        │  呼 on_goal_received_callback_  =  Navigator::onGoalReceived
        ▼
Navigator::onGoalReceived
        ├─ mutex 已在導航？ → reject
        ├─ goalReceived(goal)  →  loadBehaviorTree(goal.behavior_tree)
        │                          initializeGoalPose(goal)  // 寫 goal 到 blackboard、重置 recoveries、start_time_
        └─ accepted → plugin_mutex_->startNavigating("navigate_to_pose")
        │
        ▼
BtActionServer 以 bt_loop_duration_（10ms = 100Hz）tick BT 樹
        │  每個週期呼 on_loop_callback_ = Navigator::onLoop → 算並 publish feedback
        │  BT 內節點（ComputePathToPose / FollowPath ...）讀寫 blackboard 的 goal/path
        ▼
BT 結束（SUCCEEDED / FAILED / CANCELED）
        │  呼 on_completion_callback_ = Navigator::onCompletion
        ▼
Navigator::onCompletion
        ├─ plugin_mutex_->stopNavigating()
        └─ goalCompleted(result, status)
```

`move.xml` 的樹（navigate_to_pose 預設）：
```
PipelineSequence "NavigateWithReplanning"
 ├─ RateController hz=1.0
 │    └─ ComputePathToPose  goal={goal} path={path} planner_id=GridBased
 └─ FollowPath  path={path} controller_id=FollowPath
```
`{goal}` / `{path}` 就是 blackboard key，對應 navigator 寫入的 `goal_blackboard_id_` / `path_blackboard_id_`。

---

## 7. Blackboard：navigator 與 BT 的溝通介面

Blackboard 是 navigator（C++）與 BT 節點（XML/plugin）之間唯一的資料通道。

| Key | 寫入者 | 內容 |
|---|---|---|
| `tf_buffer` | base `on_initialize` | 共用 TF buffer，給 BT 節點查座標。 |
| `odom_smoother` | base `on_initialize` | 速度估計。 |
| `initial_pose_received` | base `on_initialize` | bool 旗標。 |
| `number_recoveries` | base + `initializeGoalPose(s)` | recovery 次數，feedback 用。 |
| `goal` / `goals` | 衍生 `initializeGoalPose(s)` | 目標點，給 `ComputePathTo(Through)Poses` 讀。 |
| `path` | BT 節點（ComputePath...） | 規劃出的路徑，給 `FollowPath` 與 `onLoop()` 算剩餘距離。 |

---

## 8. 設計重點筆記（給後續維護）

1. **非 lifecycle 設計**：`TaskRunner` 直接繼承 `rclcpp::Node`，不是 `LifecycleNode`。初始化/清理改用自家的 `initialize()` / `cleanup()`，由 `main.cpp` 手動驅動（對應 workspace「de-lifecycled nav stack」的決定）。
2. **互斥集中在 base**：要加新 navigator，只要繼承 `Navigator<NewAction>`、實作純虛擬介面、在 `TaskRunner::initialize()` 多 `new` 一個並呼 `on_initialize()`，互斥/blackboard/action server 樣板全部免費繼承。
3. **參數兩處要同步**：`TaskRunner` ctor 內的 `plugin_libs` 預設清單，與 `params/task_runner_params.yaml` 的 `plugin_lib_names` 應保持一致（YAML 會覆蓋）。新增 BT 節點 plugin 時兩邊都要加。
4. **WeakPtr 防循環**：navigator 與 `BtActionServer` 對 parent node 都只持 `WeakPtr`；TF listener 要先於 buffer reset（見 `cleanup()`）。
5. **目前空實作**：兩個衍生類別的 `goalCompleted()` 都是空的，是預留的收尾掛勾。
