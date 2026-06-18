# BtServiceNode：包一個 ROS 2 service 的 BT 節點

> 原始碼：`include/syncai_behavior_tree/bt_service_node.hpp`
> 設計與 `BtActionNode`（`bt_action_node.hpp`，包 ROS action）同一套哲學。

---

## 1. 繼承關係：它是三兄弟的「第四個兄弟」

`BtServiceNode` 直接繼承 `BT::ActionNodeBase`，跟 Sync / Stateful / Async **同層**，不是站在它們之上或之下。

```
ActionNodeBase   (leaf action 的共同基底；只把 type() 釘成 ACTION，不實作 tick())
├─ SyncActionNode       ← 框架定好「tick 必須秒回」的合約
├─ StatefulActionNode   ← 框架把 tick() 拆成 onStart/onRunning/onHalted
├─ AsyncActionNode      ← 框架開 worker thread 跑 tick()
└─ BtServiceNode        ← 不靠上面任何鷹架，自己手寫 tick() + halt()
```

`ActionNodeBase` 本身幾乎是空的（`action_node.h:35`）：只宣告自己是 leaf、`type()` 回 `ACTION`，**不實作 `tick()` / `executeTick()`**。所以它是「最自由」的 action 基底——`BtServiceNode` 選它，正是為了能完全自訂 `tick()` 的結構（service 專屬的 hook 流程）。

---

## 2. Template Method：tick() 是骨架，業務邏輯放 hook

子類別**不該 override `tick()`**。`tick()` 是基底寫好的「不變流程」，子類別只覆寫「會變的細節」hook。

```cpp
BT::NodeStatus tick() override {            // 別動它
  if (!request_sent_) {
    should_send_request_ = true;
    on_tick();                              // ← hook 1：填 request 內容
    if (!should_send_request_) return FAILURE;
    future_result_ = service_client_->async_send_request(request_).share();  // 非阻塞送出
    sent_time_ = node_->now();
    request_sent_ = true;
  }
  return check_future();                     // 查回應（內部會呼叫 on_completion）
}
```

| Hook | 你在這裡做什麼 | 預設 |
|------|----------------|------|
| `on_tick()` | 從 port/blackboard 讀值填進 `request_`；或設 `should_send_request_ = false` 跳過 | 空 |
| `on_completion(response)` | 解析 response、寫回 blackboard，回傳最終 `NodeStatus` | 回 SUCCESS |
| `on_wait_for_result()` | 等待回應期間每次 timeout 時做的事（如發 feedback） | 空 |

`request_sent_` 旗標就是它的小型狀態機：`false` =「該送新 request」、`true` =「已送出、正在等」。取代了 `StatefulActionNode` 的 onStart/onRunning 分派。

---

## 3. constructor 做的事

```cpp
BtServiceNode(name, conf, service_name="")
: BT::ActionNodeBase(name, conf), ... {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");  // 從黑板拿共用 ROS node
  callback_group_ = node_->create_callback_group(MutuallyExclusive, false);  // 專屬 callback group
  callback_group_executor_.add_callback_group(callback_group_, ...);
  server_timeout_ = ... ;                  // 從黑板/port 取總時限
  max_timeout_ = bt_loop_duration * 0.5;   // 單次 tick 最多 spin 半個週期
  service_client_ = node_->create_client<ServiceT>(service_name_, ..., callback_group_);
  if (!service_client_->wait_for_service(...)) throw ...;  // server 不在就丟例外
}
```

- **`config()`**：繼承自 `TreeNode`，回傳建構時收到的 `NodeConfiguration`（內含共用 `blackboard` 與 port remapping）。`config().blackboard` 就是整棵樹共享的黑板。
- **`->template get<T>()`** 的 `template`：C++ 消歧義關鍵字（不是 blackboard 的東西），因為在 class template 裡呼叫 member template `Blackboard::get<T>`，告訴編譯器後面的 `<` 是模板參數列、不是小於號。
- **`BtServiceNode() = delete;`**：明確刪除無參數建構式——這個 node 一定要有 `NodeConfiguration`（黑板、ROS node）才能初始化，把無參數誤用擋在編譯期。
- **專屬 callback group + executor**：service client 綁在自己的 group，spin 時只處理「這個 node 的 service callback」，不干擾主 executor、避免重入。

---

## 4. check_future()：核心 poll 邏輯

兩個不同的 timeout 是理解關鍵：

| 變數 | 意義 |
|------|------|
| `server_timeout_` | 整個 service call 的**總時限**（送出到放棄） |
| `max_timeout_` = `bt_loop_duration * 0.5` | **單次 tick** 最多 spin 多久 |

```cpp
virtual BT::NodeStatus check_future() {
  auto elapsed   = now - sent_time_;
  auto remaining = server_timeout_ - elapsed;     // 總預算還剩多少

  if (remaining > 0ms) {
    auto timeout = min(remaining, max_timeout_);   // 這 tick 最多 spin 多久
    rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);

    if (rc == FutureReturnCode::SUCCESS) {          // future 完成（response 到了）
      request_sent_ = false;
      return on_completion(future_result_.get());   // 交給 hook 判定最終狀態
    }
    if (rc == FutureReturnCode::TIMEOUT) {          // 這次沒等到
      on_wait_for_result();
      if (now - sent_time_ < server_timeout_)
        return BT::NodeStatus::RUNNING;             // 總時限沒到 → 下一圈再來
    }
  }
  // 總時限耗盡 / 被中斷
  request_sent_ = false;
  return BT::NodeStatus::FAILURE;
}
```

### 三種出口

| 情況 | 條件 | 回傳 | `request_sent_` |
|------|------|------|------------------|
| 回應在這 tick 到了 | `rc == SUCCESS` | `on_completion()` 的結果 | reset false |
| 還沒到、總時限沒到 | `rc == TIMEOUT` 且 `elapsed < server_timeout_` | **RUNNING** | 維持 true（下 tick 續查） |
| 總時限耗盡 / 中斷 | 其餘 | **FAILURE** | reset false |

### 兩層 SUCCESS 不要混淆

- **`FutureReturnCode::SUCCESS`** = 「future 完成了（response 收到了）」，與內容好壞無關。
- **`NodeStatus::SUCCESS`** = 你在 `on_completion(response)` 看過內容後，判定「這次行為成功」。

---

## 5. spin_until_future_complete：要 spin，future 才會前進

`async_send_request` 回傳的 future **一開始不是 ready**。response 要靠 spin 才會被收進來：

```
service server 回 response
  → DDS/rmw 收到 bytes（放進底層 queue）
  → 【此時 future 仍 not-ready！】
  → spin_until_future_complete 驅動 executor 處理該事件
  → client callback 把值寫進 future_result_ → future 變 ready → 回 SUCCESS
```

所以這行有**雙重角色**：(1) pump——實際處理 ROS 事件；(2) wait——一邊處理一邊看 future 好了沒，最多等 `timeout`。**沒有 spin，response 即使到 rmw 層，future 也永遠不會完成。**

這跟 `AsyncActionNode`（有獨立 thread 自動推進）不同：service node 是**靠 tree thread 主動來 spin 才前進**——用 poll 手工模擬出一個 done-callback 的效果（`on_completion` 扮演完成後的處理函式，但由迴圈反覆 poll 驅動，不是事件推播）。

---

## 6. 延遲分析：response 完成後多久會被判定 SUCCESS？

因為 `spin_until_future_complete` 在每 tick 會**主動 spin-等待最多 `max_timeout_`**（≈半個週期），而非「瞬間取樣一下」，所以一個 tick cycle 分兩段（以 loop 100ms、max_timeout_ 50ms 為例）：

```
t=100ms ┬ tick → spin 最多 50ms  ← 這 50ms「正在 spin」，response 一到立刻抓到
t=150ms ┤ 沒到 → RUNNING
        │ onLoop + sleep          ← 這 50ms「沒人 spin」= 盲區
t=200ms ┴ 下一 tick → 再 spin 50ms ...
```

- response 落在 **spin 視窗**（如 101ms）→ 幾乎即時偵測，當下 tick 就回 SUCCESS（**不必等到 200ms**）。
- response 的底層到達落在 **盲區**（如 160ms）→ 沒人 spin、future 不前進，等到 200ms 下一次 spin 開頭才被抓到。

→ **最壞延遲 ≈ 一個盲區長度（約半個週期），不是一整圈，更不是跳過一輪。**

為什麼不整圈都 spin（消除盲區）？那會讓 tree thread 整圈卡在這個 node 上，`cancelRequested` / `onLoop` / 其他 node 全動不了。`max_timeout_ = 0.5 × bt_loop_duration` 是刻意留一半時間給迴圈喘息、檢查取消——用一點點延遲換迴圈的可回應性。

---

## 7. halt()

```cpp
void halt() override {
  request_sent_ = false;
  setStatus(BT::NodeStatus::IDLE);
}
```

被取消時重置狀態機並回 IDLE。注意它**沒有**主動取消已送出的 service request（service 不像 action 有 cancel 概念）——已在路上的 response 會被忽略，下次重新一輪。

---

## 8. 與三種 ActionNode 的定位對照

| | 等結果方式 | 會回 RUNNING | 開 thread | tick() 由誰寫 |
|---|---|---|---|---|
| Sync | 不等（秒回） | ❌ | ❌ | 你 |
| Stateful | 完全不阻塞，每 tick 查一下 | ✅ | ❌ | 框架（你寫 onRunning） |
| Async | tick 在 worker thread 阻塞 | ✅ | ✅ | 你（跑在 worker thread） |
| **BtServiceNode** | **tree thread 阻塞，但每 tick 上限半週期** | ✅ | ❌（用 executor spin） | 框架（你寫 on_tick/on_completion） |

`BtServiceNode` 在「完全不阻塞（Stateful）」與「整段阻塞別的 thread（Async）」之間取折衷：**在 tree thread 上做有時間上限的 spin**——因為它必須 spin 才能驅動 ROS callback，又不能卡死迴圈。

---

## 9. 相關文件

- [action_node_types.md](action_node_types.md) — Sync / Stateful / Async 三種 ActionNode
- [behavior_tree_tick_notes.md](behavior_tree_tick_notes.md) — tickRoot → executeTick → tick 的傳遞機制、RUNNING 與 loopTimeout
