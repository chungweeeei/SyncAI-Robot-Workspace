# 三種 ActionNode：Sync / Stateful / Async

> 對應範例：
> - `examples/sync_action_demo.cpp` — `SyncActionNode`
> - `examples/stateful_action_demo.cpp` — `StatefulActionNode`
> - `examples/async_action_demo.cpp` — `AsyncActionNode`
>
> 原始碼：`src/third-party/behaviortree_cpp_v3/`（主要看 `include/.../action_node.h` 與 `src/action_node.cpp`）

---

## 0. 共同點

三者都是 **leaf node**（沒有 child、真正「做事」的人），都繼承自 `ActionNodeBase : LeafNode`。
差別只有一個：**動作要花多久、`tick()` 怎麼跑**。

```
TreeNode
└─ LeafNode
    └─ ActionNodeBase
        ├─ SyncActionNode       ← 瞬間完成
        ├─ StatefulActionNode   ← 長動作，tree thread 上分段查進度
        └─ AsyncActionNode      ← 長動作，框架另開 worker thread 跑
```

> 注意：`AsyncActionNode` 是 **action node（leaf）**，不是 control node。

---

## 1. 速查對照表

| | tick() 跑在哪 | tick() 能阻塞嗎 | 怎麼分段 / 查進度 | 取消方式 |
|---|---|---|---|---|
| **Sync** | tree thread | ❌ 不行（會卡死整棵樹） | 不需要，秒回 SUCCESS/FAILURE | 不適用（太快） |
| **Stateful** | tree thread | ❌ 不行 | 你寫 `onRunning()`，每 tick 回 RUNNING | `onHalted()` |
| **Async** | **框架另開的 worker thread** | ✅ **可以**（sleep / 迴圈都行） | 框架自動：worker 沒跑完就一直回 RUNNING | `tick()` 內輪詢 `isHaltRequested()` 主動跳出 |

核心一句話：
- **Sync / Stateful** 的 tick 在 tree thread 上，所以 **tickRoot 會等它跑完**，那些函式必須秒回。
- **Async** 的 tick 被搬到 worker thread，所以 **tickRoot 不等**，直接回 RUNNING 繼續迴圈。

---

## 2. SyncActionNode

只覆寫 `tick()`，**必須當下回 `SUCCESS` / `FAILURE`，不可回 `RUNNING`**
（`SyncActionNode::executeTick()` 若偵測到 RUNNING 會直接 `throw LogicError`，見 `action_node.cpp:54`）。

```cpp
class PrintMessage : public BT::SyncActionNode {
  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    std::cout << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;   // 當下完成
  }
};
```

適用：印 log、設 blackboard、做判斷等「瞬間完成」的工作。
**誤用警告**：在 `tick()` 裡 sleep / 等 IO 會卡死整棵樹（連 `cancelRequested` / `onLoop` 都動不了）。

---

## 3. StatefulActionNode

`tick()` 已由框架實作好（**不能 override**），它依 node 當前狀態自動分派到三個 callback：

| node 狀態 | 呼叫 | 時機 |
|-----------|------|------|
| IDLE | `onStart()` | 還沒開始，或上次跑完被 reset |
| RUNNING | `onRunning()` | 上一次 tick 回了 RUNNING |
| （被 halt） | `onHalted()` | 被父節點/engine 呼叫 `halt()` 時，用來清理 |

```cpp
class CountDown : public BT::StatefulActionNode {
  BT::NodeStatus onStart() override {       // 啟動（非阻塞），立刻回 RUNNING
    progress_ = 0; getInput("ticks", ticks_);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {     // 每 tick 查一次進度，秒回
    if (++progress_ >= ticks_) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { /* 清理 */ }
};
```

關鍵：`onStart` / `onRunning` 都跑在 **tree thread** 上，所以**必須秒回**——靠「每 tick 回 RUNNING、下一圈再進來」來分段，而不是在函式裡 sleep。
真正耗時的工作應發生在別處（另一個 process / ROS action server），這裡只負責「啟動」與「查進度」。

### 觀察到的行為（`stateful_action_demo`）

`CountDown ticks="5"` 會橫跨約 6 圈 `run()` 迴圈，每圈相隔 `loopTimeout`(100ms)，
Sequence 卡在它的 idx、不回頭重跑前面已 SUCCESS 的 child，直到它回 SUCCESS 才前進。

---

## 4. AsyncActionNode

框架把你的 `tick()` 丟到**另一條 thread** 跑，所以 `tick()` 裡**允許阻塞 / 跑迴圈 / sleep**。

```cpp
class HeavyWork : public BT::AsyncActionNode {
  BT::NodeStatus tick() override {          // 這個 tick() 在 worker thread 上跑
    for (int i = 1; i <= steps; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 阻塞 OK
      if (isHaltRequested()) return BT::NodeStatus::FAILURE;         // 必須自己輪詢取消
    }
    return BT::NodeStatus::SUCCESS;
  }
};
```

### 機制（`AsyncActionNode::executeTick`，`action_node.cpp:160`）

```cpp
NodeStatus AsyncActionNode::executeTick() {
  if (status() == NodeStatus::IDLE) {       // ← 只有「第一次」tick 才成立
    setStatus(NodeStatus::RUNNING);
    thread_handle_ = std::async(std::launch::async, [this]() {
        auto status = tick();               // 在 worker thread 跑你的 tick()
        if (!isHaltRequested()) setStatus(status);
        // 例外處理：把 worker thread 的例外存進 exptr_，等 tree thread 來 rethrow
        emitStateChanged();
    });
  }
  // 每次都走到這：若 exptr_ 有東西就 rethrow（把例外搬回 tree thread）
  return status();                          // ← 立刻回，不等 worker
}
```

重點拆解：

1. **`std::async(std::launch::async, ...)` 一執行就啟動**——`std::launch::async` 強制立刻開新 thread 跑 lambda，不是「先建立再 start」。
2. **只在 IDLE→RUNNING 開「一條」thread、開「一次」**。第 2 次之後的 tick，`if(IDLE)` 不成立，只 `return status()` 純讀狀態，**不再開 thread**。所以一個 Async node = 一條 worker thread、跑一次 `tick()`。
3. **`[this]` 捕獲**：lambda 要呼叫 `tick()` / `isHaltRequested()` / `setStatus()` 等成員，所以捕獲 `this`。
4. **被 halt 時故意不 `setStatus`**：此時由 `halt()` 掌控狀態，避免 race。
5. **跨 thread 的例外**：worker thread 的例外無法被 tree thread 的 try/catch 直接接到，所以用 `std::current_exception()` 存進 `exptr_`，下次 `executeTick()` 在 tree thread 上 `std::rethrow_exception()`，讓 `run()` 外層的 try/catch（`behavior_tree_engine.cpp:69`）接到。
6. **`thread_handle_` 必須存成成員**：若 `future` 被當臨時值丟掉，其解構子會阻塞等 thread 跑完，退化成同步。存起來才真背景跑；`halt()` 也靠它 `wait()`。

### halt 行為（`action_node.cpp` `AsyncActionNode::halt`）

```cpp
void AsyncActionNode::halt() {
  halt_requested_.store(true);              // 設 flag
  if (thread_handle_.valid()) thread_handle_.wait();  // 等 worker thread 結束
}
```

→ 所以 `tick()` **必須定期檢查 `isHaltRequested()` 主動跳出**，否則 `halt()` 會一直 `wait()` 到迴圈自然結束，取消就失靈。

### 觀察到的行為（`async_action_demo`）

worker thread 每 1000ms 走一步、tree thread 每 100ms 轉一圈，log 會看到兩條 thread 訊息**交錯**：

```
(tree thread) run loop #1
...
(tree thread) run loop #10
    [HeavyWork] (worker thread) 完成第 1/5 步
(tree thread) run loop #11
...
```

`tickRoot()` 完全不為 HeavyWork 停下來——tick 到 Async node 只是 `return status()`，馬上往下 `onLoop()` → sleep 100ms。

### 兩個 caveat（也是 v3 較少推薦 Async 的原因）

1. **取消要自己顧**：`tick()` 內必須輪詢 `isHaltRequested()`。
2. **thread 安全要自己顧**：worker 與 tree thread 並行，共享狀態 / blackboard 要加鎖避免 race。

---

## 5. 多條 worker thread 何時發生？

單一 Async node 只開**一條** worker thread。要「多條並行」得有**多個** Async node 同時 RUNNING——典型是放在 `Parallel` 底下，Parallel 一圈 tick 多個 child，各自開一條 worker thread。在 `Sequence` 裡一次只有一個 child RUNNING，所以只有一條。

---

## 6. 實務選擇

- 瞬間工作 → **Sync**
- 長動作、但能拆成「啟動 + 每 tick 查進度」→ **Stateful**（首選）
- 長動作、且工作本身是個會阻塞的單一函式、又不想拆 → **Async**（但要自己處理取消與 thread 安全）

> 真實的 ROS 長動作（導航、規劃、控制）幾乎都用 **`BtActionNode`**（基於 Stateful 的非阻塞 poll 模型），把耗時工作丟給 ROS action server 那個獨立 process，BT 端只做非阻塞輪詢——既不用自己管 thread，取消也乾淨。參見 `plugins/action/`（如 `ComputePathToPose`）。

---

## 7. 相關文件

- [behavior_tree_tick_notes.md](behavior_tree_tick_notes.md) — `tickRoot()` 如何從 root 一路 tick 到 leaf、Sequence 的 `current_child_idx_` 記憶點、`loopTimeout` 與 RUNNING 的關係
