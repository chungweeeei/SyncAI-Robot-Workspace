# BehaviorTree.CPP Tick 機制筆記

> 以 `examples/sync_action_demo.cpp` 為例，整理「`tickRoot()` 如何從 root 一路 tick 到每個 leaf」的流程。
> 原始碼路徑：`src/third-party/behaviortree_cpp_v3/`

---

## 1. 三種 Action 基底類別

實作一個 behavior 時，依「動作要花多久完成」選基底類別：

| 基底類別 | 適用情境 | tick 行為 |
|----------|----------|-----------|
| `BT::SyncActionNode` | 瞬間完成的動作 | `tick()` 一次直接回 `SUCCESS`/`FAILURE`，**不允許回 `RUNNING`** |
| `BT::StatefulActionNode` | 長時間、需多次 tick 的動作 | 拆成 `onStart()` / `onRunning()` / `onHalted()`，可回 `RUNNING` |
| `BT::AsyncActionNode` | 在自己的 thread 跑的動作 | `tick()` 在背景 thread 執行（v3 較少建議使用） |

`sync_action_demo.cpp` 裡的 `PrintMessage` 就是 `SyncActionNode`：讀一個 `message` port、印出來、回 `SUCCESS`。

---

## 2. Engine 的 run() 迴圈

`BehaviorTreeEngine::run()`（`src/behavior_tree_engine.cpp:36`）固定頻率 tick 整棵樹：

```cpp
while (rclcpp::ok() && result == RUNNING) {
  if (cancelRequested()) { tree->rootNode()->halt(); return CANCELED; }
  result = tree->tickRoot();   // ← 主動 tick 整棵樹一次
  onLoop();
  loopRate.sleep();            // ← 睡到湊滿 loopTimeout（例 100ms）
}
```

- `loopTimeout`（例 100ms）是**整圈迴圈的週期**：每圈 `tickRoot()` 一次後睡到湊滿。
- 不是「被動確認狀態」，而是**主動 tick**——tick 本身就是在驅動 node 往前跑，不 tick 它就不動。
- 迴圈在 `tickRoot()` 回 `SUCCESS`/`FAILURE`（或 cancel）時結束。

---

## 3. Tick 如何從 root 往下傳遞

傳遞機制就是 **`executeTick()` ↔ `tick()` 的遞迴**，沒有神祕的走訪器。

### 三層呼叫鏈

```
Tree::tickRoot()                    ← engine 每圈呼叫
  └─ rootNode()->executeTick()      ← TreeNode::executeTick()，每個 node 共用的外殼
        └─ tick()                   ← virtual，實際型別決定跑哪個版本
```

### `tickRoot()`（`bt_factory.h:210`）

```cpp
NodeStatus ret = rootNode()->executeTick();
if (ret == SUCCESS || ret == FAILURE)
    rootNode()->setStatus(IDLE);     // 整棵跑完就 reset root 回 IDLE，方便重跑
return ret;
```

### `TreeNode::executeTick()`（`tree_node.cpp:32`）

每個 node 共用的外殼：pre-condition → `tick()` → post-condition → `setStatus()`。
它呼叫的 `tick()` 是 **virtual**，所以實際執行哪個版本由 node 真正型別決定：

- `<Sequence>` → `SequenceNode::tick()`
- `PrintMessage` → `PrintMessage::tick()`

### 往下傳遞 = control node 在自己的 tick() 裡再 tick child

「往下 tick」不是框架走訪，而是**每個 composite node 在自己的 `tick()` 裡主動 `executeTick()` 它的 child**。
child 若又是 composite 就再往下一層，遞迴成 DFS（深度優先）。

---

## 4. XML 結構與 rootNode

`sync_action_demo.cpp` 的 XML：

```xml
<root main_tree_to_execute="MainTree">   <!-- 容器，不是 node -->
  <BehaviorTree ID="MainTree">           <!-- 一棵具名樹的外框，不是 node -->
    <Sequence name="root">               <!-- 這才是真正的 rootNode -->
      <PrintMessage message="hello behavior tree 1"/>
      <PrintMessage message="hello behavior tree 2"/>
      <PrintMessage message="hello behavior tree 3"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- `<root>`、`<BehaviorTree>` **不是 BT node**，只是容器/metadata。
- 真正的 node 是 `<BehaviorTree>` 裡那唯一一個 child → 這裡是 `<Sequence>`。
- 所以 `tree.rootNode() == <Sequence>`，`tickRoot()` **從 Sequence 那層開始**。
- `<BehaviorTree>` 裡只能放一個 node；要放多個動作就用 composite（如 `<Sequence>`）包起來。
- 若 root 直接是一個 leaf（例 `<BehaviorTree>` 裡只放 `<PrintMessage/>`）也行，這時 `rootNode()` 就是那個 leaf，沒有 Sequence 那層。

---

## 5. SequenceNode::tick() 的細節

`src/controls/sequence_node.cpp:31`：

```cpp
NodeStatus SequenceNode::tick()
{
  const size_t children_count = children_nodes_.size();   // 1. 取 child 數量當迴圈邊界
  setStatus(NodeStatus::RUNNING);                          // 2. 先把自己設成 RUNNING

  while (current_child_idx_ < children_count)              // 3. 從「目前進度」往後依序 tick
  {
    TreeNode* child = children_nodes_[current_child_idx_];
    const NodeStatus child_status = child->executeTick();  // ← 對「這一個」child tick 剛好一次

    switch (child_status) {
      case RUNNING:  return RUNNING;                       // child 沒做完 → Seq 也回 RUNNING，idx 不動
      case FAILURE:  resetChildren(); current_child_idx_ = 0; return FAILURE;  // 整串失敗
      case SUCCESS:  current_child_idx_++; break;          // 成功才 idx++，while 繼續 → tick 下一個
      case IDLE:     throw LogicError("...");              // child 不該回 IDLE
    }
  }

  // 全部 child 都 SUCCESS
  resetChildren(); current_child_idx_ = 0;
  return SUCCESS;
}
```

### 三個關鍵觀念

1. **每個 child 一次只 tick「一下」。** 不是「把一個 child tick 到完才換下一個」。child 回什麼決定 while 要不要繼續：
   - `SUCCESS` → `idx++`、while 繼續 → **同一次** `Sequence::tick()` 裡馬上 tick 下一個
   - `RUNNING` → 立刻 `return RUNNING`、idx 不動 → 這圈到此為止
   - `FAILURE` → reset、`return FAILURE`

2. **`current_child_idx_` 是「進度記憶點」。** 它是 member 變數、跨 tick 保留。while 的起點是它、不是 0。若上一圈卡在某個 RUNNING child，下一次 `tickRoot()` 直接從該 child 繼續，**不會回頭重跑已 SUCCESS 的 child**。

3. **跑完會 reset。** 全 SUCCESS 或任一 FAILURE 時 `resetChildren()` 把 child 打回 IDLE、`idx=0`；`tickRoot` 也把 root 設回 IDLE，整棵樹能乾淨重跑。

---

## 6. 兩種情境對照

### (A) 全是 SyncAction（每個秒回 SUCCESS）

一次 `Sequence::tick()` 就把三個 child 各 tick 一次、全部跑完，**全在同一圈 `tickRoot()` 裡**：

```
tickRoot() 第1圈:
  Sequence::tick()
    idx=0 → PrintMessage("1") executeTick ×1 → SUCCESS → idx=1
    idx=1 → PrintMessage("2") executeTick ×1 → SUCCESS → idx=2
    idx=2 → PrintMessage("3") executeTick ×1 → SUCCESS → idx=3
    return SUCCESS                     ← 整棵樹結束
```

→ 三行瞬間印完、engine 第一圈就拿到 SUCCESS。`loopTimeout` 100ms 在這裡**沒派上用場**（因為沒有 node 回 RUNNING）。

### (B) 中間那個是 StatefulAction（要跑 3 圈才 SUCCESS）

這時才看得到「一圈 tick 一下」、`loopTimeout` 真的造成等待：

```
tickRoot() 第1圈: Seq::tick → idx0 "1" SUCCESS,idx=1 → idx1 Move→RUNNING → return RUNNING
tickRoot() 第2圈: Seq::tick → idx1 Move→RUNNING → return RUNNING        (沒回去碰 "1")
tickRoot() 第3圈: Seq::tick → idx1 Move→SUCCESS,idx=2 → idx2 "3" SUCCESS,idx=3 → return SUCCESS
```

第 2、3 圈因為 `current_child_idx_` 記著 idx=1，Sequence 不回頭 tick 已 SUCCESS 的 "1"，直接從卡住的 child 繼續。每圈之間隔的就是 `loopTimeout`（100ms）。

---

## 7. 一句話總結

- `tickRoot()` → `rootNode()->executeTick()` → `tick()`，composite 在自己的 `tick()` 裡再 `executeTick()` child，遞迴往下成 DFS。
- Sequence：**從 `current_child_idx_` 往後，每個 child 一次只 tick 一下；SUCCESS 才換下一個，RUNNING 就停在原地等下一圈。**
- `loopTimeout` 只有在「有 node 回 RUNNING」時才會造成等待；全 Sync 的樹會在一圈內跑完。
- `SyncActionNode` 不能回 `RUNNING`，要看到 RUNNING 行為得用 `StatefulActionNode`。
