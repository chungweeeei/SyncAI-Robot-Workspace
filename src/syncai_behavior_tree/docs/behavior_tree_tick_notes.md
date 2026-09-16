# Notes on the BehaviorTree.CPP tick mechanism

> Using `examples/sync_action_demo.cpp` as the running example, this walks through how `tickRoot()` ticks from the root all the way down to every leaf.
> Source path: `src/third-party/behaviortree_cpp_v3/`

---

## 1. The three Action base classes

When implementing a behavior, pick the base class by how long the action takes to complete:

| Base class | Use case | Tick behaviour |
|----------|----------|-----------|
| `BT::SyncActionNode` | Actions that complete instantly | A single `tick()` returns `SUCCESS`/`FAILURE` directly; **returning `RUNNING` is not allowed** |
| `BT::StatefulActionNode` | Long actions that need many ticks | Split into `onStart()` / `onRunning()` / `onHalted()`; may return `RUNNING` |
| `BT::AsyncActionNode` | Actions that run on their own thread | `tick()` runs on a background thread (less recommended in v3) |

`PrintMessage` in `sync_action_demo.cpp` is a `SyncActionNode`: it reads a `message` port, prints it, and returns `SUCCESS`.

---

## 2. The engine's run() loop

`BehaviorTreeEngine::run()` (`src/behavior_tree_engine.cpp:36`) ticks the whole tree at a fixed rate:

```cpp
while (rclcpp::ok() && result == RUNNING) {
  if (cancelRequested()) { tree->rootNode()->halt(); return CANCELED; }
  result = tree->tickRoot();   // <- actively tick the whole tree once
  onLoop();
  loopRate.sleep();            // <- sleep until loopTimeout (e.g. 100 ms) is used up
}
```

- `loopTimeout` (e.g. 100 ms) is the **period of one full loop iteration**: each round calls `tickRoot()` once and then sleeps for the remainder.
- This is not "passively checking status"; it is an **active tick** - the tick itself is what drives a node forward. A node that is not ticked does not move.
- The loop ends when `tickRoot()` returns `SUCCESS`/`FAILURE` (or on cancel).

---

## 3. How the tick propagates down from the root

The propagation mechanism is just the **recursion `executeTick()` <-> `tick()`**; there is no magic traversal.

### The three-level call chain

```
Tree::tickRoot()                    <- called by the engine every round
  └─ rootNode()->executeTick()      <- TreeNode::executeTick(), the shell shared by every node
        └─ tick()                   <- virtual; the concrete type decides which version runs
```

### `tickRoot()` (`bt_factory.h:210`)

```cpp
NodeStatus ret = rootNode()->executeTick();
if (ret == SUCCESS || ret == FAILURE)
    rootNode()->setStatus(IDLE);     // once the whole tree finishes, reset root to IDLE so it can run again
return ret;
```

### `TreeNode::executeTick()` (`tree_node.cpp:32`)

The shell shared by every node: pre-condition -> `tick()` -> post-condition -> `setStatus()`.
The `tick()` it calls is **virtual**, so which version actually runs is decided by the node's concrete type:

- `<Sequence>` -> `SequenceNode::tick()`
- `PrintMessage` -> `PrintMessage::tick()`

### Propagating downward = a control node ticking its children inside its own tick()

"Ticking downward" is not a framework traversal; **every composite node actively calls `executeTick()` on its children inside its own `tick()`**.
If a child is itself a composite it recurses one level further, which makes the whole thing a DFS (depth-first).

---

## 4. XML structure and rootNode

The XML in `sync_action_demo.cpp`:

```xml
<root main_tree_to_execute="MainTree">   <!-- container, not a node -->
  <BehaviorTree ID="MainTree">           <!-- the frame of one named tree, not a node -->
    <Sequence name="root">               <!-- this is the actual rootNode -->
      <PrintMessage message="hello behavior tree 1"/>
      <PrintMessage message="hello behavior tree 2"/>
      <PrintMessage message="hello behavior tree 3"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- `<root>` and `<BehaviorTree>` are **not BT nodes**; they are only containers / metadata.
- The real node is the single child inside `<BehaviorTree>` -> here that is `<Sequence>`.
- So `tree.rootNode() == <Sequence>`, and `tickRoot()` **starts at the Sequence level**.
- `<BehaviorTree>` may hold only one node; to run several actions, wrap them in a composite (such as `<Sequence>`).
- The root may also be a leaf directly (e.g. `<BehaviorTree>` containing only `<PrintMessage/>`); then `rootNode()` is that leaf and there is no Sequence level.

---

## 5. SequenceNode::tick() in detail

`src/controls/sequence_node.cpp:31`:

```cpp
NodeStatus SequenceNode::tick()
{
  const size_t children_count = children_nodes_.size();   // 1. child count as the loop bound
  setStatus(NodeStatus::RUNNING);                          // 2. set itself to RUNNING first

  while (current_child_idx_ < children_count)              // 3. tick forward in order from the "current progress"
  {
    TreeNode* child = children_nodes_[current_child_idx_];
    const NodeStatus child_status = child->executeTick();  // <- tick "this one" child exactly once

    switch (child_status) {
      case RUNNING:  return RUNNING;                       // child not done -> Seq returns RUNNING too, idx unchanged
      case FAILURE:  resetChildren(); current_child_idx_ = 0; return FAILURE;  // the whole chain fails
      case SUCCESS:  current_child_idx_++; break;          // idx++ only on success; while continues -> tick the next
      case IDLE:     throw LogicError("...");              // a child must not return IDLE
    }
  }

  // every child returned SUCCESS
  resetChildren(); current_child_idx_ = 0;
  return SUCCESS;
}
```

### Three key ideas

1. **Each child is ticked only "once" per round.** It is not "tick one child to completion, then move to the next". What the child returns decides whether the while loop continues:
   - `SUCCESS` -> `idx++`, while continues -> the next child is ticked right away, **in the same** `Sequence::tick()`
   - `RUNNING` -> immediately `return RUNNING`, idx unchanged -> this round ends here
   - `FAILURE` -> reset, `return FAILURE`

2. **`current_child_idx_` is the "progress bookmark".** It is a member variable and survives across ticks. The while loop starts from it, not from 0. If the previous round got stuck on a RUNNING child, the next `tickRoot()` resumes directly at that child and **never goes back to re-run children that already returned SUCCESS**.

3. **It resets when done.** On all-SUCCESS or any FAILURE, `resetChildren()` puts the children back to IDLE and `idx=0`; `tickRoot` also sets the root back to IDLE, so the whole tree can be re-run cleanly.

---

## 6. Two scenarios side by side

### (A) All SyncActions (each returns SUCCESS immediately)

A single `Sequence::tick()` ticks the three children once each and runs the whole thing to completion, **all within the same `tickRoot()` round**:

```
tickRoot() round 1:
  Sequence::tick()
    idx=0 -> PrintMessage("1") executeTick x1 -> SUCCESS -> idx=1
    idx=1 -> PrintMessage("2") executeTick x1 -> SUCCESS -> idx=2
    idx=2 -> PrintMessage("3") executeTick x1 -> SUCCESS -> idx=3
    return SUCCESS                     <- the whole tree is done
```

-> The three lines print instantly and the engine gets SUCCESS on its very first round. The 100 ms `loopTimeout` **never comes into play** here (because no node returned RUNNING).

### (B) The middle one is a StatefulAction (needs 3 rounds to reach SUCCESS)

Only now do you see "one tick per round" and `loopTimeout` actually causing a wait:

```
tickRoot() round 1: Seq::tick -> idx0 "1" SUCCESS,idx=1 -> idx1 Move->RUNNING -> return RUNNING
tickRoot() round 2: Seq::tick -> idx1 Move->RUNNING -> return RUNNING        (never touches "1" again)
tickRoot() round 3: Seq::tick -> idx1 Move->SUCCESS,idx=2 -> idx2 "3" SUCCESS,idx=3 -> return SUCCESS
```

In rounds 2 and 3, `current_child_idx_` remembers idx=1, so the Sequence does not go back to tick the already-SUCCESS "1"; it resumes at the stuck child. The gap between rounds is `loopTimeout` (100 ms).

---

## 7. One-line summary

- `tickRoot()` -> `rootNode()->executeTick()` -> `tick()`; a composite calls `executeTick()` on its children inside its own `tick()`, recursing downward as a DFS.
- Sequence: **starting from `current_child_idx_`, each child is ticked only once per round; SUCCESS moves on to the next child, RUNNING stays put and waits for the next round.**
- `loopTimeout` only causes a wait when some node returns RUNNING; an all-Sync tree finishes within a single round.
- `SyncActionNode` cannot return `RUNNING`; to see RUNNING behaviour you need a `StatefulActionNode`.
