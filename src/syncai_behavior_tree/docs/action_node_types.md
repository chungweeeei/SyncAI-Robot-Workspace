# The three ActionNode kinds: Sync / Stateful / Async

> Companion examples:
> - `examples/sync_action_demo.cpp` — `SyncActionNode`
> - `examples/stateful_action_demo.cpp` — `StatefulActionNode`
> - `examples/async_action_demo.cpp` — `AsyncActionNode`
>
> Source: `src/third-party/behaviortree_cpp_v3/` (mainly `include/.../action_node.h` and `src/action_node.cpp`)

---

## 0. What they share

All three are **leaf nodes** (no children; they are the ones that actually do the work), and all inherit from `ActionNodeBase : LeafNode`.
There is exactly one difference: **how long the action takes and how `tick()` runs**.

```
TreeNode
└─ LeafNode
    └─ ActionNodeBase
        ├─ SyncActionNode       <- completes instantly
        ├─ StatefulActionNode   <- long action, progress polled in slices on the tree thread
        └─ AsyncActionNode      <- long action, the framework spawns a worker thread for it
```

> Note: `AsyncActionNode` is an **action node (leaf)**, not a control node.

---

## 1. Quick comparison

| | Where tick() runs | May tick() block | How progress is sliced / polled | Cancellation |
|---|---|---|---|---|
| **Sync** | tree thread | No (it would freeze the whole tree) | Not needed; returns SUCCESS/FAILURE immediately | N/A (too fast) |
| **Stateful** | tree thread | No | You write `onRunning()`, returning RUNNING each tick | `onHalted()` |
| **Async** | **worker thread spawned by the framework** | **Yes** (sleep / loops are fine) | Automatic: keeps returning RUNNING until the worker finishes | `tick()` must poll `isHaltRequested()` and bail out itself |

The one-line version:
- **Sync / Stateful** tick on the tree thread, so **tickRoot waits for them to finish**; those functions must return immediately.
- **Async** has its tick moved to a worker thread, so **tickRoot does not wait**; it returns RUNNING and carries on looping.

---

## 2. SyncActionNode

Only `tick()` is overridden, and it **must return `SUCCESS` / `FAILURE` right away; returning `RUNNING` is not allowed**
(`SyncActionNode::executeTick()` throws `LogicError` if it sees RUNNING; see `action_node.cpp:54`).

```cpp
class PrintMessage : public BT::SyncActionNode {
  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    std::cout << msg.value() << "\n";
    return BT::NodeStatus::SUCCESS;   // done on the spot
  }
};
```

Use for: logging, setting blackboard values, making decisions - anything that completes instantly.
**Misuse warning**: sleeping or waiting on IO inside `tick()` freezes the whole tree (even `cancelRequested` / `onLoop` stop running).

---

## 3. StatefulActionNode

`tick()` is already implemented by the framework (**cannot be overridden**); it dispatches to three callbacks according to the node's current status:

| Node status | Callback | When |
|-----------|------|------|
| IDLE | `onStart()` | Not started yet, or reset after the previous run finished |
| RUNNING | `onRunning()` | The previous tick returned RUNNING |
| (halted) | `onHalted()` | When the parent / engine calls `halt()`; used for cleanup |

```cpp
class CountDown : public BT::StatefulActionNode {
  BT::NodeStatus onStart() override {       // kick off (non-blocking), return RUNNING immediately
    progress_ = 0; getInput("ticks", ticks_);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {     // poll progress once per tick, return immediately
    if (++progress_ >= ticks_) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { /* cleanup */ }
};
```

Key point: `onStart` / `onRunning` both run on the **tree thread**, so they **must return immediately**. Work is sliced by returning RUNNING each tick and re-entering on the next round, not by sleeping inside the function.
The actual time-consuming work should happen elsewhere (another process / a ROS action server); this node is only responsible for "kicking off" and "polling progress".

### Observed behaviour (`stateful_action_demo`)

`CountDown ticks="5"` spans about 6 rounds of the `run()` loop, each `loopTimeout` (100 ms) apart.
The Sequence stays parked at its index and does not go back to re-run the children that already returned SUCCESS; it only advances once CountDown returns SUCCESS.

---

## 4. AsyncActionNode

The framework runs your `tick()` on **another thread**, so `tick()` **may block / loop / sleep**.

```cpp
class HeavyWork : public BT::AsyncActionNode {
  BT::NodeStatus tick() override {          // this tick() runs on the worker thread
    for (int i = 1; i <= steps; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // blocking is OK
      if (isHaltRequested()) return BT::NodeStatus::FAILURE;         // must poll for cancellation yourself
    }
    return BT::NodeStatus::SUCCESS;
  }
};
```

### Mechanism (`AsyncActionNode::executeTick`, `action_node.cpp:160`)

```cpp
NodeStatus AsyncActionNode::executeTick() {
  if (status() == NodeStatus::IDLE) {       // <- true only on the very first tick
    setStatus(NodeStatus::RUNNING);
    thread_handle_ = std::async(std::launch::async, [this]() {
        auto status = tick();               // runs your tick() on the worker thread
        if (!isHaltRequested()) setStatus(status);
        // exception handling: stash the worker thread's exception in exptr_ for the tree thread to rethrow
        emitStateChanged();
    });
  }
  // always reaches here: if exptr_ holds something, rethrow it (moves the exception back to the tree thread)
  return status();                          // <- returns immediately, does not wait for the worker
}
```

Breaking it down:

1. **`std::async(std::launch::async, ...)` starts the moment it is called** - `std::launch::async` forces a new thread to run the lambda immediately; it is not "create first, start later".
2. **Exactly one thread is spawned, exactly once, on the IDLE->RUNNING transition.** From the second tick on, `if(IDLE)` is false and it only does `return status()`, a pure status read; **no further threads are spawned**. So one Async node = one worker thread running `tick()` once.
3. **The `[this]` capture**: the lambda calls members such as `tick()` / `isHaltRequested()` / `setStatus()`, so it captures `this`.
4. **Deliberately no `setStatus` when halted**: at that point `halt()` owns the status, which avoids a race.
5. **Exceptions across threads**: an exception thrown on the worker thread cannot be caught by a try/catch on the tree thread, so it is stored in `exptr_` via `std::current_exception()`; the next `executeTick()` calls `std::rethrow_exception()` on the tree thread, where the try/catch around `run()` (`behavior_tree_engine.cpp:69`) catches it.
6. **`thread_handle_` must be kept as a member**: if the `future` were discarded as a temporary, its destructor would block until the thread finishes, degrading to synchronous execution. Keeping it is what makes it truly run in the background, and `halt()` relies on it for `wait()`.

### halt behaviour (`action_node.cpp` `AsyncActionNode::halt`)

```cpp
void AsyncActionNode::halt() {
  halt_requested_.store(true);              // set the flag
  if (thread_handle_.valid()) thread_handle_.wait();  // wait for the worker thread to finish
}
```

-> So `tick()` **must check `isHaltRequested()` periodically and bail out on its own**; otherwise `halt()` keeps `wait()`ing until the loop ends naturally and cancellation is effectively broken.

### Observed behaviour (`async_action_demo`)

The worker thread advances one step every 1000 ms while the tree thread loops every 100 ms, so the log shows the two threads' messages **interleaved**:

```
(tree thread) run loop #1
...
(tree thread) run loop #10
    [HeavyWork] (worker thread) finished step 1/5
(tree thread) run loop #11
...
```

`tickRoot()` never stops for HeavyWork: ticking an Async node is just `return status()`, then straight on to `onLoop()` -> sleep 100 ms.

### Two caveats (also why v3 recommends Async less)

1. **You own cancellation**: `tick()` must poll `isHaltRequested()`.
2. **You own thread safety**: the worker and the tree thread run concurrently, so shared state / the blackboard need locking to avoid races.

---

## 5. When do multiple worker threads occur?

A single Async node spawns **one** worker thread. To get several in parallel you need **several** Async nodes RUNNING at the same time - typically under a `Parallel`, which ticks several children per round, each spawning its own worker thread. Inside a `Sequence` only one child is RUNNING at a time, so there is only one.

---

## 6. Choosing in practice

- Instant work -> **Sync**
- Long action that can be split into "kick off + poll progress each tick" -> **Stateful** (preferred)
- Long action that is a single blocking function you do not want to split -> **Async** (but you handle cancellation and thread safety yourself)

> Real ROS long actions (navigation, planning, control) almost always use **`BtActionNode`** (a non-blocking poll model built on the Stateful idea), handing the heavy work to the ROS action server in its own process while the BT side only polls without blocking - no thread management of your own, and cancellation stays clean. See `plugins/action/` (e.g. `ComputePathToPose`).

---

## 7. Related documents

- [behavior_tree_tick_notes.md](behavior_tree_tick_notes.md) — how `tickRoot()` ticks from the root all the way down to the leaves, the `current_child_idx_` bookmark in Sequence, and how `loopTimeout` relates to RUNNING
