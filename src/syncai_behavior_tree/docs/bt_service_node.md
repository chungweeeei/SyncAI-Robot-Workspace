# BtServiceNode: a BT node wrapping a ROS 2 service

> Source: `include/syncai_behavior_tree/bt_service_node.hpp`
> Designed with the same philosophy as `BtActionNode` (`bt_action_node.hpp`, which wraps a ROS action).

---

## 1. Inheritance: the "fourth sibling" of the three

`BtServiceNode` derives directly from `BT::ActionNodeBase`, at the **same level** as Sync / Stateful / Async, neither above nor below them.

```
ActionNodeBase   (common base of leaf actions; only pins type() to ACTION, does not implement tick())
├─ SyncActionNode       <- framework-enforced contract: "tick must return immediately"
├─ StatefulActionNode   <- framework splits tick() into onStart/onRunning/onHalted
├─ AsyncActionNode      <- framework spawns a worker thread to run tick()
└─ BtServiceNode        <- relies on none of the scaffolding above; hand-writes tick() + halt()
```

`ActionNodeBase` itself is almost empty (`action_node.h:35`): it only declares itself a leaf and returns `ACTION` from `type()`; it **does not implement `tick()` / `executeTick()`**. That makes it the "freest" action base, and `BtServiceNode` picks it precisely so it can fully customise the structure of `tick()` (a hook flow specific to services).

---

## 2. Template Method: tick() is the skeleton, business logic goes in hooks

Subclasses **should not override `tick()`**. `tick()` is the "invariant flow" written in the base; subclasses only override the hooks, the "details that vary".

```cpp
BT::NodeStatus tick() override {            // leave this alone
  if (!request_sent_) {
    should_send_request_ = true;
    on_tick();                              // <- hook 1: fill in the request
    if (!should_send_request_) return FAILURE;
    future_result_ = service_client_->async_send_request(request_).share();  // non-blocking send
    sent_time_ = node_->now();
    request_sent_ = true;
  }
  return check_future();                     // poll for the response (calls on_completion internally)
}
```

| Hook | What you do here | Default |
|------|----------------|------|
| `on_tick()` | Read values from ports / the blackboard into `request_`; or set `should_send_request_ = false` to skip | empty |
| `on_completion(response)` | Parse the response, write back to the blackboard, return the final `NodeStatus` | returns SUCCESS |
| `on_wait_for_result()` | What to do on each timeout while waiting for the response (e.g. publish feedback) | empty |

The `request_sent_` flag is its miniature state machine: `false` = "a new request should be sent", `true` = "sent, waiting". It replaces the onStart/onRunning dispatch of `StatefulActionNode`.

---

## 3. What the constructor does

```cpp
BtServiceNode(name, conf, service_name="")
: BT::ActionNodeBase(name, conf), ... {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");  // fetch the shared ROS node from the blackboard
  callback_group_ = node_->create_callback_group(MutuallyExclusive, false);  // dedicated callback group
  callback_group_executor_.add_callback_group(callback_group_, ...);
  server_timeout_ = ... ;                  // total time limit, from the blackboard / port
  max_timeout_ = bt_loop_duration * 0.5;   // a single tick spins for at most half a period
  service_client_ = node_->create_client<ServiceT>(service_name_, ..., callback_group_);
  if (!service_client_->wait_for_service(...)) throw ...;  // throw if the server is not there
}
```

- **`config()`**: inherited from `TreeNode`; returns the `NodeConfiguration` received at construction (holding the shared `blackboard` and the port remapping). `config().blackboard` is the blackboard shared by the whole tree.
- **The `template` in `->template get<T>()`**: a C++ disambiguation keyword (nothing to do with the blackboard). Because the member template `Blackboard::get<T>` is called from inside a class template, it tells the compiler that the following `<` opens a template argument list and is not a less-than operator.
- **`BtServiceNode() = delete;`**: explicitly deletes the no-argument constructor. This node cannot be initialised without a `NodeConfiguration` (blackboard, ROS node), so misuse without arguments is stopped at compile time.
- **Dedicated callback group + executor**: the service client is bound to its own group, so spinning only handles "this node's service callbacks", without interfering with the main executor or risking re-entrancy.

---

## 4. check_future(): the core poll logic

Two different timeouts are the key to understanding it:

| Variable | Meaning |
|------|------|
| `server_timeout_` | The **total time limit** of the whole service call (from send to giving up) |
| `max_timeout_` = `bt_loop_duration * 0.5` | How long a **single tick** may spin at most |

```cpp
virtual BT::NodeStatus check_future() {
  auto elapsed   = now - sent_time_;
  auto remaining = server_timeout_ - elapsed;     // how much of the total budget is left

  if (remaining > 0ms) {
    auto timeout = min(remaining, max_timeout_);   // how long this tick may spin at most
    rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);

    if (rc == FutureReturnCode::SUCCESS) {          // future completed (response arrived)
      request_sent_ = false;
      return on_completion(future_result_.get());   // let the hook decide the final status
    }
    if (rc == FutureReturnCode::TIMEOUT) {          // nothing arrived this time
      on_wait_for_result();
      if (now - sent_time_ < server_timeout_)
        return BT::NodeStatus::RUNNING;             // total limit not reached -> come back next round
    }
  }
  // total limit exhausted / interrupted
  request_sent_ = false;
  return BT::NodeStatus::FAILURE;
}
```

### Three exits

| Case | Condition | Returns | `request_sent_` |
|------|------|------|------------------|
| Response arrived this tick | `rc == SUCCESS` | the result of `on_completion()` | reset to false |
| Not yet, total limit not reached | `rc == TIMEOUT` and `elapsed < server_timeout_` | **RUNNING** | stays true (keep polling next tick) |
| Total limit exhausted / interrupted | everything else | **FAILURE** | reset to false |

### Do not confuse the two levels of SUCCESS

- **`FutureReturnCode::SUCCESS`** = "the future completed (a response was received)", regardless of what it contains.
- **`NodeStatus::SUCCESS`** = your verdict in `on_completion(response)`, after inspecting the contents, that "this behavior succeeded".

---

## 5. spin_until_future_complete: the future only advances if something spins

The future returned by `async_send_request` **is not ready at first**. The response is only taken in when something spins:

```
service server returns a response
  -> DDS/rmw receives the bytes (queued at the lower layer)
  -> [the future is still not-ready at this point!]
  -> spin_until_future_complete drives the executor to process that event
  -> the client callback writes the value into future_result_ -> future becomes ready -> returns SUCCESS
```

So this one line has a **dual role**: (1) pump - actually processing ROS events; (2) wait - watching whether the future is done while processing, for at most `timeout`. **Without spinning, the future never completes, even after the response has reached the rmw layer.**

This differs from `AsyncActionNode` (which has its own thread advancing automatically): a service node **only advances because the tree thread comes by to spin** - polling is used to hand-simulate the effect of a done-callback (`on_completion` plays the post-completion handler, but it is driven by the loop polling repeatedly, not by an event push).

---

## 6. Latency analysis: how long after the response completes is SUCCESS declared?

Because `spin_until_future_complete` **actively spin-waits for up to `max_timeout_`** (about half a period) on every tick, rather than "sampling instantaneously", one tick cycle has two phases (using loop 100 ms, max_timeout_ 50 ms as the example):

```
t=100ms ┬ tick -> spin up to 50ms   <- these 50ms are "spinning"; a response is caught the moment it lands
t=150ms ┤ nothing -> RUNNING
        │ onLoop + sleep            <- these 50ms "nobody spins" = blind spot
t=200ms ┴ next tick -> spin another 50ms ...
```

- A response landing in the **spin window** (e.g. 101 ms) -> detected almost instantly; that very tick returns SUCCESS (**no need to wait until 200 ms**).
- A response whose low-level arrival falls in the **blind spot** (e.g. 160 ms) -> nobody spins, the future does not advance; it is caught at the start of the next spin at 200 ms.

-> **Worst-case latency is about one blind spot (roughly half a period), not a full round, and certainly not a skipped round.**

Why not spin for the whole period (eliminating the blind spot)? That would pin the tree thread on this node for the whole round; `cancelRequested` / `onLoop` / every other node would stop moving. `max_timeout_ = 0.5 x bt_loop_duration` deliberately leaves half the time for the loop to breathe and check for cancellation - trading a little latency for the loop's responsiveness.

---

## 7. halt()

```cpp
void halt() override {
  request_sent_ = false;
  setStatus(BT::NodeStatus::IDLE);
}
```

On cancellation it resets the state machine and returns to IDLE. Note that it does **not** actively cancel a request already sent (services, unlike actions, have no notion of cancel) - a response already in flight is ignored, and a fresh round starts next time.

---

## 8. Positioning relative to the three ActionNode kinds

| | How it waits for the result | Returns RUNNING | Spawns a thread | Who writes tick() |
|---|---|---|---|---|
| Sync | Does not wait (returns immediately) | No | No | you |
| Stateful | Never blocks; polls once per tick | Yes | No | framework (you write onRunning) |
| Async | tick blocks on a worker thread | Yes | Yes | you (running on the worker thread) |
| **BtServiceNode** | **Blocks the tree thread, but capped at half a period per tick** | Yes | No (uses executor spin) | framework (you write on_tick/on_completion) |

`BtServiceNode` sits between "never blocks (Stateful)" and "blocks a different thread for the whole duration (Async)": **a time-capped spin on the tree thread** - because it must spin to drive the ROS callbacks, yet must not stall the loop.

---

## 9. Related documents

- [action_node_types.md](action_node_types.md) — the three ActionNode kinds: Sync / Stateful / Async
- [behavior_tree_tick_notes.md](behavior_tree_tick_notes.md) — the tickRoot -> executeTick -> tick propagation mechanism, RUNNING and loopTimeout
