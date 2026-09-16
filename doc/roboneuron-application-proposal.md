# Applying the RoboNeuron Mechanisms to This Workspace

> Target: `src/syncai_ros_mcp/` (the existing MCP runtime server)
> Related: `doc/mcp-server-proposals.md` (another set of proposals; see §7 for how they relate)
> Paper: RoboNeuron: A Middle-Layer Infrastructure for Agent-Driven Orchestration
> in Embodied AI (arXiv:2512.10394v2, Institute of Automation, Chinese Academy of Sciences)
> Status: **proposal, not implemented**.

RoboNeuron is an infrastructure layer that sits between "an LLM agent's MCP tool calling" and "the ROS2
middleware". This note answers one question: **which of its mechanisms are worth bringing into this workspace, and
which are not**.

The conclusion up front: this stack already has RoboNeuron's skeleton — `syncai_ros_mcp` is its control plane, and
the Temporal backend carries the lifecycle governance. What is genuinely worth adding are three mechanisms:
**schema-based tool derivation, typed capability tools, and the semantics of "switching inside a stable
boundary"**. And there is a ready-made perfect match: **gait policy switching is our inference switching**.

---

## 0. Quick tour of the paper's mechanisms (only the ones used here)

| Mechanism | What the paper does | In one sentence |
|---|---|---|
| Schema-based tool derivation (Alg. 1) | Derive tool signatures automatically from ROS message definitions; register `(name, Σ, Encoder, Publisher)` into a registry | The agent sees typed tools, not a generic publish |
| Direct path | tool call → validate → encode → publish; supports short command sequences with step durations | One-shot, low-latency primitives |
| Closed-loop path (PIC) | perception–inference–control as three modules, chained over topics, with a fixed action contract | Long-running closed-loop behaviour |
| Lifecycle control | Each long-running module is its own OS process (`spawn`); the stop tool does a bounded wait, then force-terminates | A closed loop is a governed service, not an unattended background task |
| Stable inference boundary | VLA-specific logic is confined inside the inference module; swapping backend / runtime / acceleration preset leaves the surrounding topic wiring untouched | topology-preserving switching |

The paper's own positioning statement matters: it is **not** a task planner; orchestration is left to an external
LLM agent. That is exactly isomorphic to our architecture: LLM agent → MCP → Temporal workflow → BT navigator. So
adopting it requires no change to any task-orchestration design.

---

## 1. Correspondence table: paper concepts ↔ current state of this stack

| RoboNeuron mechanism | Our current state | Gap |
|---|---|---|
| Unified MCP tool interface | ✅ `syncai_ros_mcp` (FastMCP, port 8000) | Already have it |
| Schema-based tool derivation | ❌ Only the generic `publish_once(topic, msg_type, dict)` | **Biggest gap, see §2** |
| Direct path (validate → encode → publish) | ⚠️ Exists, but no typed validation, no sequence publishing | §4 |
| Closed-loop path + lifecycle | ⚠️ Temporal tasks (create/get/cancel) + byobu `switch_mode` | §5; coarse-grained but the skeleton is there |
| Stable boundary / backend switching | ⚠️ `SetPolicyMode` (PPO/HIMLOCO/CHAMP/ISSAC) exists but is not exposed to the agent | **Just one layer short, see §3** |
| Stop tool (bounded wait → force) | ⚠️ driver_manager has a safe-shutdown path, but it is not an agent tool | §4 |
| PIC (perception–inference–control) | ❌ No camera, no VLA (nothing under `src/` consumes torch/onnx/Image) | **Not applicable yet, see §6** |

---

## 2. Proposal A: Schema-based tool derivation → add `tools/registry.py` ★ core

This is the paper's core contribution, and it lands squarely on three pain points of the existing implementation.
Today, for the agent to move the robot, it has to do this:

```
publish_once(topic='/robot01/cmd_vel', msg_type='geometry_msgs/msg/Twist',
             msg={'linear': {'x': 0.3}})
```

1. **The agent has to assemble the absolute topic name itself** — a direct violation of the iron rule in CLAUDE.md
   (never hardcode `/<robot_id>/…`). The root cause is that `mcp_server_node.py` **has no namespace**, so the agent
   is forced to use absolute names. That is not the agent's fault; the tool surface forces it to break the rule.
2. **There is no argument schema** — the error surfaces only when `set_message_fields` fails, so the agent can only
   guess the fields.
3. **The agent has to re-derive the msg_type string on every call** — exactly the source of interface drift the
   paper describes.

### Approach (corresponding to the paper's Algorithm 1)

- Add a capability manifest, suggested location `config/capabilities.yaml`. Each entry:
  `{tool_name, topic (relative name), msg_type, description, qos}`. Putting it in config rather than hard-coding it
  in Python is consistent with the "the window list is data" philosophy of `config/sessions/*.yaml`.
- On startup, `registry.py` does the following for each entry: `get_message()` → recursively expand
  `get_fields_and_field_types()` (`get_message_details` in `topics.py` **already has this recursion written**;
  extract and reuse it, do not copy it) → dynamically generate a pydantic model as the argument schema → create a
  **persistent** publisher bound to the relative topic → register via `mcp.tool()`.
- **Put the MCP node inside the `robot_id` namespace** (reading `config/system.ini` like every other launch file).
  The registry's relative topics then resolve automatically to `/<robot_id>/…`, and the agent no longer knows the
  namespace exists. This is exactly what the paper's Case I, "the same velocity tool bound to different platforms",
  means on our fleet: **one manifest, one MCP server each on robot01 / robot02, an identical tool surface**.

A side benefit: registry entries carry per-topic QoS, which incidentally resolves the TODO already noted in
`topics.py` — the map topic needs TRANSIENT_LOCAL but is hard-coded VOLATILE.

The generic `publish_once` / `subscribe_once` **stay** as an escape hatch: the paper's Fig. 1 also keeps a Direct
Path down to the low level. Criterion: a capability that is in the manifest goes through the typed tool; only those
that are not fall back to the generic tools.

The first batch of manifest entries: `cmd_vel` (Twist), the initial pose for relocalize, plus the locomotion tools
of §3 (those are services and do not go through this topic registry, but they share the "typed + capability card"
presentation).

## 3. Proposal B: gait policy switching = our topology-preserving switching → add `tools/locomotion.py`

The paper's "topology-preserving inference switching" means: swap the backend, but the observation stream, the
action contract, and the downstream wiring all stay put. Our gait controller has exactly this structure:

- `cmd_vel` in, gait out — a fixed contract
- `SetPolicyMode` (0 PPO / 1 HIMLOCO / 2 CHAMP / 3 ISSAC) — the **backend switch**; swapping the RL policy leaves
  every piece of nav-stack wiring untouched
- `SetMotionKey` / `SetSpeedScale` — runtime presets

Today these are reachable only through the generic `call_service`. Suggested typed tools:

```
set_locomotion_policy(policy)   # send SetPolicyMode and read back the measured value, see below
set_motion_state(key)           # likewise
emergency_stop()                # corresponds to the stop_base capability card in the paper's Case I
```

### A caveat that must be written into the tool behaviour: COMMANDED vs MEASURED

Underneath, these services are **one-way UDP with no ack** (`udpSend()` discards the return value of `sendto()`),
so a `success` return only means "it was sent". The comments in `RobotLowLevelMode.msg` already spell this out
thoroughly. The correct tool design is therefore: after the call, **subscribe once to the `mode` topic (or read
`RobotState.low_level_mode`) and report the measured value**, so the agent receives both a commanded and a measured
value. This is more honest than the paper — the paper's backend switch is in-process, whereas ours crosses an
unreliable link.

⚠️ Two traps already recorded in existing documents that the tool must handle rather than let through:

- An all-zero `low_level_mode` is ambiguous ("no first sample received yet" vs the legitimate "PPO / Stand") and
  carries no freshness information. Confirm `motor_status.timestamp` is advancing before reading back.
- `policy_state` has no sentinel; `motion_state == 8` is the controller's own UNKNOWN. Integers outside the table
  (e.g. MPC) must be passed through as-is, not clamped, not treated as errors — `RobotLowLevelMode.msg` records an
  explicit design decision on this.
- ESTOP: the backend's `set_motion_key` schema accepts `'4'` but **deliberately does not forward it** (recorded in
  `doc/mcp-server-proposals.md` §5.3). `emergency_stop()` must go through the ROS service path, and be a standalone
  tool rather than hidden inside the parameter space of `set_motion_state`.

## 4. Proposal C: sequence publishing + stop semantics → completing the direct path

Paper III-B: the direct path supports "short command sequences with step durations", with an explicit termination
at the end. For a quadruped this is not a nice-to-have: the behaviour after a single Twist depends on the gait
controller's watchdog, so one `publish_once` from the agent either has no effect or has an uncontrollable one.

```
move_base_timed(vx, wz, duration_s)   # publish continuously at 10 Hz; automatically send a zero Twist when time is up
```

The registry tool supports an optional `sequence: [{msg, duration}]`, and **always appends a zero-Twist at the
end** — merging the paper's scripted motion with its stop semantics.

### The tension with `mcp-server-proposals.md` §4, and why it is not a conflict

That document explicitly opposes "continuous `cmd_vel` teleoperation", on the grounds that putting an LLM inside a
10 Hz velocity loop is a bad idea. This proposal is **not** that thing, and the criterion is the one that document
itself gives: "one-shot motion keys / nav goals are fine (they have explicit termination conditions)". The
termination condition of `move_base_timed` is **inside the tool** — the duration runs out and the zero-Twist closes
it off; the LLM is not in the loop, it only initiates one bounded motion. What is genuinely forbidden is "the agent
decides a velocity every 100 ms", and that is still not done.

Nonetheless, two guardrails are recommended: a `duration_s` cap (e.g. 5 s) hard-coded into the tool; and under the
real-robot profile, require the manifest to explicitly enable this tool (sim enabled by default).

## 5. Proposal D: expose the lifecycle, rather than rebuild the spawn mechanism

The paper manages long-running modules with `spawn` + a stop tool. **No need to copy that** — the stack already has
two ready-made lifecycle layers; the agent just cannot reach them:

- **Coarse-grained**: `NodeManager`'s byobu sessions. Add `tools/lifecycle.py`: `get_robot_mode()` /
  `switch_robot_mode(mode)` wrapping `GetMode` / `SwitchMode` service clients. The tool description must carry the
  paper-style semantic warnings: a switch is a destructive, long operation (~40 byobu commands); switching away from
  MANUAL drops an unsaved map (`pgo_node` keyframes live in RAM; `save_maps` is the only serialisation path);
  MAINTENANCE cannot be switched into. All of this is in the comments of `SwitchMode.srv`; just move it into the
  description.
- **Fine-grained (closed-loop tasks)**: create/get/cancel in `tasks.py` go through Temporal, and that **already
  is** what the paper calls an "explicitly governed system service, not an unmanaged background task" — the
  workflow query is the paper's "agent monitors progress", and cancellation is the stop tool. **Nothing to change
  here**; this is where we are stronger than the paper's prototype.

## 6. Counter-argument: do not build PIC for now

PIC presupposes a vision stream + a VLA policy. This stack currently has **no camera and no ROS-side model
inference** (the G23's RL policy runs on the gait controller, not on the ROS side). Forcing PIC in has no mount
point.

But it is worth **reserving the contract**: if a VLA is added in future (semantic goal navigation, a manipulator),
follow the paper and first pin down the action-contract topic (the paper carries a 6-DoF delta + gripper in a
`Float64MultiArray`), confining the inference module inside the boundary. At that point perception = a newly added
camera driver node, control = the existing controller / task_runner; only the middle cell is filled in, and the
surroundings are not rewired.

Another extension the paper lacks but we will need sooner or later: **action tools**. The paper admits it only did
topic-based exposure (services / actions are future work), while this stack's core entry points happen to be
actions (`NavigateToPose`, `ExecuteTask`). Today the agent can only detour through the backend REST. Letting the
agent issue navigation goals directly (without Temporal scheduling) means writing send_goal / feedback / cancel
ourselves — outside the paper's scope, and if the Temporal path suffices it can go last. When implementing, mind the
threading iron rule of `mcp-server-proposals.md` §5.2: waiting on an action result must not block spin.

---

## 7. Relationship to `mcp-server-proposals.md`

The two documents are complementary and do not overlap: that one answers "**what went wrong**" (log archaeology,
stack doctor, cross-source correlation) and belongs to the diagnostic plane; this one answers "**how the agent uses
the robot as a set of typed capabilities**" and belongs to the control plane. The shared judgments are already
cross-referenced: commanded vs measured double verification (their §2 example C = our §3), tool tiers and the
explicit ESTOP refusal (their §5.3), the rclpy spin thread (their §5.2).

Implementation criterion: a tool that merely types an existing interface → belongs to this document; a tool that
needs cross-source correlation or encodes tribal knowledge into a conclusion → belongs to that one.

---

## 8. Suggested landing order

1. **Proposal A** (registry + manifest + namespacing the MCP node) — solves typed tools, the namespace violation,
   and the QoS TODO in one go
2. **Proposal B** (locomotion, with measured-state read-back) — small effort, most demonstrative; this is our
   topology-preserving switching demo
3. **Proposal C** (timed sequence + zero-Twist close-off) — safety
4. **Proposal D** (get_mode / switch_mode)
5. Action tools (as needed, last)

A + B together are roughly 300–400 lines of Python, all landing in the vendored `syncai_ros_mcp` package, without
touching a single line of the C++ nav stack — matching the paper's positioning: add things at the middleware layer,
and the existing control stack is not rewired at all.
