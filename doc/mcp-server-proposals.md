# MCP Server Application Proposals

> Target: the MCP tool layer in this workspace that "nobody has built yet"
> Related: `src/syncai_ros_mcp/` (the existing runtime server; see §6 for how they relate)
> Status: **proposal, not implemented**. This document is where the design discussion lands; it is not a manual for existing behaviour.

This note answers two questions: what an MCP server in this workspace can provide that **cannot be obtained
today**; and where the extra value lies when one server holds both **ROS tools** and **REST tools** at the same
time.

---

## 0. Mental model: an MCP server is not an API proxy

Wrapping REST endpoints one-to-one as MCP tools is the easiest and least valuable thing to do — the agent can
already make HTTP calls. Only three kinds of thing are worth wrapping as tools:

| Type | Why it has to be a tool |
|---|---|
| **Cross-source correlation** | The answer only comes together by looking at logs, DDS, REST and Temporal at once |
| **Embedded judgment** | Dumping the raw data gives the agent nothing it can read; the tribal knowledge has to be encoded |
| **State the CLI cannot reach** | gzip-rotated multilog, Temporal workflow history, the live TF tree |

This workspace has plenty of all three, and the reason is structural: **there is no lifecycle manager**. Startup
order is encoded in the `sleep` offsets of `config/sessions/*.yaml`, every node is active the moment it comes up,
and so the question "is the whole stack healthy right now" has **no single source that can be asked**.

---

## 1. The four classes of handle in this workspace

| Handle | Location | Only way to reach it today |
|---|---|---|
| Persistent logs of 10 subsystems | `log/stack/<robot_id>/<name>/` (mapping goes under the `mapping/` subtree), multilog 16 MiB × 10 gzip rotation | Manual `tail current` / `zcat @*.s`, aligning timestamps by hand |
| Live ROS graph + TF + action servers | DDS, everything namespaced under `<robot_id>/` | The `ros2 topic/node/param/service` CLI |
| Per-robot backend REST / WS | `:3000` on each robot (the real profile is `network_mode: host`) | curl / the frontend on `:3001` |
| Orchestration state | Temporal `:7233`, task queue partitioned by `robot_id` | Temporal UI `:8081` |

One detail deserves particular attention: the `MonitorManager` in `syncai_sys_manager` **only prints memory /
disk usage to stdout; it publishes no topic** — deliberately, because `ROS_LOG_DIR` is a tmpfs and the byobu
`pipe-pane` capture is the persistent record. In other words, **the robot's resource history exists only in the
logs**, with no programmatic access path whatsoever. That fact directly decides the first priority in §3.

---

## 2. ROS tools + REST tools in the same server: this is the core value

The answer is yes, but the reason is not "supporting both is more convenient"; it is that **the two sides answer
different kinds of question**:

| | ROS tools | REST tools |
|---|---|---|
| Answer | "The physical state at **this** instant" | "The **recorded** intent and history" |
| Data | TF, costmap, action feedback, `cmd_vel` | vertices, tasks, schedules, map metadata |
| Source | DDS (volatile, high-rate, no history) | Postgres (persistent, low-rate, has ids) |
| Requires | An rclpy process on the same DDS domain | Only HTTP reachability to `:3000` |

The genuinely valuable tools are the ones that **cross that line** — a single interface cannot do it, and an agent
stitching it together on its own will get it wrong:

**Example A: map vertex reachability verification (the robot does not move at all)**

```
verify_vertices(map_name)
  REST : GET /api/v1/maps/{name}  → fetch every vertex and its coordinates
  ROS  : call the planner's ComputePathToPose action for each vertex
  ROS  : read TF map → <robot_id>/base_link to use the current position as the start
  Judge: which CHARGER / HOME points cannot be planned to right now; is any path length anomalous
```

The REST half knows "which points should exist"; the ROS half knows "whether they are reachable right now".
Together they are the single sentence "I changed the map, verify it for me" — apart, they are twenty manual
operations.

**Example B: the complete causal chain of a task failure**

```
explain_task(task_id)
  REST     : GET /api/v1/tasks/{id}        → step list and the claimed state
  Temporal : workflow history              → which activity failed, how many retries
  ROS      : the navigate_to_pose result code
  Logs     : controller / lio_bridge output for the same time window
```

Each of the four sources holds only one piece. Today, answering "why is this task stuck" means opening four
windows.

**Example C: double verification of state**

`RobotState.msg` deliberately carries two sets of fields side by side: commanded and measured. `SetPolicyMode` /
`SetMotionKey` are **one-way UDP with no ack**, while `low_level_mode` is the **measured value** read back from the
gait controller's telemetry. A tool that both sends the command (REST) and confirms the measured value followed
(ROS) turns "the command was sent" into "the command took effect" — a guarantee REST alone can never give.

⚠️ But an all-zero `low_level_mode` is **ambiguous**: it is both "no first sample received yet" and the legitimate
"PPO / Stand", and it **carries no freshness information at all**. Any tool doing this kind of confirmation must
handle the ambiguity itself (for example, first confirm that `motor_status.timestamp` is advancing) and cannot take
all-zero as a reading at face value.

---

## 3. Proposal list (ordered by value)

### Proposal 1: Log archaeology MCP ★ recommended first

**The pain is measured, not hypothetical.** Almost every failure in this stack is a cross-pane causal chain: LIO
drops → lio_bridge has no TF → controller rejects the goal → task_runner reports failure. **Four panes, four
stories**, and no reading tool at all (the old `scripts/tailog.sh` was deleted along with `byobu_session*.sh`).
Add the fact from §1 that `MonitorManager` writes only to stdout, and the logs are the **only** source of the
robot's resource history.

```
list_subsystems()                  → which panes have logs, and the time of each one's last entry
tail(subsystem, lines, level)      → automatically spans current + the @*.s rotations, transparently gunzips
grep_logs(pattern, since, until)   → search across subsystems
timeline(since, until)             → merge every pane onto a single time axis
resource_history(since)            → extract the memory / disk curves from sys_manager's pane
```

`timeline` is the core, not a convenience version of `tail`: what it does is **time alignment**, which is exactly
the step humans get wrong most easily.

- Host: the robot itself (needs the filesystem)
- Risk: **zero**, purely read-only
- Note: multilog's `@*.s` files are gzip, `current` is not; the timestamp format is decided by multilog's `t` flag

### Proposal 2: Stack doctor — startup health-check MCP

Turn the tribal knowledge in `CLAUDE.md` into executable **judgments**, rather than merely dumping the graph:

```
check_stack()   → compare the actual nodes against the expected list in start_nav.yaml
                  + is the TF chain map → <robot_id>/odom → <robot_id>/base_link complete
                  + are compute_path_to_pose / follow_path / navigate_to_pose advertised
                  + the actual publish rate of the main topics
                  + infer "which link is the most likely culprit" from the sleep ordering
check_params()  → compare each node's live params against the params YAML and list the differences
check_identity()→ do the node namespace / TF frame prefix all equal [system] robot_id
```

`check_params` targets a known trap: adding `name=` to a launch `Node` makes `planner_server` and its internal
`global_costmap` collide on the same name, and **the internal costmap silently loses all its parameters**. Today
that bug is caught only if someone remembers it. `check_identity` targets another: TF frame names are **not**
namespaced by ROS, so the launch file must override them explicitly, and a missed override fails silently.

- Host: the robot itself (needs rclpy + visibility of DDS)
- Risk: low (read-only, but mind the spin-thread problem in §5)

### Proposal 3: Fleet / container operations MCP

`docker-compose.robots.yml` already has `robot01` / `robot02` in the real profile plus three in the sim profile,
but **each robot's backend is its own `:3000`, and nothing stands above them**. That layer is naturally MCP:

```
list_robots() / robot_state(robot_id)   → aggregate each robot's :3000 (real goes over the host network + mDNS *.local)
dispatch(task, prefer=idle)             → pick a robot by battery / state (the Temporal queue is already partitioned by robot_id)
compose_up / down / logs(service)
rebuild(package)
```

It also solves a real nuisance along the way: recreating a container wipes the hand-installed build dependencies
(Sophus / GTSAM are compiled from source), and that recovery procedure can be wrapped as a tool.

- Host: **the dev machine / fleet side**, not the robot
- Risk: medium (`compose down` stops a real robot)

### Proposal 4: Temporal task archaeology MCP

`RobotWorkflow` has queries and cancellation, and REST has tasks / schedules, but the answer to "why is this task
stuck" lives in the workflow history, which today is visible only in the UI on `:8081`. List running workflows,
explain the failed activity, see the retry count, map against `StepType` (`MOVE` / `ARTIFACT` / `STANDUP` /
`LIEDOWN`).

- Host: anywhere with HTTP/gRPC reachability to `:7233`
- Risk: mostly read-only; `reset` / `terminate` belong to the "moves things" tier

### Proposal 5: Navigation parameter-tuning experiment MCP (sim only)

13 params YAML files; RPP controller plus costmap inflation is pure trial and error. It only gets interesting as an
agent loop: change a live param → send one `NavigateToPose` → collect metrics from the actual topics (path length,
elapsed time, minimum obstacle distance, `cmd_vel` jitter) → return a score.

Note that the controller **clamps linear acceleration itself**; there is no velocity smoother in the stack, so the
`cmd_vel` jitter metric reflects the controller parameters directly, with nothing in between cleaning up after it.

- Host: the robot itself or a sim container
- Risk: **high — a moving robot**. Recommend hard-restricting to the sim profile

### Proposal 6: Map production line MCP

`pgo/save_maps` → `map.pcd` + `patches/` + `poses.txt` → `syncai_backend/helpers/pcd_to_gridmap.py` →
`gridmap.pgm` / `.yaml` → vertices (`VertexType` GENERAL / ARTIFACT / CHARGER / HOME / WAITING). This line is
currently half CLI, half frontend map editor, and **the last step is offline and run manually after the fact** — so
a map directory spends a while with only a pcd and no gridmap.

The most valuable capability is the **reachability verification** of §2 example A.

- Host: the robot itself (the pcd files live there) + REST
- Risk: low (produces new files; does not overwrite existing maps)

### Proposal 7: rosbag / LIO replay MCP

The bag-recording procedure in `doc/record-lidar.md` (`/livox/lidar`, `/livox/imu`, zstd compression, 2 GB
splits) is currently hand-pasted commands. Record, list bags, inspect bag contents, rerun LIO on a bag to compare
drift. Useful for FAST-LIO2 tuning, but comparatively narrow.

One connection worth mentioning: `pgo_node` is the **only** thing that can save a map, the keyframes are all in
RAM, and if a mapping run finishes without saving, the map is gone — **the only after-the-fact remedy is replaying
a bag**. That turns bag recording from "a convenience for tuning" into "the only insurance".

---

## 4. Counter-arguments: things that should not be MCP

**"Repo convention lint" should not be an MCP.** Checking whether a subscriber hard-codes an absolute topic,
whether `use_sim_time` is overridden by launch, whether the TF frame parameters are overridden — all of this can be
done with ordinary file tools, and a skill or subagent is more direct than an MCP. The only thing worth an MCP is
**"how does this file differ from nav2 upstream"**: this is a port, half the questions are "did we change this or
is it original", and answering that needs an upstream checkout — that is genuinely external state.

**Do not build continuous `cmd_vel` teleoperation.** The UDP link in `syncai_driver_manager` is **one-way, no
ack**, and the safe-shutdown path relies on a human hitting Ctrl-C in that pane. Putting an LLM inside a 10 Hz
velocity loop is a bad idea. One-shot motion keys / nav goals are fine (they have explicit termination conditions);
continuous velocity control is not.

**Do not wrap `/api/v1/robot/state` and call it a proposal.** That is a one-to-one REST proxy; the agent can curl
it itself. If it is wrapped at all, wrap the version "with judgment": for example, check `localization_valid`,
whether `motor_status.timestamp` is advancing, and whether `low_level_mode` is meaningful at the same time, then
return a one-sentence conclusion.

---

## 5. Architecture decisions (to be settled before implementation)

**5.1 Which machine the server runs on decides what it can do**

The three hosting requirements are mutually exclusive, and forcing them into one process would be ugly:

| Proposal | Needs |
|---|---|
| 1, 2, 6 | The robot's filesystem / processes / DDS |
| 3 | Cross-robot visibility + the docker socket |
| 4 | Only reachability to Temporal |

Recommendation: **start a new server doing only proposals 1 + 2** (same host, read-only, fewest dependencies), and
once it runs smoothly decide whether 3 should become a standalone fleet-side server.

**5.2 The threading relationship between rclpy and MCP**

The existing pattern is `rclpy.spin()` owning the main thread with FastMCP on a background daemon thread. Every new
ROS tool must obey the same rule: **a tool function must not block spin**. Waiting on an action result, waiting on
a service response, `wait_for_message` and the like must go through their own callback group, otherwise they
deadlock the very callback they are waiting on. This is the trap proposals 2 and 5 are most likely to fall into.

**5.3 Tools must be tiered, and the tier must appear in the name**

```
read-only     : list_*, get_*, check_*, explain_*, tail, timeline
mutating      : set_*, dispatch, create_*        → requires confirmation
destructive   : compose_down, kill_session, delete_*  → not exposed by default
```

The `'4'` (ESTOP) of `/api/v1/robot/set_motion_key` is a ready-made lesson: the schema accepts it, but the backend
**deliberately does not forward it**. The tool layer must replicate that explicit refusal rather than silently
letting it through.

**5.4 `robot_id` is an implicit parameter of every tool**

A single DDS domain can host several robots. A per-robot server resolves it once from `config/system.ini` and is
done; a fleet-side server (proposal 3) **must take `robot_id` explicitly in every tool**, and must never guess.

---

## 6. Relationship to the existing `syncai_ros_mcp`

The existing server is a **generic ROS graph reflection layer**: `get_topics` / `get_services` / `publish_once` /
`subscribe_once` / `call_service`, plus thin REST wrappers for vertices and tasks. It answers "what is in this
graph".

The proposals in this document answer **"what is wrong right now"** and **"is this right"**. The difference is
judgment, not data — so this is not a replacement relationship. The criterion at implementation time: if a tool
merely dumps some interface, it belongs to the existing server; if it needs cross-source correlation, or needs to
encode tribal knowledge into a conclusion, it belongs to the new one.

---

## 7. Suggested landing order

1. **Proposal 1** (log archaeology) — read-only, zero risk, pays back the same day, and restores a tool that was
   already deleted
2. **Proposal 2** (stack doctor) — same host, turns the knowledge in `CLAUDE.md` into executable judgments
3. Observe what the agent actually solves with those two, then decide which of **3 / 4** comes first
4. **Proposal 5** (tuning) last, and only against sim

---

## 8. Agent wiring: deepagents (LangChain)

Everything above is about "what the tools look like"; this section answers "who calls them". Conclusion:
**interactive diagnosis needs no code at all** — Claude Code is itself an MCP client:

```bash
claude mcp add --transport http syncai http://robot01.local:8000/mcp
```

The scenario where deepagents is worth writing is the **embedded / automated agent**: scheduled health checks,
conversational diagnosis built into the operator console, automatic fault analysis triggered by the backend. First
settle "who calls this agent, and when", then decide whether to raise a harness of your own.

### 8.1 Why deepagents

Three reasons, each mapping directly onto a design already in this document:

1. **MCP wiring is one line of config.** deepagents consumes MCP tools through `MultiServerMCPClient` from
   `langchain-mcp-adapters`, with streamable HTTP support — `syncai_ros_mcp` is exactly FastMCP over HTTP (port
   8000), a direct match.
2. **`interrupt_on` is a direct landing of the tool tiers in §5.3.** The read-only / mutating / destructive
   three-tier scheme can be written declaratively as an approval policy, with no hand-written gate.
3. **planning + subagents fit the task shape of proposals 1 and 2.** "Why is the task stuck" is a multi-step
   causal chain across logs / DDS / Temporal, which is exactly what its todo-planning is for; its virtual
   filesystem can also absorb large chunks of log output without blowing up the main context.

The architecturally clean point: the agent process speaks HTTP to `:8000` only and **never touches rclpy**, so
the spin-thread problem of §5.2 does not concern it — that is always the MCP server's responsibility. The agent can
run on a dev machine or anywhere on the fleet side that can reach the robot.

### 8.2 Minimal wiring

```python
import asyncio
from langchain_mcp_adapters.client import MultiServerMCPClient
from langgraph.checkpoint.memory import MemorySaver
from deepagents import create_deep_agent

async def main():
    client = MultiServerMCPClient({
        "robot01": {
            "transport": "http",
            "url": "http://robot01.local:8000/mcp",  # the container already resolves mDNS
        },
    })
    tools = await client.get_tools()

    agent = create_deep_agent(
        model="anthropic:claude-opus-5",
        tools=tools,
        system_prompt="You are the operations assistant for the SyncAI quadruped robot...",
        interrupt_on=INTERRUPT_POLICY,   # see 8.3
        checkpointer=MemorySaver(),      # required for HITL; switch to a Postgres checkpointer in production
    )
    result = await agent.ainvoke(
        {"messages": [{"role": "user", "content": "Check robot01's current navigation state"}]},
        config={"configurable": {"thread_id": "1"}},
    )

asyncio.run(main())
```

The model string is `anthropic:claude-opus-5`. In production the checkpointer must be swapped for a persistent
Postgres checkpointer — this stack conveniently already has Postgres (`:5432`).

### 8.3 Tool tiers → `interrupt_on` policy

A direct translation of the tiers in §5.3:

```python
INTERRUPT_POLICY = {
    # read-only: do not intercept
    "get_topics": False, "tail": False, "timeline": False, "check_stack": False,
    # mutating: requires human approval
    "create_task":    {"allowed_decisions": ["approve", "reject"]},
    "set_motion_key": {"allowed_decisions": ["approve", "edit", "reject"]},
    # destructive: approval-gated, or simply not exposed
    "switch_mode":    {"allowed_decisions": ["approve", "reject"]},
}
```

Advanced: the `when` predicate (langchain ≥ 1.3.3) allows conditional interception, e.g. intercept only those
`set_motion_key` calls whose key is a dangerous value and let the rest through.

⚠️ **`interrupt_on` is UX, not a safety boundary.** The interception happens on the agent client side; anything
that hits `:8000` directly bypasses it. The correct place for the lesson in §5.3 (the backend deliberately not
forwarding ESTOP `'4'`) is still the **server side** — the deepagents approval mechanism is a second layer stacked
on top, not a replacement.

### 8.4 The multi-robot trap

When `MultiServerMCPClient` mounts robot01 / robot02 at the same time, both sides' `get_topics` and `tail`
**collide on name**. The least-effort fix is **one agent instance per robot** — consistent with the philosophy of
partitioning the Temporal task queue by `robot_id`. If a single agent must manage the whole fleet, fall back to
§5.4: a fleet-side server where every tool takes `robot_id` explicitly, rather than stacking several per-robot
servers inside one client.

### 8.5 Fit with the landing order

This pairs naturally with the order in §7: proposals 1 and 2 are all read-only tools, so build the agent first with
everything at `interrupt_on: False` and run it at zero risk to validate the value; by the time mutating tools
(create_task, switch_mode) are to be exposed, the checkpointer + interrupt skeleton is already in place and it is
only a few lines of tier configuration. Note that the HITL approve / reject flow needs a UI endpoint — the operator
console (the Next.js frontend) is the natural home, and that is one thing to do before wiring up mutating tools.
