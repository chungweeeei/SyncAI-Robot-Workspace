# Task Self-Healing Loop Proposal: Fail → Attribute → Tune → Retry

> Target: `src/syncai_device_agent/` (the deepagents runtime) + the Temporal `RobotWorkflow` in
>       `syncai_backend`
> Related: `doc/deep-agent-proposal.md` (agent-side wiring / the tool vs skill criteria, which this document follows)
>       `doc/mcp-server-proposals.md` §2 example B (the four-source causal chain of `explain_task`)
>       `doc/gridmap-tuning-agent-proposal.md` (the offline, safe version of the same loop; build that one first)
> Status: **proposal, not implemented**. Neither of the two tools required (failure context / parameter
>       read-write) exists yet in `syncai_ros_mcp`; see §2.

*Note: `src/syncai_device_agent/` was removed from the workspace in commit 99141a6; this proposal predates that removal.*

This note answers one question: is the loop "after a task fails, have the agent read the logs, analyse, adjust
parameters, and run again" achievable?

The conclusion up front: **it is, but the version that succeeds is not "an agent that runs its own loop".** The
loop belongs to Temporal, the judgment belongs to the agent, and the authority to act belongs to a whitelist — only
with the three separated does this problem hold together. And the genuinely hard part is not the agent framework;
it is **attribution**: most task failures are not something parameter tuning can fix at all, and an agent that
reflexively tunes parameters will, in those cases, break the parameters while the problem remains.

---

## 0. Mental model: the agent does not own the loop

The version most easily written wrong is letting the agent run its own while loop:

```
❌ agent: read logs → tune → rerun → fails → read again → tune again → …
```

An LLM is an unreliable state machine: it retries indefinitely, forgets what it has already tried, the whole loop
dies when the process dies, and not a single step is auditable.

The correct split keeps control in Temporal — `RobotWorkflow` already does this (runs steps in order, dispatches by
`StepType`, exposes a workflow query, supports cancellation, task queue partitioned by `robot_id`):

```
Temporal RobotWorkflow  ←── owns the loop: retry count, backoff, timeouts, state persistence, cancellable
        │
        │  step fails → call one activity
        ▼
   DiagnoseActivity ──→ deepagent (a single call, not a loop)
        │                input: structured failure context
        │                output: one structured decision
        ▼
   { action: "retry_as_is" | "adjust_param" | "abort_and_escalate", … }
```

**The agent is only a decision node: called once, gives one judgment, done.** That is what yields a retry cap, a
complete audit record of every attempt, mid-course cancellation, and a workflow that is not corrupted when the agent
dies.

This also matches the layering of `deep-agent-proposal.md` §0: the agent process speaks only HTTP, touches no rclpy,
and does not enter the DDS domain, so it can run on the fleet side and be invoked over HTTP by a Temporal activity.

---

## 1. The first thing to get right is attribution, not tuning

This is the core of the whole proposal. Task failures fall roughly into five classes, and **only the last one is
where parameter tuning means anything**:

| Class | Typical form in this stack | Does tuning help | Correct action |
|---|---|---|---|
| Transient environment | Path blocked by a person, door closed, obstacle lingering in the costmap | ❌ | Back off and retry as-is; **do not touch parameters** |
| Localization | LIO drift, jumping `map → odom` correction, initial pose never set (`UNINITIALIZED`) | ❌ | Relocalize / reset the initial pose |
| Hardware | `syncai_driver_manager` UDP telemetry dropped, battery <20% (`WARNING`) | ❌ | Stop and escalate to a human |
| Map / task configuration | Target vertex lies inside the keepout filter, or is simply unreachable | ❌ | Change the task or the map, not the parameters |
| **Genuinely parameters** | Inflation too thick so no path through a narrow corridor; lookahead causes corner overshoot; goal checker tolerance too strict so `FollowPath` times out | ✅ | Fine-tune a single parameter and retry |

⚠️ If the agent reflexively "tunes a parameter and tries again" for every kind of failure, in the first four classes
it will **break the parameters while the original problem remains**. That is worse than not fixing anything,
because the site now has an extra variable nobody knows about.

So this agent's most important capability is not "can tune" but **can classify, and dares to say "I should not
touch this"**. Most of the skill / system prompt should be spent on "when not to touch parameters"; the default
output should lean toward `retry_as_is` or `abort_and_escalate`, with `adjust_param` the exception that must be
argued for.

The piece of tribal knowledge mentioned in `mcp-server-proposals.md` §2 example B (an all-zero `low_level_mode` is
ambiguous; confirm `motor_status.timestamp` is advancing first) belongs at exactly this layer — it is part of the
SOP, not of any single tool.

---

## 2. The gap: neither class of tool exists yet

The tools `syncai_ros_mcp` has today come in four groups: topic / service / task / map (`get_topics`,
`get_topic_details`, `publish_once`, `subscribe_once`, `get_services`, `call_service`, `create_task`,
`get_task_state`, `cancel_task`, maps). Of the two classes this loop needs, **it has neither**:

| Need | Current state | Suggested approach |
|---|---|---|
| Read failure context | ❌ Only `get_task_state`, which gives the state but not the cause of death | `get_task_failure(task_id)`, see below |
| Read / write parameters | ❌ Only hitting `/set_parameters` raw via `call_service`; awkward and unsafe for the agent | `get_params(node)` / `set_param(...)`, see §3 |

### 2.1 Use Temporal history first, not log parsing

Intuition says "read the logs", but logs should be the **second layer**. The first layer is Temporal's workflow
history — that is **structured failure data**: which step, what `StepType`, which retry, when it failed. Letting the
LLM grep the 16 MiB × 10 gzip-rotated multilog under `log/stack/<robot_id>/<name>/` is expensive and inaccurate,
and the output will blow up the context outright.

Suggested tool shape:

```
get_task_failure(task_id) -> {
  failed_step: {index, type, target_vertex, started_at, failed_at},
  attempt: 1,                       # how many times the workflow has already retried
  nav_result_code: ...,             # MOVE steps only
  robot_state_at_failure: {...},    # battery / state / whether the pose was valid
  log_window: {node, from, to},     # ← coordinates only, no content
}
```

`log_window` returns only "where to go digging"; the agent calls `tail_logs` for the actual text when it judges
that necessary. This is progressive disclosure, combined with deepagents' built-in filesystem tools to spill large
output to files instead of context.

Relationship to `mcp-server-proposals.md` §2 example B: that `explain_task` is a **narrative for humans**; this
`get_task_failure` is the **machine-readable version for the workflow to consume**. They share the same data
sources and are worth implementing together with shared internal functions, but the output shapes differ — do not
merge them into one tool.

---

## 3. Which parameters can actually be changed dynamically (verified)

The good news is that this stack's dynamic-parameter support is more complete than expected. Source-code
verification results:

| Node / plugin | Dynamic parameters | Evidence |
|---|---|---|
| Regulated Pure Pursuit | ✅ 22 of them | `plugins/regulated_pure_pursuit_controller/…cpp:210` |
| controller_server itself | ✅ | `src/controller_server.cpp:205` |
| goal checker / progress checker | ✅ all four plugins | `plugins/*_goal_checker.cpp`, `*_progress_checker.cpp` |
| costmap (including obstacle / inflation / static layers) | ✅ | `costmap_2d_ros.cpp:274` and each layer |
| smac_planner_2d | ✅ | `plugins/smac_planner/smac_planner_2d.cpp` |
| **ROS parameters of `syncai_backend`** | ❌ **requires a restart** | CLAUDE.md: "Changing a backend ROS parameter requires restarting the backend" |

⚠️ **But "dynamically tunable" is not the same as "should be opened to the agent".** RPP's dynamic list includes
`desired_linear_vel`, `max_linear_accel`, `max_angular_accel` — precisely because they can be changed, they must be
**explicitly excluded** from the whitelist. These directly determine the kinetic energy of a quadruped robot;
humans only.

Suggested first-version whitelist (conservative; better too narrow):

| Parameter | Bounds | Applicable failure form |
|---|---|---|
| `inflation_layer.inflation_radius` | Interval derived from the body dimensions | No path through a narrow corridor |
| `<goal_checker>.xy_goal_tolerance` | Upper bound locked | `FollowPath` times out because the arrival check is too strict |
| `<goal_checker>.yaw_goal_tolerance` | Upper bound locked | Same as above |
| `RPP.lookahead_dist` / `min_` / `max_` | Narrow interval | Corner overshoot / hugging walls |

`desired_linear_vel`, `max_*_accel`, `allow_reversing`, and any costmap topic / frame parameter: **never opened**.

---

## 4. Guardrails (six of them, from the first version onward)

1. **Whitelist**: only the parameters tabulated in §3 may change; everything else is refused. The gatekeeping logic
   lives **inside** the `set_param` tool, not in the prompt.
2. **Range bounds**: every parameter carries a min/max; out-of-range is refused outright and reported.
3. **One parameter per change**: otherwise nothing can be attributed when something goes wrong.
4. **Retry cap**: at most 2 automatic attempts for the same task, then always `abort_and_escalate`. Temporal
   enforces this; it does not rely on the agent's self-discipline.
5. **Mandatory rollback**: when a task ends (success or abandonment), parameters are always restored. **An agent's
   temporary adjustment is never allowed to remain permanently in the system** — otherwise three months later
   nobody knows why the site's parameters differ from the repo's.
6. **Full audit trail**: every `adjust_param` writes one record (task, failure reason, parameter, before/after
   values, outcome); it is the only basis for reviewing later how accurate the agent's judgments were.

⚠️ Carrying over the warning from `deep-agent-proposal.md` §4: `interrupt_on` is UX, not a safety boundary. The
correct place for guardrails 1 and 2 is **inside the `set_param` tool on the MCP server side**; the agent-side
approval mechanism is a second layer stacked on top.

---

## 5. The agent's output must be structured

deepagents' `response_format` can enforce an output schema; do not let it return a paragraph of prose for the
workflow to parse:

```python
class RecoveryDecision(BaseModel):
    action: Literal["retry_as_is", "adjust_param", "abort_and_escalate"]
    category: Literal["transient", "localization", "hardware", "map_or_task", "tuning"]
    reason: str                      # one sentence for humans
    changes: list[ParamChange] = []  # non-empty only when action == adjust_param
    confidence: Literal["low", "medium", "high"]
```

The `category` field is not only for humans — it lets you count afterwards "how many transients did the agent
misclassify as tuning", which is the key number for deciding whether to enter Phase 2 in §6.

---

## 6. Three-phase landing

| Phase | What it does | Risk | Exit condition |
|---|---|---|---|
| **Phase 0** | Diagnose only. On failure, produce an analysis + "what I would have done", **without executing**. Run for two weeks | Zero | Enough cases accumulated that the misclassification rate can be quantified |
| **Phase 1** | Recommend + human approval. `set_param` behind `interrupt_on`; a human in the loop presses confirm | Low | N consecutive approvals are all "agree" |
| **Phase 2** | Automatic within the whitelist, all six guardrails on | Medium | — |

The real output of Phase 0 is not repaired tasks; it is **data**: you learn whether its attribution is accurate and
which failure classes it misjudges. Letting it change parameters without that confidence is gambling.

Phase 1 needs the HITL approve/reject UI — `deep-agent-proposal.md` §4 already identifies the operator console
(Next.js frontend `:3001`) as the natural home. That is the shared prerequisite before wiring up any mutating tool.

---

## 7. Relationship to the other proposals

| Document | Answers |
|---|---|
| `mcp-server-proposals.md` | What went wrong / is this right (diagnostic plane, MCP server layer) |
| `roboneuron-application-proposal.md` | How the robot is treated as a set of typed capabilities (control plane, MCP server layer) |
| `deep-agent-proposal.md` | Who calls these tools, how tools vs skills are split (agent layer, **wiring**) |
| **This document** | **What the agent does with those tools: a self-healing task loop (agent layer, application)** |
| `gridmap-tuning-agent-proposal.md` | The offline version of the same loop shape, zero physical risk |

In terms of implementation order, this proposal is **not recommended as the first agent to build**. Its loop shape
(execute → measure → attribute → tune → retry → converge or give up) is exactly isomorphic to
`gridmap-tuning-agent-proposal.md`; the only difference is that "execute" there means running an offline projection
once, and here it means a quadruped robot walking out the door. **Rehearse the skeleton, convergence condition,
give-up condition, and whitelist mechanism on the offline problem first**; when it is carried over, only the safety
questions remain to worry about, not the architecture at the same time.
