# Deep Agent Application Proposal: Connecting the Agent to the Robot

> Target: `src/syncai_device_agent/` (the new deepagents runtime; currently only a skeleton)
> Related: `doc/mcp-server-proposals.md` §8, `doc/roboneuron-application-proposal.md` §8
>       — those two approach the problem from the **tool / server side**; this one approaches it from the **agent side**, see §7
> Framework: `deepagents==0.7.5` / `langchain==1.3.14` / `langgraph==1.2.11` (already installed in
>       `src/syncai_device_agent/.venv`)
> Status: **proposal, not implemented**. `main.py` is currently a web_search researcher example; `tools.py` is an empty file.

*Note: `src/syncai_device_agent/` was removed from the workspace in commit 99141a6; this proposal predates that removal.*

This note answers three questions that have to be settled before any implementation: whether to define tools for
the robot at all, whether a tool can "mix HTTP + ROS", and how deepagents skills should be defined.

The conclusion up front: **yes, define tools, but the tool that mixes HTTP + ROS does not live on the agent side.**
The deepagent process speaks only HTTP; all of the ROS complexity is locked away behind the MCP server (`:8000`).
That is not a preference — it is the structure that falls out of the spin-thread iron rule in
`mcp-server-proposals.md` §5.2. And **skills are not something that gets "called"**; a skill is an operating manual
that encodes a multi-step SOP into a `SKILL.md`, which is a different layer from a tool (an atomic capability).

---

## 0. Mental model: there are two layers here, do not mix them

```
┌─ deepagent process (runs on the dev machine / fleet side) ──┐
│   create_deep_agent(tools=[...], skills=[...])                │
│   ★ speaks only HTTP, never imports rclpy                     │
└──────────────────────┬────────────────────────────────────────┘
                       │  HTTP
          ┌────────────┴────────────┐
          ▼                         ▼
   MCP server :8000            backend REST :3000
   (syncai_ros_mcp, on the robot)   (syncai_backend, FastAPI + Postgres)
   ★ owns rclpy / DDS
   ★ the ROS + REST mix happens "inside a single tool at this layer"
```

`roboneuron-application-proposal.md` §8.1 already fixes this boundary: **the agent process speaks HTTP to `:8000`
only and never touches rclpy**, so the spin-thread problem of §5.2 is irrelevant to the agent — it is always the MCP
server's responsibility. That one sentence is the shared answer to all three questions in this document; everything
below merely unpacks it.

**Why does the agent side not touch ROS directly?** Technically you could of course write a `@tool` in the same
process as the agent that does `import rclpy`. But that would:

1. **Bind the agent into the DDS domain** — the agent could no longer run on the fleet side; it would have to sit on
   the robot's network segment;
2. **Force you to handle the `rclpy.spin()` thread inside the agent process** (the trap in
   `mcp-server-proposals.md` §5.2: waiting on an action / service must not block spin).

That violates clean layering. The correct approach: anything that needs to touch ROS becomes a typed tool on the
MCP server, and the agent only calls it over HTTP.

---

## 1. Three tool sources (all HTTP on the agent side)

Connecting the agent to the robot means giving it these three classes of tool. The key is to **decide first which
layer each tool lives in**.

| Source | Underlying | How it reaches the agent | Criterion |
|---|---|---|---|
| **(a) MCP tools** | May be ROS, or server-side REST / mixed | Pulled in automatically by `MultiServerMCPClient` from `langchain-mcp-adapters` | Needs to touch ROS, or needs to correlate across sources → goes here |
| **(b) Direct REST `@tool`** | Pure HTTP against `:3000` | Wrap httpx/requests yourself with `@tool` | Only queries history / intent, does not touch ROS |
| **(c) Built-in / third-party** | Depends on the tool | deepagents built-ins + `tools=[...]` | web_search, file tools, etc. |

### (a) MCP tools — not written by you, pulled in

```python
from langchain_mcp_adapters.client import MultiServerMCPClient

client = MultiServerMCPClient({
    "robot01": {"transport": "http", "url": "http://robot01.local:8000/mcp"},
})
mcp_tools = await client.get_tools()   # cmd_vel / check_stack / verify_vertices … are all in here
```

⚠️ **Dependency gap**: `pyproject.toml` currently lists only `deepagents / dotenv / requests / structlog` —
**there is no `langchain-mcp-adapters`**. Taking route (a) means adding that dependency first.

### (b) Direct REST `@tool`

`mcp-server-proposals.md` §0 is explicit about this: wrapping REST endpoints one-to-one as MCP tools is the least
valuable thing you can do — the agent can already make HTTP calls. So pure history queries should just be wrapped as
`@tool` on the agent side, with no detour through the MCP server:

```python
from langchain_core.tools import tool
import requests

@tool
def get_map_vertices(map_name: str) -> dict:
    """Read every vertex (GENERAL/ARTIFACT/CHARGER/HOME/WAITING) of the given map, with coordinates."""
    return requests.get(f"http://robot01.local:3000/api/v1/maps/{map_name}").json()
```

Then merge:

```python
agent = create_deep_agent(
    model="anthropic:claude-opus-4-8",
    tools=[*mcp_tools, get_map_vertices],
)
```

---

## 2. The "mixed HTTP + ROS" tool lives on the server side, not the agent side

This is the point most easily gotten backwards. From the agent's point of view, **every** tool it calls is HTTP
(either against `:8000` MCP or against `:3000` REST). The thing that genuinely "calls REST + calls ROS inside a
single tool" is the tool function **on the MCP server** — which is exactly what the three examples in
`mcp-server-proposals.md` §2 are:

```
verify_vertices(map_name)   # inside the server: REST fetches vertex coordinates + ROS runs ComputePathToPose for each point
explain_task(task_id)       # inside the server: REST + Temporal history + ROS result code + logs
set_locomotion_policy(p)    # inside the server: send SetPolicyMode (ROS) + read back measured values (ROS)
```

The REST half knows "which points should exist / what the claimed state is"; the ROS half knows "whether it is
reachable right now / the measured values". Only together do they add up to a sentence like "I changed the map,
verify it for me" or "why is this task stuck".

The precise split:

- the agent's tools ➜ **all HTTP** (ROS is transparent to it)
- the MCP server's tools ➜ **may be pure REST, pure ROS, or a mix of both**

So, "can a tool be pure HTTP, or mix HTTP and ROS?" — the answer is **yes**, but the mixing layer is in
`syncai_ros_mcp`, not in `syncai_device_agent`. Adding a new mixed tool means going to the registry in `roboneuron`
§2 / the locomotion section in §3, not to the agent side.

---

## 3. Skills: not "called", but progressively disclosed SOPs

First, a common misconception to correct. `create_deep_agent` in `deepagents==0.7.5` does have a `skills`
parameter, but a skill is **not** something that gets invoked the way a tool is:

| Primitive | What it is | How the agent uses it |
|---|---|---|
| **tool** | An executable function | **Calls** it, gets a return value |
| **subagent** | A child agent (triggered through the built-in `task` tool) | **Delegates** a sub-task to it |
| **skill** | A directory + one `SKILL.md` (YAML frontmatter + markdown) | **Reads it in as an operating manual** when relevant; progressive disclosure |

In one sentence: **a tool is a "capability"; a skill is the "SOP for how to chain those capabilities to get one
thing done".**

This neatly resolves a tension: the multi-step causal chains in `mcp-server-proposals.md` §2 (`explain_task` has
to stitch together four sources) would turn into a god function if crammed into one giant MCP tool; splitting them
into "atomic capability = tool" + "procedure = skill" is the clean version.

### How to define one (the real API for this version)

A skill is a **directory** containing one `SKILL.md`:

```
skills/
└── diagnose-stuck-task/
    └── SKILL.md
```

```markdown
---
name: diagnose-stuck-task
description: Use when the user asks "why is task X stuck / failed". Stitches the causal chain across REST, Temporal, ROS and logs.
allowed-tools: [get_task, get_workflow_history, tail_logs, get_nav_result]
---

# Diagnosing a stuck task

1. `get_task(task_id)` to get the step list and the claimed state.
2. `get_workflow_history` to find which activity failed and how many times it retried.
3. If the failed step is a MOVE, `get_nav_result` reads the navigate_to_pose result code.
4. `tail_logs` to grab controller / lio_bridge output for the same time window.
5. ⚠️ An all-zero low_level_mode is ambiguous (see RobotState.msg); confirm motor_status.timestamp
   is advancing before drawing a conclusion — this piece of tribal knowledge belongs in the SOP, not in any single tool.
```

The frontmatter supports `name` / `description` / `allowed-tools` (confirmed from `deepagents/middleware/
skills.py`). **`description` is the key to progressive disclosure**: normally the agent sees only that one line, and
only when it judges the skill relevant does it read the whole body into context. So no matter how many skills there
are, they cannot blow up the main context — which matters especially for tasks like "log archaeology" that spew
large amounts of output (in combination with deepagents' built-in filesystem tools).

Attaching it:

```python
agent = create_deep_agent(
    model="anthropic:claude-opus-4-8",
    tools=[*mcp_tools, get_map_vertices],
    skills=["skills/diagnose-stuck-task"],   # path resolves through the deepagents backend filesystem
    interrupt_on=INTERRUPT_POLICY,           # see §4
)
```

### Choosing between a skill and a subagent

Both can take on a multi-step task; the criterion is **whether an independent context is needed**:

- Short procedure, needs to share the main conversation's context → **skill** (it merely injects the SOP into the
  system-prompt space).
- Long procedure, produces a lot of intermediate output, context should be isolated → **subagent** (delegated
  through the `task` tool; only the conclusion comes back when it finishes). `mcp-server-proposals.md` §8.1 says
  "why is the task stuck" suits todo-planning + subagent; this is what it means.

---

## 4. Tool tiers → `interrupt_on` (copy the existing conclusion)

deepagents' `interrupt_on` is a direct landing of the three-tier scheme in `mcp-server-proposals.md` §5.3:

```python
INTERRUPT_POLICY = {
    # read-only: do not intercept
    "get_topics": False, "tail_logs": False, "check_stack": False, "verify_vertices": False,
    # mutating: requires human approval
    "create_task":    {"allowed_decisions": ["approve", "reject"]},
    "set_motion_key": {"allowed_decisions": ["approve", "edit", "reject"]},
    # destructive: approval-gated, or simply not exposed
    "switch_mode":    {"allowed_decisions": ["approve", "reject"]},
}
```

⚠️ **`interrupt_on` is UX, not a safety boundary** (§8.3 already warns about this): the interception happens on the
agent client side, and anything that hits `:8000` directly bypasses it. The correct place for an explicit refusal
such as ESTOP `'4'` is still the **server side**; the agent's approval mechanism is a second layer stacked on top,
not a replacement. HITL approve/reject needs a UI endpoint, and the operator console (the Next.js frontend) is the
natural home — that is prerequisite work before any mutating tool is wired up.

---

## 5. The multi-robot trap

When `MultiServerMCPClient` mounts robot01 / robot02 at the same time, both sides' `get_topics` and `tail_logs`
**collide on name**. The least-effort fix is **one agent instance per robot**, consistent with the philosophy of
partitioning the Temporal task queue by `robot_id`. If a single agent must manage the whole fleet, fall back to
`mcp-server-proposals.md` §5.4: a fleet-side server where every tool takes `robot_id` explicitly, rather than
stacking several per-robot servers inside one client.

---

## 6. Landing plan for `syncai_device_agent`

Current state: `main.py` is the official deepagents web_search researcher example, `tools.py` is empty, and
`pyproject` lacks `langchain-mcp-adapters`. Suggested order of growth (aligned with the landing order of the two
proposals):

1. **Fill the dependency gap**: `langchain-mcp-adapters` (for MCP tools). REST tools can use the existing
   `requests`, but for async scenarios `httpx` is recommended.
2. **tools.py**: start with only (b) pure REST `@tool`s (get_map_vertices / get_task / robot_state) — zero risk,
   and no dependency on the MCP server being up.
3. **Wire in MCP tools**: once `syncai_ros_mcp` gains the read-only tools from `roboneuron` proposals 1/2
   (check_stack / tail_logs), consume them via `MultiServerMCPClient`.
4. **First skill**: `diagnose-stuck-task` (§3), encoding the four-source causal chain of
   `mcp-server-proposals.md` §2 example B. Everything runs with `interrupt_on: False`; zero-risk validation of the
   value.
5. **Add checkpointer + interrupt**: when it is time to expose mutating tools (create_task / switch_mode), the HITL
   skeleton is in place. Switch the checkpointer to Postgres in production (this stack already has `:5432`); do not
   leave `MemorySaver` in.

The architecturally clean point: the whole of `syncai_device_agent` speaks HTTP to `:8000` / `:3000` only; it
**does not enter the robot container and does not touch DDS**, so it can run on a dev machine or anywhere on the
fleet side that can reach the robot.

---

## 7. Relationship to the other two proposals

The three documents form one line, without overlap:

| Document | Answers | Layer |
|---|---|---|
| `mcp-server-proposals.md` | What went wrong / is this right (diagnostic plane) | MCP server |
| `roboneuron-application-proposal.md` | How the agent treats the robot as a set of typed capabilities (control plane) | MCP server |
| **This document** | **Who calls these tools, and how tools vs skills are split** | **deepagent** |

Both of those documents already open the deepagents topic in their §8 (wiring, `interrupt_on`, multi-robot); this
document promotes it from an "appendix" to a standalone design and fills in what they did not develop: **the
criteria for the three tool sources, which layer the HTTP+ROS mix happens in, and the fact that skills are not
tools**. The implementation criteria are unchanged: a tool that needs to touch ROS or correlate across sources →
belongs to the MCP server; a tool that only queries history → direct REST on the agent side; a multi-step SOP →
skill; a long task needing isolated context → subagent.
