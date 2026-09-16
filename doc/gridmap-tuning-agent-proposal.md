# Point Cloud → Gridmap Parameter-Tuning Agent Proposal

> Target: the offline map production line (the `[map]` pcd / gridmap paths in `config/system.ini`)
>       + `src/syncai_device_agent/` (the deepagents runtime)
> Related: `doc/task-recovery-agent-proposal.md` (the same loop shape, but that one makes the robot move)
>       `doc/deep-agent-proposal.md` (agent-side wiring / the tool vs skill criteria)
> Status: **proposal, not implemented**. §6 lists two things that must be confirmed first.

*Note: `src/syncai_device_agent/` was removed from the workspace in commit 99141a6; this proposal predates that removal.*

This note answers one question: is flattening a 3D point cloud into a 2D gridmap a good fit for an agent?

The conclusion up front: **the projection itself is not; choosing the parameters is.** And because this problem is
offline, measurable, and carries zero physical risk, it is the safe practice ground for the loop in
`task-recovery-agent-proposal.md` — **the recommendation is to make this the first real agent loop in this
workspace**, ahead of anything that makes the robot move.

---

## 0. Mental model: a single deterministic operation does not need an agent

```
❌ agent → convert_pcd_to_gridmap(input.pcd) → done
```

That is a script, not an agent. The only thing the LLM does in the middle is "decide which function to call", and
there is no choice to make. **Wrapping a single deterministic operation as a tool and putting an agent around it only
adds an expensive, slow shell.** This is the same argument as `mcp-server-proposals.md` §0: wrapping REST endpoints
one-to-one as MCP tools is the least valuable thing you can do.

What has value is the **parameters**. Z-slice height, resolution, occupancy threshold, outlier-removal radius…
there is no universal set of values for these; every site is different:

| Parameter off | Consequence |
|---|---|
| z lower bound too low | Floor noise is treated as obstacles; the whole map gets dirty |
| z upper bound too high | Tables, pallets, low platforms vanish from the map; the robot will hit them |
| Filter radius too large | Thin pillars and table legs get eaten |
| Occupancy threshold too loose | Walls have holes; both AMCL matching and the costmap leak |

"Run once → look at the result → judge what is wrong → adjust parameters → run again" — **that is the shape of an
agent loop**, and the judging step genuinely requires judgment, not a table lookup.

---

## 1. Current state: the judgment lives in a human

The frontend has a whole set of manual grid-editing tools: `components/maps/map-grid-editor.tsx`, `grid-canvas.tsx`,
`grid-toolbar.tsx`, together with `app/maps/[name]/edit/page.tsx`. In other words, the current flow is:

```
FAST-LIO2 / PGO / HBA → .pcd → (projection) → gridmap → a human eyeballs it → paints by hand or changes parameters and reruns
```

This loop runs once per map build, has to be redone for every site, and the basis for the judgments is mostly never
recorded — hand it to a different person and the experience has to be accumulated all over again. That is exactly
the shape worth handing to an agent: **repetitive, requires judgment, but the judgment has an objective basis**.

---

## 2. Why this is the best first agent loop

**(a) Zero physical risk.** Pure file processing; the robot does not move. If it goes wrong, rerun — the cost is CPU
time. You can afford to loosen permissions and watch what it does, which is impossible on a real robot.

**(b) There are objective evaluation metrics.** This is what decides whether the loop can exist at all; see §3. The
agent must be able to self-assess "was this attempt better or worse than the last one", otherwise it is just
thrashing.

**(c) It naturally covers all of deepagents' core concepts.** tools / backend / permissions are all necessary in
this problem rather than contrived; see §4.

**(d) It is the safe isomorph of `task-recovery-agent-proposal.md`.** The two loops have exactly the same shape:

```
execute → measure → attribute → tune → retry → converge or give up
```

The only difference is whether "execute" means running an offline projection once, or a quadruped walking out the
door. **The skeleton, convergence condition, give-up condition, and parameter whitelist can all be rehearsed here
and then carried over.**

---

## 3. The evaluation metrics decide whether the loop holds

Without quantitative metrics the agent can only "tell stories about a picture", and the loop will not converge.
Suggested tool:

```
evaluate_gridmap(path) -> {
  unknown_ratio,        # share of -1 cells
  free_connectivity,    # number of free-space connected components / share of the largest: are corridors that should connect actually connected
  wall_breaks,          # number of wall breaks
  wall_thickness,       # wall-thickness distribution (too thick = inflation or z range too wide)
  speckle_count,        # number of isolated occupied cells (noise indicator)
  diff_vs_reference,    # difference against the previous map version / CAD (optional)
}
```

⚠️ **The primary signal must be numeric; images are supporting evidence.** The model can read a pgm, and feeding
the rendered image alongside helps with "where does it look wrong", but **do not let visual judgment be the sole
basis** — it is unstable, and it cannot be used to rank two attempts. Metrics drive convergence; images explain.

The metrics pull against each other (lowering unknown usually raises speckle), so the skill has to spell out the
priority order, for example: connectivity > wall integrity > noise > unknown-cell share. That priority order is
tribal knowledge and belongs in the SOP.

---

## 4. Configuration: what each of the three concepts maps to

| Concept | How it is used here |
|---|---|
| **tools** | `run_projection(params)`, `evaluate_gridmap(path)`, `diff_maps(a, b)` |
| **backend** | `CompositeBackend`: default `StateBackend` (intermediate data) + `FilesystemBackend` pointed at the working directory (the real .pcd / .pgm) + `/memories/` via `StoreBackend` |
| **permissions** | `deny` overwriting `/input/**`; `deny` touching `config/`; `interrupt` on writes to `/approved/**` |
| (not needed) | sandbox. Unless the projection has to run open3d / PCL and you do not want it installed locally |

`/memories/` is the most valuable by-product of this problem: it accumulates site-level knowledge such as "the B1
warehouse site uses z=[0.15, 1.2]". After three sites, the fourth run has precedents to consult instead of guessing
from zero — that is what `StoreBackend` genuinely earning its place looks like, far more useful than remembering a
user's name.

---

## 5. Directory layout and guardrails

```
working directory (FilesystemBackend root)
├── input/site_b1.pcd        ← permissions: read-only. Regeneration cost is a full re-mapping run
├── candidates/              ← agent writes freely
│   ├── attempt_01.{pgm,params.json,metrics.json}
│   └── attempt_02.…
└── approved/                ← permissions: interrupt; requires human approval before writing
```

First-version task statement:

> Produce a usable 2D gridmap from `/input/site_b1.pcd`. For every attempt, save the parameters and evaluation
> metrics into `/candidates/`, at most 5 attempts. If it converges, report which set is best and why; if it does
> not, say what you tried and where you got stuck.

Guardrails (the same set of habits as `task-recovery-agent-proposal.md` §4, deliberately kept consistent):

1. **Attempt cap of 5** — otherwise it will keep tuning forever.
2. **Parameter whitelist + ranges** — e.g. limit the z-slice to 0–2 m; do not let it hand back physically absurd
   values.
3. **Raw data read-only** — the `.pcd` cannot be overwritten; that is something only a full re-mapping run can
   bring back.
4. **Output isolation** — the agent writes only to `/candidates/`; anything entering `/approved/` requires human
   approval.
5. **Every attempt leaves a trace** — params + metrics are saved as a pair, otherwise attempts cannot be compared
   and there is nothing to review afterwards.

---

## 6. To be confirmed

Two things need clarifying before implementation (not verified in this document):

1. **Who currently does `.pcd → gridmap`?** Possibly the `FASTLIO2_ROS2` submodule
   (`chungweeeei/SyncAI-Fast-LIO2`, containing LIO + PGO + HBA + localizer), `scripts/`, or the map manager in
   `syncai_sys_manager`.
2. **Can it be run from the CLI with parameters?** This is the premise of the whole proposal — something the agent
   cannot invoke cannot be iterated on. If the parameters are currently hard-coded or buried in a GUI flow, **the
   first step is not writing an agent; it is turning that into a parameterised CLI**. That step has value in its
   own right, even if the agent is never built.

---

## 7. Relationship to the other proposals

| Document | Answers |
|---|---|
| `mcp-server-proposals.md` | What went wrong (diagnostic plane, MCP server layer) |
| `roboneuron-application-proposal.md` | How the robot is treated as typed capabilities (control plane, MCP server layer) |
| `deep-agent-proposal.md` | Who calls the tools, how tools vs skills are split (agent layer, wiring) |
| `task-recovery-agent-proposal.md` | Online task self-healing loop (agent layer, **moves the robot**) |
| **This document** | **Parameter-tuning loop for the offline map production line (agent layer, does not move the robot)** |

One structural difference is worth pointing out: this proposal **does not need the MCP server**. Its tools are
local CLI wrappers; they touch neither the ROS graph nor DDS, and do not need the robot powered on. So it can
proceed entirely independently of the landing progress of the first three documents — which is one more reason it
suits being the first exercise.
