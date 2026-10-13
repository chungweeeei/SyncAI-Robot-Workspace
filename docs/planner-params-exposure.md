# Exposing global-planner parameters to the backend — proposal

Status: **proposal** (2026-10-14), nothing implemented. Scope: which
`planner_server` parameters the backend should let an operator set, starting
with the path planner. Values are those in
`src/syncai_global_planner/params/planner_server_params.yaml` at `dev`
`db38672`.

## Why the inflation layer is in scope

The configured planner is `SmacPlanner2D` (id `GridBased`), but it does not
decide alone where a path runs. Smac pays `1 + cost_travel_multiplier *
cost / 252` per metre, and the cost it pays is the **global costmap's
inflation band**: in open space the path rides that band's edge, so
`inflation_radius` is, directly, the wall clearance. Exposing Smac's knobs
without the inflation layer's would give the operator a "clearance" setting
that cannot move the path off a wall. The two belong in one settings group.

## First version: five parameters

All five are handled by an existing dynamic-parameter callback
(`SmacPlanner2D::dynamicParametersCallback`, `InflationLayer::dynamicParametersCallback`),
so a `SetParameters` call takes effect on the next plan with no restart.

| Node | Parameter | Today | Effect | Bounds |
|---|---|---|---|---|
| `planner_server` | `GridBased.cost_travel_multiplier` | 1.0 | How hard the path is pulled to the centre of a corridor. Higher: more centred, longer routes, more bowing in open space (2.0 bowed visibly). Lower: straighter, closer to the band edge. 0 is plain distance A*. | ≥ 0; a UI range of 0–3 is plenty |
| `planner_server` | `GridBased.tolerance` | 0.5 m | How far from a blocked goal the planner may end instead. | ≥ 0; above ~1 m a MOVE "succeeds" visibly short of its target |
| `planner_server` | `GridBased.allow_unknown` | false | Whether a path may cross unknown cells of the gridmap. | bool. `true` also lets the smoother move waypoints onto unknown cells (it follows the same flag) |
| `global_costmap` | `inflation_layer.inflation_radius` | 0.8 m | Robot-centre-to-wall distance in open space; passages narrower than twice this cost more per metre. | **≥ 0.455 m** (the padded footprint's circumscribed radius) — below that the inscribed core loses meaning; see the footprint comment in the params file |
| `global_costmap` | `inflation_layer.cost_scaling_factor` | 1.5 | Decay of the band: how strongly a narrow passage pulls the path to its centre (at 1.5 the band edge is still ~42 % of lethal). | > 0; higher fades the band faster and weakens the centring |

The backend should enforce the bounds itself: neither callback rejects an
out-of-range value today (Smac's accepts anything; the inflation layer's
only compares against the current value).

## Not in the first version

**Advanced** — usable, but an operator cannot tell what a change did:

| Parameter | Today | Dynamic | Why not yet |
|---|---|---|---|
| `GridBased.smoother.w_smooth` / `w_data` | 0.4 / 0.1 | **no** | Corner roundness. Read once in `SmootherParams::get()`; the callback ignores `smoother.*`. The smoother has no cost term, so raising `w_smooth` cuts corners toward walls — keep it well below ~0.5 regardless. |
| `GridBased.smoother.max_iterations` / `do_refinement` / `tolerance` | 1000 / true / 1e-10 | no | Convergence internals. |
| `GridBased.max_planning_time` | 2.0 s (code default) | yes | Search time budget. |
| `GridBased.max_iterations` / `max_on_approach_iterations` | 1 000 000 / 1000 (code defaults) | yes | Search budget. |
| `GridBased.use_final_approach_orientation` | false | yes | Final pose heading: the goal's (false) or the path's last segment (true). |
| `is_path_valid.footprint_check` / `heading_margin` | true / 0.35 rad | yes | Replan trigger; an A/B switch for debugging, not a site setting. |

**Never exposed:**

- `downsample_costmap` / `downsampling_factor` — pointless at 0.05 m, and
  turning it off at runtime does not work: the callback only builds a
  downsampler, never drops one, so the planner keeps downsampling
  (`plugins/smac_planner/smac_planner_2d.cpp`, the `reinit_downsampler`
  branch).
- `planner_plugins` / `GridBased.plugin` — `move.xml` hardcodes
  `planner_id="GridBased"`.
- `footprint` / `footprint_padding` — one robot, one footprint shared with
  the controller's local costmap, and the global padding must stay larger
  than the local one.

## Open before the backend wires this up

1. **Persistence.** A `SetParameters` value is lost when `switch_mode` or
   `restart_mode` rebuilds the nav session. Either the backend stores the
   values and re-applies them once the nav session is up, or it writes an
   override params file the planner launch loads. The second makes that
   file part of the contract with the backend (like `robotNN.ini`), so it
   needs its own path and format decision.
2. **Keepout band does not follow the inflation parameters.**
   `KeepoutFilter::inflateMask()` reads `inflation_radius` /
   `cost_scaling_factor` off the `InflationLayer`, but only when a mask
   arrives or the footprint changes. After a runtime change, walls get the
   new band and keepout zones keep the old one until
   `filter_mask_server/load_map` is called again. Fix here (have the
   inflation layer's callback trigger a re-inflation of the mask) before
   exposing the two inflation parameters, or have the backend reload the
   mask after setting them.
3. **Smoother parameters at runtime.** Only needed if the advanced group is
   exposed later: the callback would have to rebuild `_smoother` from the
   new `SmootherParams` under `_mutex`.
