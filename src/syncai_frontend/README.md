# syncai_frontend

The operator console for one SyncAI robot. It is a browser client of
`syncai_backend` and nothing else: every byte it shows comes from that
process's REST/WebSocket surface on port **3000**, and it holds no robot state
of its own beyond what a page needs to render.

- **Next.js 16.2.10** (App Router) + **React 19**. The pinned Next has breaking
  changes relative to what models were trained on — read the relevant guide in
  `node_modules/next/dist/docs/` before writing Next.js code (see `AGENTS.md`).
- **shadcn-style UI** (`components/ui/`) built on `@base-ui/react`, Tailwind 4.
- **TanStack Query** for every REST read.
- **Raw three.js** for the 3D view — no react-three-fiber. WebRTC streaming of
  the view was considered and deferred.

Dev server and production server both listen on **3001**
(`next dev -p 3001` / `next start -p 3001` in `package.json`), so the console
and the backend can share a host without a proxy.

## Routes (`app/`)

| Route | What it is |
|---|---|
| `/` | Dashboard: `PointCloudView` (live cloud, map, robot mesh, route, vertices) left, `TelemetryRail` right. Gated on `GET /api/v1/robot/state` — no state, no panels. |
| `/mapping` | Mapping mode: switch `AUTO`/`MANUAL`, drive, watch pgo's "map so far" stream, save the map and watch its 2D grid build, or discard the run and start a new map. The page owns the confirmation rule and drives a single alert dialog from it: **every** act here is confirmed, including a plain mode switch in either direction (it tears the stack down for ~30 s and stops whatever the robot was doing), and losing an unsaved run is what escalates the copy and turns the confirm button red. |
| `/maps` | The map library (`MapLibrary`): catalogue cards, thumbnails, per-card gridmap state (and why a conversion failed), Rebuild-grid, inline Rename, Delete as an X in the card's corner (Rename and Delete are greyed on the map in use — the backend refuses them too; Delete confirms in an alert dialog that names the map and counts the vertices and megabytes going with it), and **Switch**, which is what un-greys the other two. Switch is the card's top-left corner tile, in the same slot as the in-use badge and never shown beside it: a solid check where the robot is, swap arrows on every map it could move to. (Not an unfilled check — that reads as "already done, greyed out", which is a state this control genuinely has for maps it cannot switch to.) It is a live call — it re-points the running localizer and map_server and rewrites the instance INI, no stack restart — and its alert dialog carries the one consequence an unlabelled arrow cannot: the pose resets to the new map's origin, so set an initial pose on the dashboard afterwards. All three controls hand their result sentence up to one line above the grid. |
| `/maps/[name]/edit` | The gridmap editor — replaces a step that used to be done in GIMP. Its title renames the map in place: double-click it (or press F2 on it) and Enter saves, Escape cancels — no button, and refused outright while the gridmap is dirty, since a rename moves the directory and the reload that follows would drop the buffer. Its Vertex mode also places stops. It opens on **Pan** like the grid half does — a press drags the map — and the tool row arms the other two: **Place** stages a vertex where you press (drag to aim it), **Select** drags a box over several, with Shift to add to the set and a Delete for the whole band. Escape disarms back to Pan. A stop you mark by driving to it instead comes from **Use robot position**. The robot's own footprint is drawn (to scale, `signal-live`, in both modes) wherever it is standing — both it and the button are offered only while the robot is localized on *this* map, and the button names the reason when it is not. Vertex mode also carries `ManualControl` (bottom right), so driving to the next stop does not mean leaving the editor; leaving the mode unmounts it, which closes the teleop channel and stops the robot. |
| `/recordings` | Bag recording: start a `ros2 bag record` on the robot, watch it grow, and manage what is on disk. The recorder panel has two faces rather than one with disabled fields — idle asks what to record (name, topic chips defaulting to the LIO inputs, zstd off), live reports the elapsed clock, bytes written and resolved topic names — and which face is shown comes from `GET /api/v1/recordings/active`, not from a local "we pressed Start" flag, so a recording started from a shell or a second console is shown correctly and one that died is reaped within the second. The list below is every bag on the robot, newest first, with the live one still in it; a finished bag with zero messages is called out, because nothing refuses a topic that does not exist (the recorder waits for it, so it can be armed before bringup) and a typo is otherwise silent. Delete is the row's X, behind the map library's alert dialog, and is refused for the live recording. |
| `/model-preview` | Backend-free preview of the G23 GLB inside the real canvas, for checking scale / up-axis / forward-axis of a re-baked asset. |
| `/settings` | Appearance + wifi (`nmcli` through the backend). |
| `/tasks` | Task console: template library, step composer, dispatch, schedules, the active run. |

## Layering

```
app/          route shells; almost no logic ("chrome only" — a component owns the page)
components/   console/ (shell: nav rail, status strip, shared providers), dashboard/,
              mapping/, maps/, recordings/, tasks/, settings/, ui/ (shadcn primitives)
hooks/        one hook per backend interaction (use-maps, use-active-tasks, use-teleop-sender…)
lib/api/      typed fetchers per backend router + config.ts + query-keys.ts
lib/ros/      the WebSocket clients (telemetry, point cloud, teleop) and their frame decoders
lib/robot/    G23 joint table (URDF link names ↔ GLB node names)
lib/recording/ how a bag's duration / size / message count are read, shared by the two
              recording surfaces so one quantity never appears in two spellings
```

**Backend addressing.** Every backend path goes through `apiUrl()` / `wsUrl()`
from `lib/api/config.ts` — never a literal host. Resolution order:
`NEXT_PUBLIC_API_BASE` / `NEXT_PUBLIC_WS_BASE` if set; otherwise the page's own
hostname on port 3000 (what makes `http://<robot-ip>:3001` work on the LAN);
otherwise `http://localhost:3000` for SSR / build time. The WS base is derived
from the HTTP one by swapping the scheme. Even `<img src>` for map thumbnails is
absolutised through `apiUrl`, because a relative URL would resolve against the
frontend's own origin.

**REST reads go through TanStack Query.** One `QueryClient` lives in
`components/query-provider.tsx` with `retry: false` and
`refetchOnWindowFocus: false` — the poll intervals *are* the retry policy, and
the status indicators exist to report a failure the moment it happens. Every
cache key lives in `lib/api/query-keys.ts`, so cache *sharing* between hooks is a
decision visible in one place: the gridmap editor and the dashboard read the
same `mapVertices` entry, which is what makes a vertex moved on one screen
already current on the other. Add new keys there, never inline in a hook.

**The WebSocket streams stay outside TanStack** — a push stream has nothing to
refetch. `hooks/use-telemetry.ts` (pose ~20 Hz, joints, path) and
`hooks/use-teleop-sender.ts` (outbound `{vx, vy, wz}` at ~10 Hz) are React
state; the point cloud additionally bypasses React entirely: frames
(`[u32 count][f32 xyz…]`, ~10 Hz × a few hundred KB) go straight into three.js
buffers in `components/dashboard/pointcloud-canvas.tsx`.

The shell in `app/layout.tsx` runs exactly **two polls** for the whole console
(`RobotStateProvider` at 1 Hz, `ActiveTaskProvider` at 2 s); pages read those
providers rather than polling on their own, so the header can never disagree
with the rail.

**The one self-cancelling poll is the gridmap conversion.** Saving a map starts
a pcd → gridmap conversion in a backend thread that runs for tens of seconds
after the POST has answered, and it has no push channel — the backend keeps no
job resource and a conversion finishing is not a ROS topic. So `hooks/use-maps.ts`
re-reads the catalogue every 2 s while any map reports `grid_status:
"converting"` and stops with the last one; the status is both the trigger and
the off switch. `useMapConversion(name)` is the same query entry scoped to one
map, which is how `/mapping`'s save control reports the outcome without a second
request — and it is why a failed conversion now says so on screen instead of
only in the robot's backend log.

## The robot mesh

`public/models/g23.glb` is **generated**, by `scripts/urdf2glb.py` at the
workspace root, from `src/syncai_bringup/description/G23.urdf` and its STLs.
Two invariants the canvas depends on: GLB node names **equal URDF link names**
(that is how joint angles from the telemetry stream find their mesh — see
`lib/robot/g23-joints.ts`), and the coordinates stay **Z-up** in the ROS
convention rather than glTF's nominal +Y-up, because the canvas builds a Z-up
world so map coordinates pass straight through. The script runs `gltfpack`
(meshopt compression), which the canvas decodes with `MeshoptDecoder`. Re-bake
after any URDF change; `/model-preview` is where to check the result.

## Fonts

`app/layout.tsx` loads Archivo and IBM Plex Mono through `next/font`. Archivo's
`axes: ["wdth"]` is load-bearing: without it `next/font` ships the weight-only
subset and the condensed `.instrument-label` style in `globals.css` silently
renders at normal width.

## Running

```bash
npm install
npm run dev        # http://<host>:3001, HMR
npm run build && npm start
```

On the robot, `NodeManager` starts `npm run dev` in the `frontend` window of
both session specs (`config/sessions/*.yaml`) with `cwd: src/syncai_frontend`.

**`next.config.ts` `allowedDevOrigins` hardcodes LAN IPs.** Next 16 only trusts
`localhost` for dev/HMR requests, so opening the dashboard from another origin
(the container's bridge IP, a robot's LAN address) breaks the HMR WebSocket
unless that origin is listed. Edit it per robot / network. Note the package's
`.gitignore` names `/next.config.ts`; the file is tracked anyway, so edits still
show up in `git status` — do not commit a per-site IP list by accident.

**Ports differ between package.json and the Dockerfile.** `package.json` pins
3001 for both `dev` and `start`. The `Dockerfile` (multi-stage, `output:
"standalone"`, `node server.js`) sets `PORT=3000` and `EXPOSE 3000` — the
standalone server ignores `package.json` scripts, so a container built from it
comes up on 3000, colliding with the backend if both run on one host network.
Nothing in this workspace runs that image today (the sessions use `npm run dev`);
if it is ever deployed, either pass `-e PORT=3001` or fix the Dockerfile, and
set `NEXT_PUBLIC_API_BASE` since the same-hostname fallback assumes the backend
is on 3000 of the page's host.
