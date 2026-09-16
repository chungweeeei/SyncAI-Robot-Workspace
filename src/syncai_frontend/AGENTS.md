<!-- BEGIN:nextjs-agent-rules -->
# This is NOT the Next.js you know

This version has breaking changes — APIs, conventions, and file structure may all differ from your training data. Read the relevant guide in `node_modules/next/dist/docs/` before writing any code. Heed deprecation notices.
<!-- END:nextjs-agent-rules -->

# Project notes (syncai_frontend)

Operator console for `syncai_backend` (REST/WS on port 3000). Dev and prod
servers both run on port **3001** (`package.json`). Full orientation in
`README.md`.

**Layer map.** `app/` route shells ("chrome only" — a component owns each page)
→ `components/` (`console/` shell + providers, `dashboard/`, `mapping/`,
`maps/`, `tasks/`, `settings/`, `ui/` shadcn primitives on `@base-ui/react`) →
`hooks/` (one per backend interaction) → `lib/api/` (typed fetchers, `config.ts`,
`query-keys.ts`) and `lib/ros/` (WebSocket clients + frame decoders).

**Backend addressing.** All backend paths go through `apiUrl()` / `wsUrl()`
from `lib/api/config.ts`, never a literal host — the fallback is the page's own
hostname on port 3000, and `NEXT_PUBLIC_API_BASE` / `NEXT_PUBLIC_WS_BASE`
override it.

**Query keys.** REST reads use TanStack Query (client in
`components/query-provider.tsx`: retry and focus-refetch are off — poll
intervals are the retry policy). Every key lives in `lib/api/query-keys.ts`,
whose header states the rule:

> Keys for the TanStack Query cache, centralised so cache *sharing* is a
> decision made in one visible place. The interesting entry is mapVertices:
> useMapVertices (the gridmap editor) and useActiveMapVertices (the dashboard
> and task screens) deliberately read the same key, which is what makes a
> vertex placed or moved on one screen already current on the other — with
> neither hook knowing the other exists. Keep any new key here rather than
> inline in its hook, or that property quietly stops being checkable.

WebSocket streams (telemetry, point cloud, teleop) stay outside TanStack; the
point cloud bypasses React state entirely and writes into three.js buffers in
`components/dashboard/pointcloud-canvas.tsx`. Raw three.js — no
react-three-fiber.

**Design.** `.claude/skills/frontend-design/SKILL.md` is the visual-design
guidance for new or reshaped UI in this package; load it before building
screens.
