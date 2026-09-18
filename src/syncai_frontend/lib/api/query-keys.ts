// Keys for the TanStack Query cache, centralised so cache *sharing* is a
// decision made in one visible place. The interesting entry is mapVertices:
// useMapVertices (the gridmap editor) and useActiveMapVertices (the dashboard
// and task screens) deliberately read the same key, which is what makes a
// vertex placed or moved on one screen already current on the other — with
// neither hook knowing the other exists. Keep any new key here rather than
// inline in its hook, or that property quietly stops being checkable.

export const queryKeys = {
  /** GET /api/v1/robot/state — the console's single 1 Hz poll. */
  robotState: ["robot-state"] as const,
  /** GET /api/v1/active_tasks — the console's single 2 s poll. */
  activeTasks: ["active-tasks"] as const,
  /** GET /api/v1/maps — the catalogue, read by every screen that needs the active map. */
  maps: ["maps"] as const,
  /** GET /api/v1/maps/<name>/vertices — one map's stops, keyed by map name. */
  mapVertices: (name: string) => ["map-vertices", name] as const,
  /** GET /api/v1/task_templates — the operator's template library. */
  taskTemplates: ["task-templates"] as const,
  /** GET /api/v1/schedules — Temporal's schedule list. */
  schedules: ["schedules"] as const,
  /**
   * GET /api/v1/recordings — the bag catalogue. Shared by the list and by the
   * recorder panel's "did the bag I just stopped come out playable" read, so
   * stopping a recording updates both from one request.
   */
  recordings: ["recordings"] as const,
  /**
   * GET /api/v1/recordings/active — the live recorder, polled on its own.
   *
   * Deliberately NOT folded into `recordings`: it is the 1 Hz tick behind the
   * elapsed readout, and the catalogue it would drag along walks every bag
   * directory on the robot to answer.
   */
  activeRecording: ["active-recording"] as const,
  /**
   * GET /api/v1/network/wifi/scan — a ~45 s nmcli rescan on the robot. Fetched
   * once per Settings visit and on an explicit Rescan only; see useWifiScan for
   * why it is never refetched on its own.
   */
  wifiScan: ["wifi-scan"] as const,
};
