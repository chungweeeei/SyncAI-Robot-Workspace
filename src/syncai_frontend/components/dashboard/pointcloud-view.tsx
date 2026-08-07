"use client";

import * as React from "react";
import { Grid2x2Icon } from "lucide-react";

import { Segmented, overlayPanel } from "@/components/console/instrument";
import { GoalControl } from "@/components/dashboard/goal-control";
import { InitialPoseControl } from "@/components/dashboard/initial-pose-control";
import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { VertexMoveDialog } from "@/components/dashboard/vertex-move-dialog";
import { VertexPlaceControl } from "@/components/dashboard/vertex-place-control";
import { useActiveMapVertices } from "@/hooks/use-active-map-vertices";
import { useGoalTask } from "@/hooks/use-goal-task";
import { useInitialPose } from "@/hooks/use-initial-pose";
import { useActiveMap } from "@/hooks/use-maps";
import { useTelemetry } from "@/hooks/use-telemetry";
import { apiUrl } from "@/lib/api/config";
import { cn } from "@/lib/utils";
import type { MapVertex } from "@/lib/types/map";
import type { PlanarPose } from "@/lib/types/robot";
import type { StreamStatus } from "@/lib/types/stream";

const STATUS_LABEL: Record<StreamStatus, string> = {
  connecting: "Connecting",
  open: "Cloud live",
  closed: "Cloud down",
  error: "Cloud error",
};

const CAMERA_OPTIONS = [
  { value: "move" as const, label: "Move" },
  { value: "focus" as const, label: "Focus" },
];

/**
 * Data-wiring wrapper for the 3D point-cloud viewer — the console's only
 * viewport since the 2D grid canvas was removed. Resolves the active map from
 * the catalogue for the ground plane and its stored vertices, subscribes the
 * telemetry WebSocket for pose and joint angles, and hosts the layer toggles.
 * The live body_cloud stream itself is owned by PointCloudCanvas.
 *
 * It also owns the pick mode. A drag on the ground can mean two things — a nav
 * goal or an initial-pose estimate — and the gesture is the same for both, so
 * exactly one may be armed at a time. Keeping that in one piece of state here
 * (rather than a boolean inside each flow's hook) is what makes arming one
 * disarm the other by construction; two booleans would eventually both be true.
 *
 * Double-clicking a stored vertex is the third way a goal is set, and the only
 * one that needs no mode: the pose already exists and was named by whoever
 * placed it, so there is nothing to drag and nothing to disarm. It goes through
 * the same GoalTask as the other two — one running task, one read-back, one
 * Cancel button, however the pose was chosen.
 */
export function PointCloudView({
  robotId,
  className,
}: {
  robotId: string;
  className?: string;
}) {
  // Which map the stack loaded, and therefore which one's raster to lay under
  // the cloud. A failure or an unconverted map leaves activeMap.grid null and
  // the canvas renders the cloud with no ground plane, same as before.
  const { map: activeMap } = useActiveMap();
  const mapImageUrl = React.useMemo(
    () =>
      activeMap?.grid
        ? apiUrl(`/api/v1/maps/${encodeURIComponent(activeMap.name)}/image`)
        : undefined,
    [activeMap],
  );
  // The stops already placed on that map, drawn on the ground for context: where
  // the robot can be sent is part of reading where it is, and until now the only
  // place they existed was the gridmap editor. The one thing this screen may
  // write is a stop's position — see the hook on why that field and no other.
  //
  // The hook re-reads the map catalogue through its own useMaps, so the
  // dashboard mount costs a second GET /api/v1/maps. That is the price of
  // leaving the task screens' hook contract alone; both fetches are once per
  // mount, not polled.
  const stops = useActiveMapVertices();
  const { vertices, moveVertex } = stops;
  // Robot pose + joints + planned route via the telemetry WebSocket — see
  // useTelemetry on the rates and on why this is a stream and not a poll.
  const { pose, joints, path } = useTelemetry();
  const [status, setStatus] = React.useState<StreamStatus>("connecting");
  const [showMapCloud, setShowMapCloud] = React.useState(false);
  // On by default, like the vertices and for the same reason: the route is a
  // single mark that says what the robot is doing right now, not a layer someone
  // turns on to inspect the localizer.
  const [showPath, setShowPath] = React.useState(true);
  // On by default, unlike the map cloud: the vertices are a handful of markers
  // that say what the map is *for*, while the cloud is hundreds of thousands of
  // points shown only when someone is checking the localizer.
  const [showVertices, setShowVertices] = React.useState(true);
  const [cameraMode, setCameraMode] = React.useState<"move" | "focus">("move");
  /**
   * Bumped to swing the camera overhead; the canvas owns the camera and reacts
   * to the change (see its `topDownNonce`). A counter rather than a boolean
   * because the useful thing about this control is pressing it *again* after
   * orbiting away, and a boolean would already be true.
   */
  const [topDownNonce, setTopDownNonce] = React.useState(0);
  /**
   * What a drag on the ground currently produces — one value, not a mode plus a
   * separate "which vertex", because a re-place armed with no vertex (or a
   * vertex left behind by a disarmed re-place) is a state that must not exist.
   */
  const [pick, setPick] = React.useState<
    | { mode: "goal" | "initial-pose" }
    | { mode: "vertex"; vertex: MapVertex }
    | null
  >(null);
  /** The stop a double-click is asking about; null when the dialog is closed. */
  const [askedVertex, setAskedVertex] = React.useState<MapVertex | null>(null);
  /**
   * A stop whose new pose is being written. It keeps the marker off the map for
   * the length of the request: dropping it at release would put the old mark
   * back at the old spot until the PUT lands, which reads as the re-place having
   * been rejected and then, a moment later, applied.
   */
  const [savingVertex, setSavingVertex] = React.useState<MapVertex | null>(null);

  const task = useGoalTask(robotId);
  const estimate = useInitialPose();

  // Destructured because they are the stable parts of the hooks' return objects
  // (the objects themselves are fresh every render, so depending on those would
  // rebuild the callback on every telemetry frame).
  const { commitPose, clear: clearEstimate } = estimate;
  const { commitGoal, sendGoal } = task;

  const armPick = React.useCallback(
    (mode: "goal" | "initial-pose") => {
      // Arming the pose-estimate tool drops whatever the last drag left behind,
      // so the panel always describes the gesture in progress rather than the
      // previous one. It is also the deterministic half of the read-back's
      // clean-up: the timed auto-clear in useInitialPose handles the operator
      // who walks away, this handles the one who goes straight into another
      // drag, and between them Clear is never the only way out.
      if (mode === "initial-pose") clearEstimate();
      setPick((cur) => (cur?.mode === mode ? null : { mode }));
    },
    [clearEstimate],
  );

  // Arming a re-place replaces whatever was armed, for the same reason the two
  // pose tools are mutually exclusive: the gesture is identical, so only the
  // armed mode says what a drag means.
  const armReplace = React.useCallback((vertex: MapVertex) => {
    setAskedVertex(null);
    setPick({ mode: "vertex", vertex });
  }, []);

  // Single-shot, like RViz's nav-goal / pose-estimate tools: one drag, one pose,
  // then the mode disarms so a stray click on the map cannot restage it. That
  // disarm carries more weight for an initial pose than it used to — that flow
  // publishes on release now, so a second click would re-seed the localizer, not
  // just move a marker — and it carries the same weight for a re-place, which
  // writes the row on release.
  const commitPick = React.useCallback(
    (picked: PlanarPose) => {
      if (pick?.mode === "initial-pose") {
        commitPose(picked);
      } else if (pick?.mode === "vertex") {
        const target = pick.vertex;
        setSavingVertex(target);
        // The list is patched from the server's echo, so a success re-draws the
        // marker at its new pose in the same render that clears this.
        void moveVertex(target.id, picked).finally(() => setSavingVertex(null));
      } else {
        commitGoal(picked);
      }
      setPick(null);
    },
    [pick, commitPose, commitGoal, moveVertex],
  );

  // Confirmed in the dialog, so it goes straight out as a MOVE task — the
  // question was already asked, and staging it again would leave the operator
  // hunting for a Send button after they said yes. The dialog closes first: the
  // task's own state is reported by GoalControl, which is where an error from
  // this submit shows up too.
  const moveToVertex = React.useCallback(
    (vertex: MapVertex) => {
      setAskedVertex(null);
      void sendGoal({ x: vertex.x, y: vertex.y, theta: vertex.theta });
    },
    [sendGoal],
  );

  return (
    <div className={cn("relative h-full w-full", className)}>
      <PointCloudCanvas
        meta={activeMap?.grid ?? undefined}
        mapImageUrl={mapImageUrl}
        mapName={activeMap?.name}
        pose={pose}
        joints={joints}
        showMapCloud={showMapCloud}
        path={path}
        showPath={showPath}
        vertices={vertices}
        showVertices={showVertices}
        onVertexActivate={setAskedVertex}
        movingVertex={
          pick?.mode === "vertex" ? pick.vertex : (savingVertex ?? null)
        }
        cameraMode={cameraMode}
        goal={task.goal}
        initialPose={estimate.pose}
        topDownNonce={topDownNonce}
        pickMode={pick?.mode ?? null}
        onPickCommit={commitPick}
        onStatus={setStatus}
      />

      <VertexMoveDialog
        vertex={askedVertex}
        busy={task.busy}
        running={task.running}
        onConfirm={moveToVertex}
        onReplace={armReplace}
        onClose={() => setAskedVertex(null)}
      />

      {/* Stream health for the cloud itself. The status strip's sweep covers the
        * 1 Hz state poll; this WebSocket is a separate link that can fail on its
        * own, so it gets its own indicator — in the same three tones. */}
      <div
        className={cn(
          overlayPanel,
          "absolute top-3 right-3 flex items-center gap-2 px-2 py-1.5",
        )}
      >
        <span
          className={cn(
            "inline-block size-2 rounded-full",
            status === "open"
              ? "bg-signal-live"
              : status === "connecting"
                ? "bg-signal-caution"
                : "bg-signal-warn",
          )}
        />
        <span className="instrument-label text-muted-foreground">
          {STATUS_LABEL[status]}
        </span>
      </div>

      {/* Both pose tools in one column, goal first: it is the one used on every
        * run, while an initial pose is a recovery action. */}
      <div className="absolute top-3 left-3 flex w-56 flex-col gap-2">
        <GoalControl
          task={task}
          armed={pick?.mode === "goal"}
          onArm={() => armPick("goal")}
        />
        <InitialPoseControl
          estimate={estimate}
          armed={pick?.mode === "initial-pose"}
          onArm={() => armPick("initial-pose")}
        />
        {/* Third in the column and only while it is live: a re-place is entered
          * from the map, not from here, so a resting control would be a button
          * that does nothing until something else has already happened. It also
          * outlives the gesture when the write fails — that sentence has to land
          * somewhere, and the panel that armed it is where the operator is
          * looking. */}
        <VertexPlaceControl
          vertex={pick?.mode === "vertex" ? pick.vertex : null}
          busy={stops.busy}
          error={stops.writeError}
          onCancel={() => setPick(null)}
          onDismissError={stops.clearWriteError}
        />
      </div>

      {/* Viewport controls sit along the bottom edge, out of the way of the
        * goal readback and of the robot, which the camera keeps centred. */}
      <div className="absolute bottom-3 left-3 flex items-center gap-2">
        <Segmented
          value={cameraMode}
          options={CAMERA_OPTIONS}
          onChange={setCameraMode}
          className={overlayPanel}
        />
        {/* Beside the camera modes, not among them: Move and Focus say what a
          * drag does and stay true until changed, while this is a one-shot
          * placement that the very next drag can orbit out of. Rendering it as
          * a third segment would leave a segment highlighted for a view the
          * operator is no longer in. */}
        <button
          type="button"
          onClick={() => setTopDownNonce((n) => n + 1)}
          title="Look straight down at the map"
          className={cn(
            overlayPanel,
            "instrument-label flex h-6 items-center gap-1.5 px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground",
          )}
        >
          <Grid2x2Icon aria-hidden className="size-3.5" />
          Top down
        </button>
        <LayerToggle
          label="Map cloud"
          on={showMapCloud}
          onToggle={() => setShowMapCloud((v) => !v)}
        />
        {/* Only offered when there is something to hide. A control that toggles
          * an empty layer is indistinguishable from one that is broken. */}
        {vertices.length > 0 && (
          <LayerToggle
            label="Vertices"
            on={showVertices}
            onToggle={() => setShowVertices((v) => !v)}
          />
        )}
        {/* Same rule, and here it means the control comes and goes with the run:
          * there is no route to hide between tasks. */}
        {path !== undefined && path.points.length > 0 && (
          <LayerToggle
            label="Path"
            on={showPath}
            onToggle={() => setShowPath((v) => !v)}
          />
        )}
      </div>
    </div>
  );
}

/**
 * One optional scene layer, on or off. Pressed state uses the commanded hue,
 * like the pick-mode buttons above it: what is drawn in the viewport is a choice
 * the operator made, and it has to be readable as one at a glance.
 */
function LayerToggle({
  label,
  on,
  onToggle,
}: {
  label: string;
  on: boolean;
  onToggle: () => void;
}) {
  return (
    <button
      type="button"
      aria-pressed={on}
      onClick={onToggle}
      className={cn(
        overlayPanel,
        "instrument-label h-6 px-2 transition-colors",
        on
          ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
          : "text-muted-foreground hover:bg-elevated hover:text-foreground",
      )}
    >
      {label}
    </button>
  );
}
