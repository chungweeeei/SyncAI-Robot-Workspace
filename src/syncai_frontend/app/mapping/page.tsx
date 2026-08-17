"use client";

import * as React from "react";
import { Grid2x2Icon } from "lucide-react";

import { Segmented, overlayPanel } from "@/components/console/instrument";
import { ManualControl } from "@/components/dashboard/manual-control";
import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { ModeControl } from "@/components/mapping/mode-control";
import { SaveMapControl } from "@/components/mapping/save-map-control";
import {
  AlertDialog,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { Button } from "@/components/ui/button";
import { useModeSwitch } from "@/hooks/use-mode-switch";
import { useTelemetry } from "@/hooks/use-telemetry";
import type { SwitchableMode } from "@/lib/api/mapping";
import type { StreamStatus } from "@/lib/types/stream";
import { cn } from "@/lib/utils";

const STATUS_LABEL: Record<StreamStatus, string> = {
  connecting: "Connecting",
  open: "Cloud live",
  closed: "Cloud down",
  error: "Cloud error",
};

// The "map so far" stream's pill reads "Map live" as soon as the socket is
// open — the first dot of data still needs pgo (MANUAL mode) to have banked a
// keyframe, which the mode control alongside already communicates.
const MAP_STATUS_LABEL: Record<StreamStatus, string> = {
  connecting: "Map connecting",
  open: "Map live",
  closed: "Map down",
  error: "Map error",
};

const CAMERA_OPTIONS = [
  { value: "move" as const, label: "Move" },
  { value: "focus" as const, label: "Focus" },
];

/** One stream-health row: dot in the three link tones, then the label. */
function StreamPill({
  status,
  label,
}: {
  status: StreamStatus;
  label: string;
}) {
  return (
    <span className="flex items-center gap-2">
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
      <span className="instrument-label text-muted-foreground">{label}</span>
    </span>
  );
}

/**
 * The mapping screen: drive the robot around and watch the map being built.
 *
 * The same viewport-plus-rail shape as the dashboard, but deliberately NOT
 * gated on the robot state the way that page is: this screen's defining moment
 * is the mode switch, during which the backend is down and there is no state —
 * a page that blanked itself right then would go dark exactly when the
 * operator needs to see "Switching" holding steady.
 *
 * The canvas gets no map meta and no ground image: in MANUAL there is no
 * loaded map — what is on screen IS the map. Two layers say how the run is
 * going: the dim "map so far" layer is pgo's merged keyframe cloud
 * (loop-closure-corrected — watch it snap into shape when a loop closes),
 * streamed only while a mapping session is up, and the bright live scan rides
 * on top of it over the same body_cloud WebSocket as the dashboard (pgo
 * broadcasts the map TF during mapping, so that stream works unchanged). The
 * robot model needs the telemetry pose, which mapping's TF chain may not
 * provide; the clouds are the primary instrument either way.
 *
 * The one rule this page owns: leaving MANUAL with an unsaved run loses the
 * run (pgo holds it in RAM; sys_manager will not stop you), so that switch
 * asks first. `savedRun` resets whenever a new MANUAL run starts, tracked by
 * watching `reported` change — the adjust-during-render pattern, same as
 * VertexMoveDialog's `shown`.
 */
export default function MappingPage() {
  const control = useModeSwitch();
  const { pose, joints } = useTelemetry();
  const [cloudStatus, setCloudStatus] = React.useState<StreamStatus>("connecting");
  const [mapCloudStatus, setMapCloudStatus] =
    React.useState<StreamStatus>("connecting");
  const [showMapSoFar, setShowMapSoFar] = React.useState(true);
  const [cameraMode, setCameraMode] = React.useState<"move" | "focus">("move");
  const [topDownNonce, setTopDownNonce] = React.useState(0);
  const [savedRun, setSavedRun] = React.useState(false);
  const [confirmingLeave, setConfirmingLeave] = React.useState(false);

  const { reported, pending, switchTo } = control;

  // A new MANUAL run means the save guard re-arms — whatever was saved last
  // run says nothing about this one.
  const [prevReported, setPrevReported] = React.useState(reported);
  if (reported !== prevReported) {
    setPrevReported(reported);
    if (reported === "MANUAL") setSavedRun(false);
  }

  const mapping = reported === "MANUAL" && !pending;

  const selectMode = React.useCallback(
    (mode: SwitchableMode) => {
      if (mode === reported && !pending) return;
      if (mode === "AUTO" && reported === "MANUAL" && !savedRun) {
        // The guard, not the switch: sys_manager would happily rebuild AUTO
        // over an unsaved run and the map would be unrecoverable.
        setConfirmingLeave(true);
        return;
      }
      void switchTo(mode);
    },
    [reported, pending, savedRun, switchTo],
  );

  const discardAndLeave = React.useCallback(() => {
    setConfirmingLeave(false);
    void switchTo("AUTO");
  }, [switchTo]);

  return (
    <div className="flex h-full flex-col overflow-y-auto lg:flex-row lg:overflow-hidden">
      <section
        aria-label="Mapping viewport"
        className="relative h-[55vh] shrink-0 lg:h-full lg:flex-1"
      >
        <PointCloudCanvas
          pose={pose}
          joints={joints}
          cameraMode={cameraMode}
          topDownNonce={topDownNonce}
          onStatus={setCloudStatus}
          mapCloudStream={showMapSoFar}
          onMapStatus={setMapCloudStatus}
        />

        {/* Same stream-health pills as the dashboard viewport, one per socket
          * — the two fail independently. During a mode switch both go red with
          * everything else; the mode control's caption is what says that is
          * expected. The map row disappears with its toggle: a deliberately
          * closed stream shown as "Map down" would read as a fault. */}
        <div
          className={cn(
            overlayPanel,
            "absolute top-3 right-3 flex flex-col gap-1.5 px-2 py-1.5",
          )}
        >
          <StreamPill status={cloudStatus} label={STATUS_LABEL[cloudStatus]} />
          {showMapSoFar && (
            <StreamPill
              status={mapCloudStatus}
              label={MAP_STATUS_LABEL[mapCloudStatus]}
            />
          )}
        </div>

        <div className="absolute bottom-3 left-3 flex items-center gap-2">
          <Segmented
            value={cameraMode}
            options={CAMERA_OPTIONS}
            onChange={setCameraMode}
            className={overlayPanel}
          />
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
          {/* Layer toggle in the dashboard's LayerToggle idiom: pressed state
            * in the commanded hue, because what is drawn is the operator's
            * choice. Off closes the WebSocket too — the layer re-arrives
            * seconds after re-enabling. */}
          <button
            type="button"
            aria-pressed={showMapSoFar}
            onClick={() => setShowMapSoFar((v) => !v)}
            className={cn(
              overlayPanel,
              "instrument-label h-6 px-2 transition-colors",
              showMapSoFar
                ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
                : "text-muted-foreground hover:bg-elevated hover:text-foreground",
            )}
          >
            Map so far
          </button>
        </div>

        {/* Only while mapping is actually live: in AUTO the dashboard is the
          * driving screen, and during a switch the channel has no backend to
          * talk to — an armed teleop that cannot send is worse than none. */}
        {mapping && <ManualControl className="absolute right-3 bottom-3" />}
      </section>

      <aside
        aria-label="Mapping controls"
        className="w-full shrink-0 border-t border-hairline bg-panel lg:h-full lg:w-72 lg:overflow-y-auto lg:border-t-0 lg:border-l"
      >
        <ModeControl control={control} onSelect={selectMode} />
        <SaveMapControl enabled={mapping} onSaved={() => setSavedRun(true)} />
      </aside>

      <AlertDialog
        open={confirmingLeave}
        onOpenChange={(open) => {
          if (!open) setConfirmingLeave(false);
        }}
      >
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>Leave mapping without saving?</AlertDialogTitle>
            <AlertDialogDescription>
              This run&apos;s map exists only in the robot&apos;s memory.
              Switching to Nav discards it — there is no way to get it back.
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <Button
              variant="outline"
              size="sm"
              onClick={() => setConfirmingLeave(false)}
            >
              Keep mapping
            </Button>
            <Button variant="destructive" size="sm" onClick={discardAndLeave}>
              Discard and switch
            </Button>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  );
}
