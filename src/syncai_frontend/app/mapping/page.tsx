"use client";

import * as React from "react";
import {
  ArrowLeftIcon,
  Grid2x2Icon,
  MapIcon,
  NavigationIcon,
  RotateCcwIcon,
  Trash2Icon,
} from "lucide-react";

import { overlayPanel } from "@/components/console/instrument";
import { ManualControl } from "@/components/dashboard/manual-control";
import { PointCloudCanvas } from "@/components/dashboard/pointcloud-canvas";
import { ModeControl } from "@/components/mapping/mode-control";
import { ResetRunControl } from "@/components/mapping/reset-run-control";
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
import { resetMappingRun, type SwitchableMode } from "@/lib/api/mapping";
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

/*
 * There is no camera-mode control here, unlike the dashboard's viewport
 * (components/dashboard/pointcloud-view.tsx, which keeps both). The canvas runs
 * on its `cameraMode` default, Move, and Focus — the mode that keeps the camera
 * locked on the robot — is not offered on this screen for now.
 *
 * Mapping is the one screen where the view is not about where the robot is. The
 * operator is driving it around to grow a cloud, and what they are reading is
 * the shape of the room that has been covered so far and the hole that has not;
 * a camera that rides the robot takes exactly that judgement away, and the run
 * is long enough that it is the judgement the whole screen is for. The
 * top-down button beside where this used to sit is the framing that question
 * actually wants.
 */

/**
 * What the operator is about to do, once they have said yes.
 *
 * `unsaved` is frozen when the dialog opens rather than read at render: it is
 * the reason the dialog says what it says, and a save landing underneath an
 * open dialog must not rewrite the question being asked.
 */
type Confirming =
  | { kind: "reset" }
  | { kind: "switch"; to: SwitchableMode; unsaved: boolean };

/**
 * The four questions this page can ask, keyed by `confirmKey` below.
 *
 * `leave` and `reset` are the two ways to lose the run; `MANUAL` and `AUTO` are
 * the plain mode switches, which destroy nothing but do tear the whole stack
 * down for ~30 s. Hence `destructive`: the red is spent only where something is
 * actually unrecoverable, so it keeps meaning that (same rule as the map
 * library's Switch-vs-Delete dialogs).
 */
const CONFIRM_COPY = {
  leave: {
    title: "Leave mapping without saving?",
    // `body: null` means the dialog writes its own line — the reset copy is the
    // one that depends on state (`savedRun`).
    body: "This run's map is only in the robot's memory. Switching discards it for good.",
    icon: Trash2Icon,
    confirm: "Discard and switch",
    cancel: "Keep mapping",
    destructive: true,
  },
  reset: {
    title: "Start a new map?",
    body: null,
    icon: RotateCcwIcon,
    confirm: "Discard and start over",
    cancel: "Keep mapping",
    destructive: true,
  },
  MANUAL: {
    title: "Switch to Mapping mode?",
    body: "The stack restarts: anything running now stops and the console drops its link for ~30 s.",
    icon: MapIcon,
    confirm: "Switch to Mapping",
    cancel: "Cancel",
    destructive: false,
  },
  AUTO: {
    title: "Switch to Nav mode?",
    body: "The stack restarts on the active map: anything running now stops and the console drops its link for ~30 s.",
    icon: NavigationIcon,
    confirm: "Switch to Nav",
    cancel: "Cancel",
    destructive: false,
  },
} as const;

/** Which copy a pending confirmation reads. */
function confirmKey(confirming: Confirming): keyof typeof CONFIRM_COPY {
  if (confirming.kind === "reset") return "reset";
  return confirming.unsaved ? "leave" : confirming.to;
}

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
 * The one rule this page owns: **every act that rebuilds or discards something
 * goes through one confirm dialog**, because nothing downstream will stop you —
 * sys_manager takes `switch_mode` at its word and pgo holds the run in RAM. So
 * both mode segments confirm, not just the one that loses a map: a switch in
 * either direction tears the stack down for ~30 s and stops whatever the robot
 * was doing, which is not something to hand to a stray tap on a touchscreen.
 * What differs is how loud the dialog is — `savedRun` is what escalates leaving
 * MANUAL from "the stack restarts" to "the map is gone". It re-arms on two
 * events that look nothing alike: a new MANUAL run, tracked by watching
 * `reported` change (the adjust-during-render pattern, same as
 * VertexMoveDialog's `shown`), and a successful reset, which has to say so
 * explicitly because `reported` never moves across one.
 */
export default function MappingPage() {
  const control = useModeSwitch();
  const { pose, joints } = useTelemetry();
  const [cloudStatus, setCloudStatus] = React.useState<StreamStatus>("connecting");
  const [mapCloudStatus, setMapCloudStatus] =
    React.useState<StreamStatus>("connecting");
  const [showMapSoFar, setShowMapSoFar] = React.useState(true);
  const [topDownNonce, setTopDownNonce] = React.useState(0);
  const [savedRun, setSavedRun] = React.useState(false);
  // One dialog, every question. They all ask "you are about to interrupt the
  // robot" and differ only in what happens next, so a second AlertDialog block
  // would be a copy of the same copy, drifting apart at the first reword.
  const [confirming, setConfirming] = React.useState<Confirming | null>(null);
  const [resetBusy, setResetBusy] = React.useState(false);
  const [resetError, setResetError] = React.useState<string | null>(null);
  const [resetDone, setResetDone] = React.useState<string | null>(null);
  // Bumped on every successful reset and used as SaveMapControl's key, which
  // remounts it. Remounting is the least code that says "different run": it
  // clears the "Saved 'foo' / 2D grid ready" line, which would otherwise sit
  // under a brand-new empty map claiming it was already saved.
  const [runNonce, setRunNonce] = React.useState(0);

  const { reported, pending, switchTo } = control;

  // A new MANUAL run means the save guard re-arms — whatever was saved last
  // run says nothing about this one.
  const [prevReported, setPrevReported] = React.useState(reported);
  if (reported !== prevReported) {
    setPrevReported(reported);
    if (reported === "MANUAL") setSavedRun(false);
  }

  const mapping = reported === "MANUAL" && !pending;

  // No segment commands a switch directly — every one of them opens the dialog
  // and the operator's second tap is what reaches sys_manager. The one press
  // that still does nothing is the mode already reported: re-selecting it is
  // not a request (sys_manager refuses to rebuild the live mode anyway), so
  // asking about it would be a dialog whose yes does nothing.
  const selectMode = React.useCallback(
    (mode: SwitchableMode) => {
      if (mode === reported && !pending) return;
      setConfirming({
        kind: "switch",
        to: mode,
        // Only leaving MANUAL can lose a run — and only towards AUTO, since
        // re-selecting MANUAL to cancel a pending switch keeps the run alive.
        unsaved: mode === "AUTO" && reported === "MANUAL" && !savedRun,
      });
    },
    [reported, pending, savedRun],
  );

  const runReset = React.useCallback(async () => {
    setConfirming(null);
    setResetBusy(true);
    setResetError(null);
    setResetDone(null);
    try {
      const result = await resetMappingRun();
      setResetDone(result.message);
      // The line that makes the leave-guard keep working. Its usual re-arm
      // watches `reported` change, and `reported` stays MANUAL straight through
      // a reset — so without this, an operator who saves map A, resets, drives
      // map B and then switches to Nav gets no warning and loses B silently.
      setSavedRun(false);
      setRunNonce((n) => n + 1);
    } catch (cause) {
      setResetError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setResetBusy(false);
    }
  }, []);

  // Confirmed even when the run IS saved: a misclick costs the run either way,
  // and "it was saved" says nothing about the minutes driven since the save.
  const requestReset = React.useCallback(
    () => setConfirming({ kind: "reset" }),
    [],
  );

  // The last question asked, held past the dialog closing so its copy does not
  // change under the close animation (adjust-during-render, as above). Reading
  // `confirming` directly would mean guarding every line of text for the null
  // frame the operator never sees.
  const [shown, setShown] = React.useState<Confirming>({ kind: "reset" });
  if (confirming && confirming !== shown) setShown(confirming);
  const confirmCopy = CONFIRM_COPY[confirmKey(shown)];
  const ConfirmIcon = confirmCopy.icon;

  const confirmDialog = React.useCallback(() => {
    if (!confirming) return;
    if (confirming.kind === "reset") {
      void runReset();
      return;
    }
    const target = confirming.to;
    setConfirming(null);
    void switchTo(target);
  }, [confirming, runReset, switchTo]);

  return (
    <div className="flex h-full flex-col overflow-y-auto lg:flex-row lg:overflow-hidden">
      <section
        aria-label="Mapping viewport"
        className="relative h-[55vh] shrink-0 lg:h-full lg:flex-1"
      >
        <PointCloudCanvas
          pose={pose}
          joints={joints}
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
        <SaveMapControl
          key={runNonce}
          enabled={mapping}
          onSaved={() => setSavedRun(true)}
        />
        <ResetRunControl
          enabled={mapping}
          busy={resetBusy}
          error={resetError}
          done={resetDone}
          onRequest={requestReset}
        />
      </aside>

      <AlertDialog
        open={confirming !== null}
        onOpenChange={(open) => {
          if (!open) setConfirming(null);
        }}
      >
        <AlertDialogContent>
          <AlertDialogHeader>
            <AlertDialogTitle>{confirmCopy.title}</AlertDialogTitle>
            <AlertDialogDescription>
              {confirmCopy.body ?? (
                <>
                  {savedRun
                    ? "Anything driven since the last save is discarded."
                    : "This run's map is discarded for good."}{" "}
                  Keep the robot still while the lidar re-levels.
                </>
              )}
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            {/* Icon and label both, the house pattern (see the map library's
              * delete and switch dialogs): the glyph tells the two buttons
              * apart at a glance, the word is what makes the committing one
              * unmistakable. The confirm glyph is the one the action already
              * wears elsewhere on this page — RotateCcw is ResetRunControl's
              * own button — so the dialog reads as that control continued. */}
            <Button variant="ghost" size="sm" onClick={() => setConfirming(null)}>
              <ArrowLeftIcon data-icon="inline-start" />
              {confirmCopy.cancel}
            </Button>
            <Button
              variant={confirmCopy.destructive ? "destructive" : "default"}
              size="sm"
              onClick={confirmDialog}
            >
              <ConfirmIcon data-icon="inline-start" />
              {confirmCopy.confirm}
            </Button>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>
    </div>
  );
}
