"use client";

import * as React from "react";
import { RefreshCwIcon, XIcon } from "lucide-react";

import { Chip, InstrumentGroup, Segmented } from "@/components/console/instrument";
import { DispatchPanel } from "@/components/tasks/dispatch-panel";
import { SaveGroup } from "@/components/tasks/save-group";
import { ScheduleForm } from "@/components/tasks/schedule-form";
import { ScheduleList } from "@/components/tasks/schedule-list";
import { StepList } from "@/components/tasks/step-list";
import { TaskLibrary } from "@/components/tasks/task-library";
import { useActiveMapVertices } from "@/hooks/use-active-map-vertices";
import { useSavedTasks } from "@/hooks/use-saved-tasks";
import { useSchedules } from "@/hooks/use-schedules";
import { useStepDrafts } from "@/hooks/use-step-drafts";
import { useTaskDispatch } from "@/hooks/use-task-dispatch";
import {
  scheduleSavedTask,
  toDispatchSteps,
  type SavedTask,
} from "@/lib/api/saved-task";
import {
  fromSavedSteps,
  stepDraftsSubmittable,
  toSavedSteps,
  toStepRequests,
} from "@/lib/task/step";

type TaskMode = "now" | "schedule";

const MODE_OPTIONS = [
  { value: "now", label: "Dispatch now" },
  { value: "schedule", label: "On a schedule" },
] as const satisfies readonly { value: TaskMode; label: string }[];

/**
 * The task console: a library of saved routes, a composer, and the two ways to run
 * what is in the composer.
 *
 * **The library is first, above the composer.** The complaint this answers is that
 * returning to the page showed nothing saved, so the first thing under the header
 * has to answer "what do I have". Putting it last would leave an operator who
 * saved a route yesterday landing on an empty step list again — the same
 * complaint, one scroll further down.
 *
 * **One route, one mount, one tracker.** Splitting into /tasks + /tasks/[id] was
 * rejected: the App Router unmounts on navigation, so walking back to an index
 * while a task ran would lose the tracked id and with it the only Cancel button
 * for a moving robot. That is the hole the page header already apologises for,
 * promoted from "if you reload" to "if you click".
 *
 * The mode picker still sits *below* the step list, and the list is outside both
 * panes, so switching modes never touches what was authored and the reading order
 * matches the sentence being composed: these steps → when → go.
 */
export function TaskConsole({ robotId }: { robotId: string | null }) {
  const [mode, setMode] = React.useState<TaskMode>("now");
  const drafts = useStepDrafts();
  const dispatch = useTaskDispatch(robotId);
  const schedules = useSchedules();
  const saved = useSavedTasks();
  const { vertices, status: verticesStatus, mapName } = useActiveMapVertices();

  /**
   * Which saved task is loaded in the composer, so Save can offer to overwrite it.
   * Null when the operator is authoring something new.
   */
  const [editing, setEditing] = React.useState<{ id: string; name: string } | null>(
    null,
  );

  /**
   * Which saved task the in-flight run came from; null when it came from the
   * composer. Only used to decide which library row shows the readback, so the
   * two degenerate cases (row deleted mid-run, row not in the current scope)
   * resolve to "no badge" rather than to a lost run.
   */
  const [dispatchedFrom, setDispatchedFrom] = React.useState<string | null>(null);

  /**
   * Reset-by-remount for the two forms, bumped after a successful write. The same
   * trick the vertex panel uses to get a fresh field per draft, rather than
   * clearing state in an effect.
   */
  const [scheduleNonce, setScheduleNonce] = React.useState(0);
  const [saveNonce, setSaveNonce] = React.useState(0);

  const stepsOk = stepDraftsSubmittable(drafts.steps);
  const stepReason = !drafts.steps.length
    ? "Add at least one step."
    : !stepsOk
      ? "Fill in every step's coordinates."
      : null;

  // Only the dispatch path needs a robot id — it is the prefix of the task id.
  // A schedule id is operator-authored, so the schedule path stays fully usable
  // on a robot that has not localized yet and has no state frame to read one from.
  const dispatchReason =
    stepReason ??
    (robotId
      ? null
      : "Waiting for the robot's first state frame — a task id is scoped by robot id.");

  /**
   * The library, scoped to what this robot can actually run: the loaded map's
   * tasks plus the map-independent ones. The rest are counted, not dropped — see
   * TaskLibrary's footnote for why that line exists.
   */
  const visibleTasks = React.useMemo(
    () => saved.tasks.filter((task) => task.map_name === null || task.map_matches_active),
    [saved.tasks],
  );
  const hiddenCount = saved.tasks.length - visibleTasks.length;

  /** Any MOVE step means the saved task has to name the map it is in. */
  const draftMapName = drafts.steps.some((step) => step.type === "MOVE")
    ? mapName
    : null;
  const saveReason =
    stepReason ??
    (draftMapName === null && drafts.steps.some((step) => step.type === "MOVE")
      ? "The robot has no map loaded, so a task with MOVE steps cannot be scoped to one."
      : null);

  const loadSaved = (task: SavedTask) => {
    // fromSavedSteps reads resolved_params, so a vertex moved since the save shows
    // its *current* pose here — which is the whole point of storing the reference.
    drafts.replace(fromSavedSteps(task.steps));
    setEditing({ id: task.id, name: task.name });
    setSaveNonce((n) => n + 1);
  };

  const dispatchSaved = (task: SavedTask) => {
    // Force the composer back to Dispatch mode before firing. Cancel lives in
    // DispatchPanel and the mode picker freezes for the duration of a run, so
    // starting a task from a library row while the Schedule pane was open would
    // leave a moving robot with no stop button anywhere on screen. The mirror of
    // the one-stop-button rule the Segmented's own comment records.
    setMode("now");
    setDispatchedFrom(task.id);
    // Deliberately does NOT load the task into the composer: that would silently
    // discard whatever the operator was authoring, which is the same class of data
    // loss this feature exists to fix.
    void dispatch.send(toDispatchSteps(task.steps));
  };

  return (
    <>
      <div className="mb-4 overflow-hidden rounded-md border border-hairline bg-panel">
        <InstrumentGroup
          label="Saved tasks"
          action={
            <div className="flex items-center gap-1.5">
              <Chip tone={visibleTasks.length ? "neutral" : "caution"}>
                {visibleTasks.length}
              </Chip>
              <button
                type="button"
                aria-label="Refresh saved tasks"
                title="Refresh saved tasks"
                disabled={saved.busy}
                onClick={saved.refresh}
                className="flex size-5 items-center justify-center rounded-sm text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-40"
              >
                <RefreshCwIcon className="size-3.5" aria-hidden />
              </button>
            </div>
          }
          caption="A saved MOVE step follows the vertex it was taken from, so moving a stop on the map updates every route that uses it."
        >
          {saved.error && (
            <p
              role="alert"
              className="text-[11px] leading-snug break-words text-signal-warn"
            >
              {saved.error}
            </p>
          )}
          <TaskLibrary
            tasks={visibleTasks}
            hiddenCount={hiddenCount}
            status={saved.status}
            busy={saved.busy}
            activeMapName={mapName}
            dispatchDisabled={dispatch.running || robotId === null}
            dispatchedFromId={dispatchedFrom}
            taskStatus={dispatch.taskStatus}
            stepStates={dispatch.stepStates}
            onDispatch={dispatchSaved}
            onLoad={loadSaved}
            onSchedule={(task) => {
              // The trigger is authored in the composer's Schedule pane, which is
              // the only place a cron/interval form exists. Loading the task first
              // is what makes that pane describe the thing being scheduled.
              loadSaved(task);
              setMode("schedule");
            }}
            onDelete={(task) => {
              // A confirm rather than an undo: there is no local history to step
              // back over. Same stance as the vertex panel and the schedule list.
              if (
                window.confirm(`Delete saved task "${task.name}"? This cannot be undone.`)
              ) {
                void saved.remove(task.id).then((gone) => {
                  if (gone && editing?.id === task.id) setEditing(null);
                });
              }
            }}
          />
        </InstrumentGroup>
      </div>

      <div className="overflow-hidden rounded-md border border-hairline bg-panel">
        <InstrumentGroup
          label="Steps"
          action={
            <div className="flex min-w-0 items-center gap-1.5">
              {editing && (
                <button
                  type="button"
                  onClick={() => setEditing(null)}
                  title="Stop editing this saved task; keep the steps"
                  className="instrument-label flex h-5 min-w-0 items-center gap-1 rounded-sm border border-signal-cmd/40 bg-signal-cmd/8 px-1.5 text-signal-cmd transition-colors hover:bg-signal-cmd/16"
                >
                  <span className="min-w-0 truncate">Editing {editing.name}</span>
                  <XIcon className="size-3 shrink-0" aria-hidden />
                </button>
              )}
              <Chip tone={drafts.steps.length ? "neutral" : "caution"}>
                {drafts.steps.length}
              </Chip>
            </div>
          }
        >
          <StepList
            steps={drafts.steps}
            vertices={vertices}
            verticesStatus={verticesStatus}
            mapName={mapName}
            disabled={dispatch.running}
            stepStates={dispatch.stepStates}
            onAdd={drafts.add}
            onPatch={drafts.patch}
            onRemove={drafts.remove}
            onMove={drafts.move}
          />
        </InstrumentGroup>

        <InstrumentGroup label="Save">
          <SaveGroup
            key={saveNonce}
            editing={editing}
            ready={stepsOk && saveReason === null}
            reason={saveReason}
            busy={saved.busy}
            error={null}
            existingNames={saved.tasks.map((task) => task.name)}
            onCreate={(name) => {
              void saved
                .create({
                  name,
                  map_name: draftMapName,
                  steps: toSavedSteps(drafts.steps),
                })
                .then((created) => {
                  if (created) {
                    setEditing({ id: created.id, name: created.name });
                    setSaveNonce((n) => n + 1);
                  }
                });
            }}
            onUpdate={(id, name) => {
              void saved
                .update(id, {
                  name,
                  map_name: draftMapName,
                  steps: toSavedSteps(drafts.steps),
                })
                .then((updated) => {
                  if (updated) {
                    setEditing({ id: updated.id, name: updated.name });
                    setSaveNonce((n) => n + 1);
                  }
                });
            }}
          />
        </InstrumentGroup>

        <InstrumentGroup label="When">
          {/* Locked while a task is in flight, and not for the reason the step
           * list is: Cancel lives in the dispatch pane, so letting the operator
           * switch to the schedule pane would hide the only stop button for a
           * robot that is currently moving. Mirroring Cancel into both panes was
           * the alternative and it is worse — two places that can stop a task. */}
          <Segmented
            stretch
            value={mode}
            options={MODE_OPTIONS}
            disabled={dispatch.running}
            onChange={setMode}
          />
        </InstrumentGroup>

        {mode === "now" ? (
          <InstrumentGroup
            label="Dispatch"
            // Static rather than a mapping over the backend's two 502 sentences
            // ("Start workflow failed" / "Failed to connect to Temporal server").
            // Both are terse and neither is actionable on its own, but substituting
            // friendlier prose would put backend copy in the frontend and go stale
            // the day the gateway's wording changes — `detail` is rendered verbatim
            // everywhere else in this console.
            caption="Dispatch goes through the Temporal orchestrator. A failure here is the orchestrator, not the robot."
          >
            <DispatchPanel
              dispatch={dispatch}
              ready={stepsOk && robotId !== null}
              reason={dispatchReason}
              onDispatch={() => {
                // Dispatching from the composer, so no library row owns this run.
                setDispatchedFrom(null);
                void dispatch.send(toStepRequests(drafts.steps));
              }}
            />
          </InstrumentGroup>
        ) : (
          <InstrumentGroup
            label="Schedule"
            caption={
              editing
                ? "Registering from a saved task freezes its coordinates now — later vertex edits do not reach a scheduled run."
                : undefined
            }
          >
            <ScheduleForm
              key={scheduleNonce}
              existingIds={schedules.schedules.map((entry) => entry.id)}
              ready={stepsOk}
              reason={stepReason}
              busy={schedules.busy || saved.busy}
              error={schedules.error}
              onCreate={(id, trigger) => {
                // Two paths on purpose. With a saved task loaded, go through
                // /saved_tasks/{id}/schedule: it re-resolves server-side, records
                // the provenance in the schedule memo (so the row can later be
                // told it has gone stale), and refuses an unattended run against
                // another map or a deleted vertex. Without one, there is no row to
                // reference and the plain schedule endpoint takes the steps.
                const done = editing
                  ? scheduleSavedTask(editing.id, id, trigger).then(() => true)
                  : schedules.create({
                      id,
                      trigger,
                      steps: toStepRequests(drafts.steps),
                    });

                void Promise.resolve(done)
                  .then((created) => {
                    if (created) {
                      setScheduleNonce((n) => n + 1);
                      schedules.refresh();
                    }
                  })
                  .catch(() => {
                    // scheduleSavedTask is the only path that can reject here —
                    // useSchedules already swallows its own. Surfaced through the
                    // saved-task hook so the sentence lands in one place.
                    saved.refresh();
                  });
              }}
            />
          </InstrumentGroup>
        )}
      </div>

      {/* A third frame, because the registered schedules are a different object
       * from the one being authored — not another group inside the composer. */}
      {mode === "schedule" && (
        <div className="mt-4 overflow-hidden rounded-md border border-hairline bg-panel">
          <InstrumentGroup
            label="Registered schedules"
            action={
              <button
                type="button"
                aria-label="Refresh schedules"
                title="Refresh schedules"
                disabled={schedules.busy}
                onClick={schedules.refresh}
                className="flex size-5 items-center justify-center rounded-sm text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-40"
              >
                <RefreshCwIcon className="size-3.5" aria-hidden />
              </button>
            }
          >
            <ScheduleList
              schedules={schedules.schedules}
              savedTasks={saved.tasks}
              status={schedules.status}
              busy={schedules.busy}
              onPause={schedules.pause}
              onResume={schedules.resume}
              onDelete={schedules.remove}
            />
          </InstrumentGroup>
        </div>
      )}
    </>
  );
}
