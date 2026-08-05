"use client";

import * as React from "react";
import {
  ChevronDownIcon,
  ChevronRightIcon,
  RefreshCwIcon,
  XIcon,
} from "lucide-react";

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
 * **The task editor is two columns and folds away.** Stacking steps, name, mode and
 * dispatch in one column meant the button that runs the thing was a scroll below
 * the thing — and on a page whose first job is "what do I have", a composer that
 * is always open pushes the library up and off the screen. So: the step list
 * takes the wide column, everything you *do* with it (name it, choose when, go)
 * rides in a narrower one that sticks to the top of the viewport while the steps
 * scroll, and the whole section collapses to a single bar when nothing is being
 * authored. Below `lg` the grid is one column again and the reading order falls
 * back to the sentence being composed: these steps → when → go.
 *
 * Switching modes still never touches what was authored — the step list is
 * outside both panes.
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

  /**
   * Whether the composer is unfolded. Starts closed: a fresh mount has an empty
   * step list, and an empty composer is three panels of chrome standing between
   * the operator and the library they came to read.
   *
   * It is forced open while a task is in flight, because Cancel lives in
   * DispatchPanel — the same one-stop-button rule the mode picker's `disabled`
   * and `dispatchSaved` already follow. A collapsed composer over a moving robot
   * would hide the only way to stop it.
   */
  const [composerOpen, setComposerOpen] = React.useState(false);
  const editorOpen = composerOpen || dispatch.running;

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

  /**
   * Schedules that name no saved task, i.e. the ones registered from loose steps
   * through `POST /api/v1/schedules`. Nothing ties them to a library row, so
   * they are counted in the library's footnote instead — see TaskLibrary.
   *
   * (Older schedules can also land here: `saved_task_id` did not always exist in
   * the memo, so one registered before it did reads as unlinked whatever it was
   * made from.)
   */
  const unlinkedScheduleCount = schedules.schedules.filter(
    (entry) => !entry.saved_task_id,
  ).length;

  /** Any MOVE step means the saved task has to name the map it is in. */
  const draftMapName = drafts.steps.some((step) => step.type === "MOVE")
    ? mapName
    : null;
  const saveReason =
    stepReason ??
    (draftMapName === null && drafts.steps.some((step) => step.type === "MOVE")
      ? "The robot has no map loaded, so a task with MOVE steps cannot be scoped to one."
      : null);

  /**
   * Let go of the saved task *and* empty the editor, then fold it away.
   *
   * It used to only detach the link and keep the steps, on the theory that
   * "load A, tweak, save as B" wanted them. But the button reads as "I am done
   * with this", and leaving a full step list behind meant the next thing the
   * operator did — dispatch, or Save as new — acted on rows they thought they
   * had put away. Emptying it is the reading the label already promises.
   *
   * Hence the confirm: there is no undo for a cleared list, and dirty tracking
   * is not available to make it conditional (see SaveGroup on why two explicit
   * buttons exist instead of a diff).
   */
  const stopEditing = () => {
    if (
      drafts.steps.length &&
      !window.confirm(
        `Stop editing "${editing?.name}"? The steps in the editor will be cleared.`,
      )
    ) {
      return;
    }
    drafts.clear();
    setEditing(null);
    // Remount SaveGroup so the name field comes back empty rather than still
    // holding the task that was just let go of.
    setSaveNonce((n) => n + 1);
    setComposerOpen(false);
  };

  const loadSaved = (task: SavedTask) => {
    // fromSavedSteps reads resolved_params, so a vertex moved since the save shows
    // its *current* pose here — which is the whole point of storing the reference.
    drafts.replace(fromSavedSteps(task.steps));
    setEditing({ id: task.id, name: task.name });
    setSaveNonce((n) => n + 1);
    // Loading a task *is* the start of editing it, so the composer unfolds
    // whether or not the operator opened it — otherwise the pencil button would
    // look like it did nothing.
    setComposerOpen(true);
  };

  const dispatchSaved = (task: SavedTask) => {
    // Force the composer back to Dispatch mode before firing. Cancel lives in
    // DispatchPanel and the mode picker freezes for the duration of a run, so
    // starting a task from a library row while the Schedule pane was open would
    // leave a moving robot with no stop button anywhere on screen. The mirror of
    // the one-stop-button rule the Segmented's own comment records.
    setMode("now");
    setDispatchedFrom(task.id);
    // Latched open, not merely forced open for the duration: `editorOpen` would
    // already unfold the composer while the task runs, but folding it shut again
    // the moment the run ends would take the terminal status and its Clear
    // button with it — from a run the operator started two clicks ago.
    setComposerOpen(true);
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
            schedules={schedules.schedules}
            unlinkedScheduleCount={unlinkedScheduleCount}
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

      {/* The editor's own header, outside both columns: it is the handle for the
        * whole section, and a chevron sitting in one column's group header would
        * fold the other column from a control that does not belong to it.
        *
        * "Task editor" on screen, "composer" in this file's prose and state
        * names — the operator's word for the thing is the one that goes in the
        * label, the same split the REST vocabulary makes between "vertex" and
        * the `MapPoint` it is stored as. */}
      <div className="flex items-center gap-2">
        <button
          type="button"
          aria-expanded={editorOpen}
          disabled={dispatch.running}
          onClick={() => setComposerOpen((v) => !v)}
          title={
            dispatch.running
              ? "A task is running — Cancel is in the dispatch pane below."
              : editorOpen
                ? "Fold the editor away"
                : "Build or edit a task"
          }
          className="instrument-label flex min-w-0 flex-1 items-center gap-2 rounded-md border border-hairline bg-panel px-3 py-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-100 disabled:hover:bg-panel"
        >
          {editorOpen ? (
            <ChevronDownIcon className="size-3.5 shrink-0" aria-hidden />
          ) : (
            <ChevronRightIcon className="size-3.5 shrink-0" aria-hidden />
          )}
          <span className="shrink-0">Task editor</span>
          {/* Collapsed, this line is the only thing saying what is in there. */}
          <span className="readout min-w-0 flex-1 truncate text-left text-[11px] normal-case">
            {editing
              ? `Editing ${editing.name}`
              : drafts.steps.length
                ? "Unsaved steps"
                : "Empty"}
          </span>
          <Chip tone={drafts.steps.length ? "neutral" : "caution"}>
            {drafts.steps.length}
          </Chip>
        </button>
        {editing && (
          <button
            type="button"
            // Frozen mid-run for the same reason the step list is: the tracked
            // statuses are keyed by position, so emptying the list under a
            // running task would leave them describing rows that are gone.
            disabled={dispatch.running}
            onClick={stopEditing}
            title="Let go of this saved task, clear the editor and fold it away"
            className="instrument-label flex h-9 min-w-0 shrink items-center gap-1 rounded-md border border-signal-cmd/40 bg-signal-cmd/8 px-2 text-signal-cmd transition-colors hover:bg-signal-cmd/16 disabled:opacity-40"
          >
            <span className="min-w-0 truncate">Stop editing</span>
            <XIcon className="size-3 shrink-0" aria-hidden />
          </button>
        )}
      </div>

      {editorOpen && (
        <div className="mt-2 grid gap-4 lg:grid-cols-[minmax(0,1fr)_22rem] lg:items-start">
          <div className="overflow-hidden rounded-md border border-hairline bg-panel">
            <InstrumentGroup label="Steps">
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
          </div>

          {/* Everything you do *with* the steps, in its own column. Sticky from lg
            * so a forty-step list scrolls past a Dispatch button that stays put —
            * the whole reason the one-column stack was uncomfortable. `top-4`
            * matches the page's own py-8 breathing room; the page scroller is the
            * containing block (see app/tasks/page.tsx). */}
          <div className="overflow-hidden rounded-md border border-hairline bg-panel lg:sticky lg:top-4">
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
                    : // Said before the fact, because it cannot be said after: a
                      // schedule registered from loose steps records no source,
                      // so no library row can ever show that it exists. Saving
                      // first is the whole difference, and it is one button away.
                      "These steps are not saved as a task, so this schedule will not show on any library row. Save it first if you want to see later that it runs on its own."
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
        </div>
      )}

      {/* A third frame, because the registered schedules are a different object
       * from the one being authored — not another group inside the composer.
       *
       * Shown whenever any exist, not only in Schedule mode. Gating it on the
       * mode meant the one place that lists what this robot does unattended was
       * behind a picker inside a folded-away editor: an operator on the Dispatch
       * pane could not see that anything was registered at all. */}
      {(mode === "schedule" || schedules.schedules.length > 0) && (
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
