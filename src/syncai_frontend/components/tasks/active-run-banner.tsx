"use client";

import * as React from "react";
import { XIcon } from "lucide-react";

import { TaskStatusChip } from "@/components/console/task-chip";
import { useConsoleActiveTasks } from "@/components/console/active-task-context";
import { cancelTask, type ActiveTask } from "@/lib/api/task";

/**
 * Runs this tab is not following, with a way to stop them.
 *
 * The console used to have exactly one handle on a task: the id returned by the
 * dispatch that started it, held in memory by the tab that dispatched it. A
 * reload, a navigation, a second browser or a schedule firing overnight all
 * produced the same situation — a robot executing a task with no status and no
 * Cancel anywhere on screen, and the Temporal UI on :8081 as the only recourse.
 * GET /api/v1/active_tasks is what ended that, and this is where it is spent.
 *
 * Rows this tab *is* following are filtered out, because DispatchPanel already
 * shows them with per-step detail this list does not have. So an empty banner is
 * the normal state, and a non-empty one always means "something is running that
 * you are not looking at".
 */
export function ActiveRunBanner({
  /** The run the composer is tracking, so it is not announced twice. */
  trackedTaskId,
}: {
  trackedTaskId: string | null;
}) {
  const { tasks, status, asOf, refresh } = useConsoleActiveTasks();

  const unattended = tasks.filter((task) => task.id !== trackedTaskId);

  // Silent while the poll is failing. The status strip already carries that
  // failure in the caution hue, and a second copy of it here would be a banner
  // that appears when nothing is wrong with the *tasks*.
  if (status !== "ok" || !unattended.length) return null;

  return (
    <div className="mb-4 space-y-1.5 rounded-md border border-signal-active/40 bg-signal-active/8 p-2.5">
      <p className="instrument-label text-signal-active">
        Running outside this editor
      </p>
      {unattended.map((task) => (
        <ActiveRunRow key={task.id} task={task} asOf={asOf} onDone={refresh} />
      ))}
    </div>
  );
}

function ActiveRunRow({
  task,
  asOf,
  onDone,
}: {
  task: ActiveTask;
  asOf: string | null;
  onDone: () => void;
}) {
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  const cancel = async () => {
    setBusy(true);
    setError(null);
    try {
      await cancelTask(task.id);
      // Re-read rather than dropping the row locally: cancelling is a request,
      // and the workflow stops when it stops. The row disappearing on the next
      // poll is the honest signal that it did.
      onDone();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="space-y-1">
      <div className="flex flex-wrap items-center gap-2">
        <TaskStatusChip status={task.status} />
        <span className="readout min-w-0 flex-1 truncate text-[12px]">
          {task.id}
        </span>
        {task.schedule_id && (
          <span className="instrument-label text-muted-foreground">
            via {task.schedule_id}
          </span>
        )}
        <span className="readout shrink-0 text-[11px] text-muted-foreground">
          {elapsed(task.started_at, asOf)}
        </span>
        <button
          type="button"
          disabled={busy}
          onClick={cancel}
          className="instrument-label flex h-6 shrink-0 items-center gap-1 rounded-sm border border-signal-warn/50 bg-signal-warn/12 px-2 text-signal-warn transition-colors hover:bg-signal-warn/20 disabled:opacity-50"
        >
          <XIcon className="size-3" aria-hidden />
          Cancel
        </button>
      </div>

      {error && (
        <p
          role="alert"
          className="text-[11px] leading-snug break-words text-signal-warn"
        >
          {error}
        </p>
      )}
    </div>
  );
}

/**
 * How long the run has been going, as `m:ss` / `h:mm:ss`.
 *
 * Measured between two *server* timestamps — the start time and the snapshot's
 * `as_of` — never against the browser's clock. The backend and the console are
 * different machines, and this answer is served from a short server-side cache,
 * so mixing the two would show a robot that started three seconds in the future
 * on a console whose clock is a little behind.
 */
function elapsed(startedAt: string, asOf: string | null): string {
  if (!asOf) return "—";
  const seconds = Math.max(
    0,
    Math.floor((Date.parse(asOf) - Date.parse(startedAt)) / 1000),
  );
  const s = String(seconds % 60).padStart(2, "0");
  const m = Math.floor(seconds / 60) % 60;
  const h = Math.floor(seconds / 3600);
  return h > 0 ? `${h}:${String(m).padStart(2, "0")}:${s}` : `${m}:${s}`;
}
