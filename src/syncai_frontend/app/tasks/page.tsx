"use client";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { TaskConsole } from "@/components/tasks/task-console";

/**
 * Chrome only — TaskConsole owns the library, the composing, the dispatching and
 * the schedules.
 *
 * A step list survives a reload: it is saved server-side and the library is the
 * first thing on the page.
 *
 * A run in flight now survives one too. `GET /api/v1/active_tasks` answers what
 * is executing on this robot's Temporal task queue, whoever started it, so a
 * reloaded page recovers the task id it never held — and with the id it gets the
 * status chip in the masthead, the banner above the library, and Cancel. That
 * also covers the two cases no browser state ever could: a run started from
 * another console, and one a schedule fired overnight.
 *
 * What still does not survive is the *detail*: which of the steps is executing.
 * That readback is joined by step id inside `useTaskDispatch`, which only has it
 * for a task this mount dispatched, so a recovered run is reported at task level
 * only. Adopting a run back into the composer is a separate piece of work; the
 * Temporal UI on :8081 remains the place to inspect one step by step.
 */
export default function TasksPage() {
  const { state } = useConsoleRobotState();

  return (
    // Like /maps and /settings, this screen owns its scroll: the shell's <main>
    // and <body> are both overflow-hidden, so h-full plus overflow-y-auto here is
    // what gives the column a definite height to scroll inside.
    <div className="h-full overflow-y-auto">
      {/* Wider than /maps' max-w-5xl, because the composer is two columns: the
       * step rows keep roughly the 3xl this page used to be while the pane that
       * names and dispatches them takes 22rem beside it. At 5xl the steps would
       * come out *narrower* than they were before the split, which is the
       * opposite of the point. Below lg the grid collapses and this is just a
       * wide-ish single column. */}
      <div className="mx-auto w-full max-w-6xl px-4 py-8">
        <header className="mb-6">
          <p className="instrument-label text-muted-foreground">Robot</p>
          <h1 className="mt-1.5 text-xl font-semibold tracking-tight">Tasks</h1>
          <p className="mt-1 text-sm text-muted-foreground">
            Build a run for{" "}
            <span className="readout">{state?.robot_id ?? "this robot"}</span> step
            by step, save it, then dispatch it now or register it to repeat. Steps
            execute in order, one at a time, and a failing step stops the ones after
            it.
          </p>
        </header>

        <TaskConsole robotId={state?.robot_id ?? null} />
      </div>
    </div>
  );
}
