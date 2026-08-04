"use client";

import { useConsoleRobotState } from "@/components/console/robot-state-context";
import { TaskConsole } from "@/components/tasks/task-console";

/**
 * Chrome only — TaskConsole owns the library, the composing, the dispatching and
 * the schedules.
 *
 * A step list now survives a reload: it is saved server-side and the library is
 * the first thing on the page. What does *not* survive is the tracked id of a run
 * already in flight — there is no `GET /api/v1/tasks` collection to recover it
 * from and nothing here persists it, so reloading mid-run leaves the robot going
 * with no status or Cancel on screen. That is the same limitation the dashboard's
 * goal flow has, and the Temporal UI on :8081 is where such a run is picked back
 * up.
 */
export default function TasksPage() {
  const { state } = useConsoleRobotState();

  return (
    // Like /maps and /settings, this screen owns its scroll: the shell's <main>
    // and <body> are both overflow-hidden, so h-full plus overflow-y-auto here is
    // what gives the column a definite height to scroll inside.
    <div className="h-full overflow-y-auto">
      {/* Between /maps' max-w-5xl (a card grid) and /settings' max-w-2xl (one
       * column of form): the step rows are wide but there is only one column of
       * them, and 5xl would stretch a coordinate field into an empty box. */}
      <div className="mx-auto w-full max-w-3xl px-4 py-8">
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
