"use client";

import { Chip } from "@/components/console/instrument";
import { useConsoleActiveTasks } from "@/components/console/active-task-context";
import { STATUS_TONE } from "@/components/console/task-chip";

/**
 * Is this robot executing something, on every screen.
 *
 * Not `TaskStatusChip`: "IN PROGRESS" is too wide for the masthead, which is
 * already carrying the robot id, the mode, the map, the link and the battery.
 * It borrows that component's tone map instead, so a running task is the same
 * magenta here as it is in the readback on /tasks.
 *
 * The three states are not symmetric, and the asymmetry is the point:
 *
 *  - **Error is never drawn as idle.** A failed poll means the console does not
 *    know, and a confident "IDLE" over a robot that is driving is the one way
 *    this indicator can do harm. It keeps the last known list and says, in the
 *    caution hue, that it is a memory.
 *  - **Idle hides on a narrow viewport**, like the Map group, because "nothing
 *    is happening" is the state you do not need to be told.
 *  - **Running never hides.** It is the only thing on this strip that says a
 *    machine is moving.
 */
export function ActiveTaskChip() {
  const { tasks, status } = useConsoleActiveTasks();

  // First paint: the strip's own Link chip already says the console is still
  // finding its feet, and a second "…" beside it adds nothing.
  if (status === "loading") return null;

  if (status === "error") {
    return (
      <Chip
        tone="caution"
        title="The orchestrator could not be reached. This is the last state the console knew about, not a current reading."
      >
        TASK ?
      </Chip>
    );
  }

  if (tasks.length === 0) {
    return (
      <Chip tone="neutral" className="hidden sm:inline-flex">
        IDLE
      </Chip>
    );
  }

  // One line per run, so a hover answers "which one, since when, and did a
  // schedule start it" without leaving the screen the operator is on.
  const title = tasks
    .map((task) => {
      const started = task.started_at.slice(11, 19);
      const via = task.schedule_id ? ` · via ${task.schedule_id}` : "";
      return `${task.id} · started ${started}Z${via}`;
    })
    .join("\n");

  return (
    <Chip tone={STATUS_TONE.IN_PROGRESS} title={title}>
      {tasks.length > 1 ? `RUNNING ×${tasks.length}` : "RUNNING"}
    </Chip>
  );
}
