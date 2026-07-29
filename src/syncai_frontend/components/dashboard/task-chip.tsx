"use client";

import { Chip, type Tone } from "@/components/console/instrument";
import type { TaskStatus } from "@/lib/api/task";

// A task in flight is active guidance, which is what magenta means everywhere
// else in the console; a finished one is just a measured outcome.
const STATUS_TONE: Record<TaskStatus, Tone> = {
  PENDING: "cmd",
  IN_PROGRESS: "active",
  COMPLETED: "live",
  FAILED: "warn",
  CANCELED: "neutral",
};

/**
 * A submitted task's status. Shared by every flow that dispatches one (nav
 * goal, posture) so the same workflow state reads the same colour wherever the
 * operator meets it.
 */
export function TaskStatusChip({ status }: { status: TaskStatus }) {
  return <Chip tone={STATUS_TONE[status]}>{status.replace("_", " ")}</Chip>;
}
