"use client";

import { ArrowDownToLineIcon, ArrowUpFromLineIcon } from "lucide-react";

import { InstrumentGroup } from "@/components/console/instrument";
import { TaskStatusChip } from "@/components/console/task-chip";
import { usePosture } from "@/hooks/use-posture";
import { cn } from "@/lib/utils";
import type { Posture } from "@/lib/api/task";

const COMMANDS: ReadonlyArray<{
  posture: Posture;
  label: string;
  icon: typeof ArrowUpFromLineIcon;
}> = [
  { posture: "STANDUP", label: "Stand", icon: ArrowUpFromLineIcon },
  { posture: "LIEDOWN", label: "Lie down", icon: ArrowDownToLineIcon },
];

/**
 * Stand up / lie down, as a one-step task each.
 *
 * These live in the rail rather than on the viewport because they are the one
 * thing the operator commands that has no position to it — putting them among
 * the map tools would imply a place on the floor they do not have.
 *
 * The button that was pressed stays lit until its task is terminal, which is
 * the only feedback the operator gets that the command is still in flight: the
 * posture itself is not in any telemetry the console reads.
 */
export function PostureControl({ robotId }: { robotId: string }) {
  const posture = usePosture(robotId);
  const { sent, taskStatus, running, busy, error } = posture;

  return (
    <InstrumentGroup
      label="Posture"
      action={taskStatus ? <TaskStatusChip status={taskStatus} /> : undefined}
    >
      <div className="grid grid-cols-2 gap-1.5">
        {COMMANDS.map((command) => (
          <button
            key={command.posture}
            type="button"
            // One posture at a time: a second command while the first is still
            // running would race the gait controller's mode change.
            disabled={busy || running}
            onClick={() => posture.send(command.posture)}
            className={cn(
              "instrument-label flex h-8 items-center justify-center gap-1.5 rounded-sm border px-2 transition-colors disabled:opacity-40",
              running && sent === command.posture
                ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
                : "border-hairline hover:bg-elevated",
            )}
          >
            <command.icon className="size-3.5 shrink-0" />
            {command.label}
          </button>
        ))}
      </div>

      {sent && !running && (
        <div className="flex items-baseline justify-between gap-2">
          <span className="instrument-label text-muted-foreground">Last</span>
          <button
            type="button"
            onClick={posture.clear}
            className="readout text-[13px] text-muted-foreground transition-colors hover:text-foreground"
          >
            {sent === "STANDUP" ? "Stand" : "Lie down"}
            <span className="ml-1.5 text-[11px]">clear</span>
          </button>
        </div>
      )}

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}
