"use client";

import {
  Chip,
  InstrumentGroup,
  Readout,
  Segmented,
} from "@/components/console/instrument";
import type { ModeSwitch } from "@/hooks/use-mode-switch";
import type { SwitchableMode } from "@/lib/api/mapping";

const MODES: readonly { value: SwitchableMode; label: string }[] = [
  { value: "MANUAL", label: "Mapping" },
  { value: "AUTO", label: "Nav" },
];

/**
 * Which operating mode is commanded, and which one the robot reports.
 *
 * The same two-row epistemics as LocomotionControl, with the roles even
 * starker: the segments are a command whose acknowledgement channel is the
 * whole stack rebuilding, and the readout is sys_manager's answer relayed
 * through robot_state. Between the two sits a 10–30 s window where the console
 * has no link at all — the caption owns saying that, so the dead status strip
 * during a switch reads as the described behaviour instead of a fault.
 *
 * Selection goes through `onSelect` rather than straight to the hook: the page
 * owns the one rule that may stop a switch (leaving MANUAL with an unsaved
 * map) and this control has no way to know whether the run was saved.
 */
export function ModeControl({
  control,
  onSelect,
}: {
  control: ModeSwitch;
  onSelect: (mode: SwitchableMode) => void;
}) {
  const { reported, stateStatus, pending, busy, error } = control;

  // MAINTENANCE (or no frame yet) lights no segment — lighting the nearest
  // one would be a claim about the robot, same rule as the policy row.
  const lit =
    pending ??
    (reported === "MANUAL" || reported === "AUTO" ? reported : null);

  return (
    <InstrumentGroup
      label="Operating mode"
      action={pending ? <Chip tone="caution">Switching</Chip> : undefined}
      caption={
        pending
          ? "The robot's stack is rebuilding. The console loses its link for a while and reconnects on its own."
          : "Switching rebuilds the robot's stack; the console drops its link for ~30 s."
      }
    >
      <div>
        <p className="instrument-label mb-1 text-muted-foreground">
          Mode · commanded
        </p>
        <Segmented
          stretch
          value={lit}
          options={MODES}
          onChange={onSelect}
          disabled={busy}
        />
      </div>

      <Readout
        label={pending ? `Reported · awaiting ${pending}` : "Reported"}
        value={reported ?? "—"}
        tone={
          pending || stateStatus === "error"
            ? "caution"
            : reported
              ? "live"
              : "neutral"
        }
      />

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}
