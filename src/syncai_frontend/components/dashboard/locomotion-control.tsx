"use client";

import { Chip, InstrumentGroup, Readout, Segmented } from "@/components/console/instrument";
import { useLocomotion, type Controller, type Policy } from "@/hooks/use-locomotion";
import type { RobotLowLevelMode } from "@/lib/types/robot";

const CONTROLLERS: readonly { value: Controller; label: string }[] = [
  { value: "RL", label: "RL" },
  { value: "MPC", label: "MPC" },
];

const POLICIES: readonly { value: Policy; label: string }[] = [
  { value: "PPO", label: "PPO" },
  { value: "HIMLOCO", label: "HIMLOCO" },
];

/**
 * Which locomotion controller is commanded, and which policy the robot reports.
 *
 * Segmented rather than the momentary buttons PostureControl uses, because these
 * are mutually exclusive *modes* the robot stays in, not verbs it performs once.
 *
 * The two rows do not mean the same kind of thing, and the labels say so:
 *
 * - **Controller** is commanded, defaulting to RL. It cannot be read back —
 *   `low_level_mode.motion` has no code for MPC — so the lit segment is an
 *   assumption plus whatever the operator last pressed.
 * - **Policy** is reported. The lit segment is `low_level_mode.policy`, i.e. what
 *   the controller says it is running. A request that the robot has not confirmed
 *   yet shows as `confirming…` rather than silently looking applied, which is the
 *   whole reason the field was added to the state payload.
 *
 * The policy row is disabled only under a commanded MPC: a policy switch is
 * meaningless unless RL is running.
 *
 * Motion is a readout rather than a control — the postures are PostureControl's,
 * and this is the one place the controller's own state machine is visible.
 */
export function LocomotionControl({
  lowLevelMode,
}: {
  lowLevelMode: RobotLowLevelMode;
}) {
  const {
    controller,
    policy,
    pendingPolicy,
    busy,
    error,
    selectController,
    selectPolicy,
  } = useLocomotion(lowLevelMode);

  const mpc = controller === "MPC";
  // The label carries the raw integer when the backend could not name the value:
  // MPC's motion code is unknown, so "UNKNOWN · 6" is the only form of this
  // readout that lets somebody find out what it actually is.
  const motion =
    lowLevelMode.motion === "UNKNOWN"
      ? `UNKNOWN · ${lowLevelMode.motion_state}`
      : lowLevelMode.motion;

  return (
    <InstrumentGroup
      label="Locomotion"
      action={
        pendingPolicy ? <Chip tone="caution">Confirming</Chip> : undefined
      }
    >
      <Row label="Controller · commanded">
        <Segmented
          stretch
          value={controller}
          options={CONTROLLERS}
          onChange={selectController}
          disabled={busy}
        />
      </Row>

      <Row
        label={
          mpc
            ? "Policy · RL only"
            : pendingPolicy
              ? `Policy · confirming ${pendingPolicy}…`
              : "Policy · reported"
        }
      >
        <Segmented
          stretch
          value={policy}
          options={POLICIES}
          onChange={selectPolicy}
          disabled={busy || mpc}
        />
      </Row>

      {/* Not in the segmented rows: the robot can report a policy this control
        * cannot ask for (CHAMP / ISSAC are real and deliberately unexposed), and
        * that must be visible rather than showing as no selection. */}
      {policy === null && (
        <Readout label="Reported" value={lowLevelMode.policy} tone="caution" />
      )}

      <Readout label="Motion" value={motion} tone="live" />

      {error && (
        <p className="text-[11px] leading-snug break-words text-signal-warn">
          {error}
        </p>
      )}
    </InstrumentGroup>
  );
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div>
      <p className="instrument-label mb-1 text-muted-foreground">{label}</p>
      {children}
    </div>
  );
}
