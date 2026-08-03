"use client";

import { InstrumentGroup, Segmented } from "@/components/console/instrument";
import { useLocomotion, type Controller, type Policy } from "@/hooks/use-locomotion";

const CONTROLLERS: readonly { value: Controller; label: string }[] = [
  { value: "RL", label: "RL" },
  { value: "MPC", label: "MPC" },
];

const POLICIES: readonly { value: Policy; label: string }[] = [
  { value: "PPO", label: "PPO" },
  { value: "HIMLOCO", label: "HIMLOCO" },
];

/**
 * Which locomotion controller is commanded, and under RL which policy.
 *
 * Segmented rather than the momentary buttons PostureControl uses, because these
 * are mutually exclusive *modes* the robot stays in, not verbs it performs once.
 * Nothing is lit until the operator picks something: the console cannot read back
 * which controller is live (see useLocomotion), so a pre-lit segment would be a
 * claim rather than a reading.
 *
 * The policy row is disabled only under MPC, not merely when the controller is
 * unknown. A policy switch is meaningless unless RL is running — but "we have not
 * been told" is not "MPC", and an operator whose robot was put into RL from a
 * terminal, or before a page reload, must not be locked out of changing its
 * policy by a state this console never had.
 */
export function LocomotionControl() {
  const { controller, policy, busy, error, selectController, selectPolicy } =
    useLocomotion();

  const mpc = controller === "MPC";

  return (
    <InstrumentGroup
      label="Locomotion"
      caption="Commanded, not measured — the robot does not report which controller is live."
    >
      <Row label="Controller">
        <Segmented
          stretch
          value={controller}
          options={CONTROLLERS}
          onChange={selectController}
          disabled={busy}
        />
      </Row>

      <Row label={mpc ? "Policy · RL only" : "Policy"}>
        <Segmented
          stretch
          value={policy}
          options={POLICIES}
          onChange={selectPolicy}
          disabled={busy || mpc}
        />
      </Row>

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
