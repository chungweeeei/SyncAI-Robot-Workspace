"use client";

import * as React from "react";
import { JoystickIcon } from "lucide-react";

import { overlayPanel } from "@/components/console/instrument";
import { Thumbstick } from "@/components/dashboard/thumbstick";
import { useJoystick } from "@/hooks/use-joystick";
import { cn } from "@/lib/utils";

/**
 * Explicit sign, fixed 5-character width: with tabular-nums (the `readout`
 * utility) the row never reflows as values change, and "+0.00 / −0.00" makes
 * the at-rest state legible as a value rather than an empty display. U+2212
 * minus, not hyphen — same width as the plus in a tabular font.
 */
function formatAxis(value: number): string {
  return `${value >= 0 ? "+" : "−"}${Math.abs(value).toFixed(2)}`;
}

/**
 * Manual drive panel: two thumbsticks (left = planar translation vx/vy — the
 * G23 can strafe — right = rotation wz) over a readout of the commanded,
 * normalized vector. Pointer and WASD/QE keyboard input, merged in
 * useJoystick.
 *
 * **Visual-only for now.** Nothing here talks to the robot: there is no
 * cmd_vel path through the backend yet. The caption under the readouts is that
 * fact made honest on screen — delete it when the sender lands. (It started as
 * a VISUAL chip in the header; three pieces of caps text on a 240px row read
 * as clutter, and a sentence says more than a badge.) The sender's tap-in
 * point is `useJoystick().vectorRef` (see its doc for why a ref and not a
 * callback), which also means this panel is not gated on RobotMode ===
 * "MANUAL" yet: with no command path there is nothing to gate, and hiding the
 * panel would only hide the one place the interaction can be exercised.
 *
 * Input has to be ARMED (the header toggle) before the panel hears anything,
 * and it comes up disarmed. The same deliberate-act grammar as the pick-mode
 * arm buttons, for the same reason: the keyboard half is a window-level WASD
 * hook, and one that is always live would turn stray keys anywhere on the
 * dashboard into stick deflection — noise today, motion once a sender exists.
 * One switch for both input kinds rather than one per kind: pointer input is
 * scoped to the wells and near-harmless alone, so a separate toggle for it
 * would be a second decision with no second question behind it.
 *
 * Every number on this panel is in the cmd hue unconditionally: a joystick has
 * no measured half, so unlike the telemetry rail there is no live/cmd split to
 * draw — cf. the epistemics note in use-locomotion.ts.
 */
export function ManualControl({ className }: { className?: string }) {
  const [armed, setArmed] = React.useState(false);
  const stick = useJoystick(armed);

  return (
    <div className={cn(overlayPanel, "w-64 p-3", className)}>
      <header className="mb-3 flex h-4 items-center justify-between gap-2">
        <h2 className="instrument-label text-muted-foreground">Manual drive</h2>
        {/* Pressed = listening, in the cmd hue like every other operator
          * choice (LayerToggle, pick modes). Icon-only, and a joystick rather
          * than a gamepad or a power glyph: the gamepad would promise
          * controller support this panel does not have, and this button powers
          * the sticks below it, not a system — the glyph is the thing it
          * switches. The title and aria-label carry the words. */}
        <button
          type="button"
          aria-pressed={armed}
          aria-label="Arm manual drive input"
          onClick={() => setArmed((v) => !v)}
          title={
            armed
              ? "Stop capturing pointer and keyboard input"
              : "Capture pointer and keyboard (WASD / QE) input"
          }
          className={cn(
            "flex size-5 items-center justify-center rounded-sm border transition-colors",
            armed
              ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
              : "border-hairline text-muted-foreground hover:bg-elevated hover:text-foreground",
          )}
        >
          <JoystickIcon aria-hidden className="size-3" />
        </button>
      </header>
      <div className="flex items-start justify-center gap-4">
        <LabeledStick caption="Translate">
          <Thumbstick
            value={stick.left}
            active={stick.leftActive}
            disabled={!armed}
            hints={{ up: "W", down: "S", left: "A", right: "D" }}
            label="Translation stick"
            onPointer={(value) => stick.setPointer("left", value)}
          />
        </LabeledStick>
        <LabeledStick caption="Rotate">
          <Thumbstick
            value={stick.right}
            active={stick.rightActive}
            disabled={!armed}
            lockY
            hints={{ left: "Q", right: "E" }}
            label="Rotation stick"
            onPointer={(value) => stick.setPointer("right", value)}
          />
        </LabeledStick>
      </div>
      {/* Three columns rather than stacked Readout rows: Readout is a
        * label-left/value-right line built for the rail's tall stack, and three
        * of them would triple this panel's height for three 5-char numbers. */}
      <div className="mt-2.5 grid grid-cols-3 border-t border-hairline pt-2 text-center">
        <AxisReadout label="VX" value={stick.vector.vx} armed={armed} />
        <AxisReadout label="VY" value={stick.vector.vy} armed={armed} />
        <AxisReadout label="WZ" value={stick.vector.wz} armed={armed} />
      </div>
      <p className="mt-2 text-[11px] leading-tight text-muted-foreground">
        Visual only — nothing is sent to the robot yet.
      </p>
    </div>
  );
}

/** A stick over its one-word job. The key hints teach the fingers; this
 *  teaches the split — which hand translates and which rotates. */
function LabeledStick({
  caption,
  children,
}: {
  caption: string;
  children: React.ReactNode;
}) {
  return (
    <div className="flex flex-col items-center gap-1.5">
      {children}
      <span className="instrument-label text-[9px] text-muted-foreground">
        {caption}
      </span>
    </div>
  );
}

/** One commanded axis: condensed caps label over the normalized value. Falls
 *  to the muted hue while disarmed — a cmd-cyan number on a panel that is not
 *  listening would claim a command channel that is switched off. */
function AxisReadout({
  label,
  value,
  armed,
}: {
  label: string;
  value: number;
  armed: boolean;
}) {
  return (
    <div>
      <span className="instrument-label text-muted-foreground">{label}</span>
      <p
        className={cn(
          "readout text-[13px] font-medium",
          armed ? "text-signal-cmd" : "text-muted-foreground",
        )}
      >
        {formatAxis(value)}
      </p>
    </div>
  );
}
