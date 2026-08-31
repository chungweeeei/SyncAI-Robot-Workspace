"use client";

import * as React from "react";
import { GripHorizontalIcon, JoystickIcon } from "lucide-react";

import { overlayPanel } from "@/components/console/instrument";
import { Thumbstick } from "@/components/dashboard/thumbstick";
import { useJoystick, type TeleopVector } from "@/hooks/use-joystick";
import { useTeleopSender } from "@/hooks/use-teleop-sender";
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

function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

/**
 * Manual drive panel: two thumbsticks (left = planar translation vx/vy — the
 * G23 can strafe — right = rotation wz) over a readout of the commanded,
 * normalized vector. Pointer and keyboard input (W/S drive, Q/E strafe, A/D
 * turn), merged in useJoystick.
 *
 * **Armed = sending.** While armed, TeleopFooter keeps a WS teleop channel
 * open and streams `vectorRef` at 10 Hz to the backend, which clamps, scales
 * and publishes cmd_vel (see use-teleop-sender.ts / lib/ros/teleop-channel.ts
 * for the no-reconnect and watchdog reasoning). If the channel drops, the
 * panel disarms itself — a dead link must not leave the button claiming the
 * robot is listening. Still not gated on RobotMode === "MANUAL", now as a
 * deliberate call rather than a vacuous one: the backend refuses teleop while
 * an autonomous MOVE is executing and the footer surfaces that refusal, which
 * covers the actual hazard without hiding the panel.
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
 *
 * The panel is movable, by the header only. The callers anchor it bottom-right
 * over the point cloud, which is exactly where the cloud's near-field returns
 * land when the robot backs toward a wall — the operator has to be able to
 * shove the panel off whatever it is covering. The body is not a handle:
 * everything below the header is a control (wells that capture pointers, the
 * arm button), and a panel that slides when a thumb misses a well would turn
 * a bad grab into both a motion command and a moved panel. Dragging never
 * disarms or interrupts the stream — moving the panel mid-drive is the point.
 */
export function ManualControl({ className }: { className?: string }) {
  const [armed, setArmed] = React.useState(false);
  const stick = useJoystick(armed);
  // A WS event, not an effect body — the allowed place for setState.
  const handleDrop = React.useCallback(() => setArmed(false), []);

  const panelRef = React.useRef<HTMLDivElement>(null);
  /**
   * Displacement from the caller's anchor, applied as a translate so the
   * `right-3 bottom-3` positioning stays the callers' business — the panel
   * never rewrites top/left, so at {0,0} it sits exactly where it always did
   * and the offset survives the caller changing its corner.
   */
  const [offset, setOffset] = React.useState({ x: 0, y: 0 });
  /**
   * The active grab, thumbstick's gestureRef idiom: pointer identity plus
   * everything measured once at pointerdown. The clamp range comes from one
   * getBoundingClientRect against the *window* viewport — not the canvas the
   * callers anchor us in: the operator drags the panel off the point cloud
   * precisely because the cloud is where it is in the way, so the whole
   * screen is legal parking. pointermove never forces layout. A window
   * resize mid-offset can strand the panel past the edge until the next grab
   * re-measures and pulls it back in — double-click resets it for free.
   */
  const dragRef = React.useRef<{
    pointerId: number;
    originX: number;
    originY: number;
    baseX: number;
    baseY: number;
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
  } | null>(null);

  const onGrab = (event: React.PointerEvent<HTMLElement>) => {
    // The arm button lives inside the header; a press on it is a press on it.
    if (dragRef.current || (event.target as HTMLElement).closest("button")) return;
    const panel = panelRef.current;
    if (!panel) return;
    const rect = panel.getBoundingClientRect();
    dragRef.current = {
      pointerId: event.pointerId,
      originX: event.clientX,
      originY: event.clientY,
      baseX: offset.x,
      baseY: offset.y,
      // How far the panel may still travel in each direction before its edge
      // leaves the window — a panel dragged fully off-screen is unrecoverable.
      minX: offset.x - rect.left,
      maxX: offset.x + (window.innerWidth - rect.right),
      minY: offset.y - rect.top,
      maxY: offset.y + (window.innerHeight - rect.bottom),
    };
    event.currentTarget.setPointerCapture(event.pointerId);
  };

  const onDrag = (event: React.PointerEvent<HTMLElement>) => {
    const drag = dragRef.current;
    if (drag?.pointerId !== event.pointerId) return;
    setOffset({
      x: clamp(drag.baseX + event.clientX - drag.originX, drag.minX, drag.maxX),
      y: clamp(drag.baseY + event.clientY - drag.originY, drag.minY, drag.maxY),
    });
  };

  // Up, cancel and lost-capture all end the grab; idempotent via the ref
  // check because pointerup is followed by an implicit lostpointercapture.
  const onRelease = (event: React.PointerEvent<HTMLElement>) => {
    if (dragRef.current?.pointerId !== event.pointerId) return;
    dragRef.current = null;
  };

  return (
    <div
      ref={panelRef}
      style={{ transform: `translate(${offset.x}px, ${offset.y}px)` }}
      className={cn(overlayPanel, "w-64 p-3", className)}
    >
      {/* touch-none, or a touch drag scrolls the page and the browser answers
        * with pointercancel mid-gesture (thumbstick's rule). */}
      <header
        onPointerDown={onGrab}
        onPointerMove={onDrag}
        onPointerUp={onRelease}
        onPointerCancel={onRelease}
        onLostPointerCapture={onRelease}
        onDoubleClick={() => setOffset({ x: 0, y: 0 })}
        title="Drag to move · double-click to reset"
        className="mb-3 flex h-4 cursor-grab touch-none items-center justify-between gap-2 select-none active:cursor-grabbing"
      >
        <h2 className="instrument-label flex items-center gap-1.5 text-muted-foreground">
          {/* The grip is the affordance — a bare label row does not announce
            * that it can be grabbed. */}
          <GripHorizontalIcon aria-hidden className="size-3" />
          Manual drive
        </h2>
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
              : "Capture pointer and keyboard (WS / QE / AD) input"
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
            hints={{ up: "W", down: "S", left: "Q", right: "E" }}
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
            hints={{ left: "A", right: "D" }}
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
      {/* Mounting TeleopFooter only while armed is what opens/closes the
        * channel AND what resets its per-session state — the mount boundary
        * replaces any setState-in-effect reset (Next 16 lint). Disarmed gets
        * a same-height line so arming never reflows the panel. */}
      {armed ? (
        <TeleopFooter vectorRef={stick.vectorRef} onDrop={handleDrop} />
      ) : (
        <p className="mt-2 text-[11px] leading-tight text-muted-foreground">
          Disarmed — nothing is sent.
        </p>
      )}
    </div>
  );
}

/**
 * The live half of the panel: owns the WS channel for exactly as long as it
 * is mounted. Three states on one line: connecting (muted), streaming (cmd
 * hue — this IS the command channel now), and a backend refusal (warn hue;
 * e.g. teleop rejected while an autonomous move runs, decays after ~2 s of
 * the frames no longer being refused).
 */
function TeleopFooter({
  vectorRef,
  onDrop,
}: {
  vectorRef: React.RefObject<TeleopVector>;
  onDrop: () => void;
}) {
  const link = useTeleopSender(vectorRef, onDrop);

  if (link.phase === "connecting") {
    return (
      <p className="mt-2 text-[11px] leading-tight text-muted-foreground">
        Connecting to robot…
      </p>
    );
  }
  if (link.refusal !== null) {
    return (
      <p className="mt-2 text-[11px] leading-tight text-signal-warn">
        {link.refusal}
      </p>
    );
  }
  return (
    <p className="mt-2 text-[11px] leading-tight text-signal-cmd">
      Streaming to robot · 10 Hz
    </p>
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
