"use client";

import * as React from "react";

import { DEADZONE, type StickValue } from "@/hooks/use-joystick";
import { cn } from "@/lib/utils";

/** Well diameter (size-24) and knob diameter (size-7), in px. The travel
 *  radius keeps the knob's edge inside the well at full deflection. */
const WELL = 96;
const KNOB = 28;
const TRAVEL = (WELL - KNOB) / 2;
/** Ring inside which the knob centre produces no command — drawn from the same
 *  DEADZONE the math uses, plus the knob's own radius so the *knob* visually
 *  clears the ring exactly when the command starts. */
const DEADZONE_RING = DEADZONE * TRAVEL * 2 + KNOB;

/**
 * One virtual thumbstick: a circular well, a knob, and the key hints for the
 * axes it drives. Purely presentational and controlled — the deflection comes
 * in as `value`, which is what lets a held key move the knob for free.
 *
 * DOM + CSS transform rather than canvas, deliberately: two circles have no
 * per-pixel content, a translate is GPU-composited and costs nothing per
 * frame, and the theme tokens (hairline, signal-cmd) apply directly instead of
 * needing a palette table the way grid-canvas does. Canvas is for the map
 * surfaces, not for chrome.
 *
 * The circular well is this console's one deliberate break from "instrument
 * faces are rectilinear" (globals.css): the clamp on the command *is* a
 * circle, and a rectangular face would lie about where the knob can go.
 *
 * Past the deadzone a needle is drawn from centre to knob. It is not
 * decoration: direction and length ARE the command, and the needle is the one
 * reading of it that works in peripheral vision while the eyes are on the
 * point cloud — the readouts below need a glance, the needle does not.
 */
export function Thumbstick({
  value,
  active,
  disabled = false,
  lockY = false,
  hints,
  label,
  onPointer,
}: {
  /** Deflection to display, in travel units (the hook clamps it). */
  value: StickValue;
  /** True while this stick is commanding — lights the cmd hue. */
  active: boolean;
  /** Input disarmed: dim the well and refuse pointers (Segmented's idiom). */
  disabled?: boolean;
  /** Rotation stick: draw a horizontal guide instead of up/down hints. */
  lockY?: boolean;
  /** Key letters shown at the well's compass points, teaching the shortcut. */
  hints: { up?: string; down?: string; left: string; right: string };
  label: string;
  /** Raw deflection while a pointer drags; null on release or cancel. */
  onPointer: (value: StickValue | null) => void;
}) {
  /**
   * The active gesture, in a ref because it changes at pointer rate: which
   * pointer owns the stick, and the well centre cached from one
   * getBoundingClientRect — the well is fixed-size, so one read per gesture is
   * enough and pointermove never forces layout (grid-canvas's rectRef rule).
   */
  const gestureRef = React.useRef<{
    pointerId: number;
    cx: number;
    cy: number;
  } | null>(null);
  /**
   * Mirrors "gestureRef is set" into render. It exists to gate the knob's
   * transition class: dragging must be 1:1 (an eased knob feels like latency),
   * while a release should glide back instead of teleporting. Keyboard moves
   * happen while not dragging, so they get the same short ease, which reads as
   * the snap it is without the flicker a hard jump would be.
   */
  const [dragging, setDragging] = React.useState(false);

  const report = React.useCallback(
    (event: React.PointerEvent) => {
      const gesture = gestureRef.current;
      if (!gesture) return;
      onPointer({
        x: (event.clientX - gesture.cx) / TRAVEL,
        y: (event.clientY - gesture.cy) / TRAVEL,
      });
    },
    [onPointer],
  );

  // Disarmed mid-drag (a second finger on the arm toggle while this one holds
  // the stick): the capture is on this element, so ending the gesture is this
  // component's job — the hook is already deaf, this stops the knob from
  // silently re-engaging the moment input is re-armed with the finger still
  // down.
  React.useEffect(() => {
    if (!disabled || !gestureRef.current) return;
    gestureRef.current = null;
    setDragging(false);
    onPointer(null);
  }, [disabled, onPointer]);

  const onPointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    // One pointer per stick; a second finger on the same well is ignored. Two
    // fingers on two *different* sticks work by construction — each well
    // captures its own pointerId on its own element.
    if (disabled || gestureRef.current) return;
    const rect = event.currentTarget.getBoundingClientRect();
    gestureRef.current = {
      pointerId: event.pointerId,
      cx: rect.left + rect.width / 2,
      cy: rect.top + rect.height / 2,
    };
    event.currentTarget.setPointerCapture(event.pointerId);
    setDragging(true);
    // Report from the press itself: a stick that waits for the first move to
    // deflect feels dead, and a tap near the rim is a deliberate command.
    report(event);
  };

  const onPointerMove = (event: React.PointerEvent<HTMLDivElement>) => {
    if (gestureRef.current?.pointerId !== event.pointerId) return;
    report(event);
  };

  // Up, cancel and lost-capture all mean the same thing: the hand is gone, the
  // stick springs back (or falls to the keyboard value — the hook decides).
  // Idempotent via the ref check, because pointerup is followed by an implicit
  // lostpointercapture.
  const endGesture = (event: React.PointerEvent<HTMLDivElement>) => {
    if (gestureRef.current?.pointerId !== event.pointerId) return;
    gestureRef.current = null;
    setDragging(false);
    onPointer(null);
  };

  const engaged = active && Math.hypot(value.x, value.y) >= DEADZONE;

  return (
    <div
      role="group"
      aria-label={label}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={endGesture}
      onPointerCancel={endGesture}
      onLostPointerCapture={endGesture}
      // touch-none, or a touch drag scrolls the page and the browser answers
      // with pointercancel mid-gesture (see grid-canvas). A long-press must not
      // open the context menu either — on a robot console that pause reads as
      // the stick dying.
      onContextMenu={(event) => event.preventDefault()}
      className={cn(
        "relative shrink-0 touch-none rounded-full border border-hairline bg-elevated/50 select-none size-24",
        // Recessed, not flat: the well is the one thing on the console the
        // operator's finger goes *into*, and the inset is what says so.
        "inset-shadow-sm",
        // Dim like a disabled Segmented: the container carries the one
        // opacity, the knob and hints just stop reacting.
        disabled ? "opacity-40" : "cursor-pointer",
      )}
    >
      {/* Axis guides, inset far enough (inset-4) to clear the key hints at the
        * compass points. The rotation stick gets no vertical line — a guide for
        * an axis the stick refuses would be the face lying about the control. */}
      <span
        aria-hidden
        className="absolute top-1/2 right-4 left-4 h-px bg-hairline"
      />
      {!lockY && (
        <span
          aria-hidden
          className="absolute top-4 bottom-4 left-1/2 w-px bg-hairline"
        />
      )}
      {/* Deadzone ring. Lit past the ring = a command is live — the same
        * grammar as a pressed LayerToggle, applied to a radius. */}
      <span
        aria-hidden
        style={{ width: DEADZONE_RING, height: DEADZONE_RING }}
        className={cn(
          "absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 rounded-full border",
          engaged ? "border-signal-cmd/50" : "border-hairline",
        )}
      />
      {hints.up && <Hint className="top-1 left-1/2 -translate-x-1/2">{hints.up}</Hint>}
      {hints.down && (
        <Hint className="bottom-1 left-1/2 -translate-x-1/2">{hints.down}</Hint>
      )}
      <Hint className="top-1/2 left-1.5 -translate-y-1/2">{hints.left}</Hint>
      <Hint className="top-1/2 right-1.5 -translate-y-1/2">{hints.right}</Hint>
      {/* The command needle — see the component doc. Under the knob, so the
        * knob reads as the hand and the needle as the instrument. */}
      {engaged && (
        <span
          aria-hidden
          style={{
            width: Math.hypot(value.x, value.y) * TRAVEL,
            transform: `rotate(${Math.atan2(value.y, value.x)}rad)`,
          }}
          className="absolute top-1/2 left-1/2 h-px origin-left bg-signal-cmd/70"
        />
      )}
      <span
        aria-hidden
        style={{
          transform: `translate(calc(-50% + ${value.x * TRAVEL}px), calc(-50% + ${value.y * TRAVEL}px))`,
        }}
        className={cn(
          "absolute top-1/2 left-1/2 flex items-center justify-center rounded-full border bg-panel shadow-sm size-7",
          active
            ? "border-signal-cmd bg-signal-cmd/20 shadow-[0_0_10px_0] shadow-signal-cmd/40"
            : "border-hairline",
          // Eased only when no pointer is captured — see `dragging`.
          !dragging && "transition-transform duration-100 ease-out motion-reduce:transition-none",
        )}
      >
        {/* Centre dot: the knob's own crosshair reference, so a deflection is
          * readable against the guides even before the needle appears. */}
        <span
          aria-hidden
          className={cn(
            "rounded-full size-[3px]",
            active ? "bg-signal-cmd" : "bg-hairline",
          )}
        />
      </span>
    </div>
  );
}

/** One key letter at a compass point of the well. */
function Hint({
  className,
  children,
}: {
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <span
      aria-hidden
      className={cn(
        "instrument-label pointer-events-none absolute text-[9px] text-muted-foreground",
        className,
      )}
    >
      {children}
    </span>
  );
}
