"use client";

import * as React from "react";

/**
 * Normalized teleop command, REP-103 body-frame axes: +vx forward, +vy left,
 * +wz counter-clockwise. Every component is in [-1, 1] on purpose — scaling to
 * m/s and rad/s is the sender's job, because the robot's velocity limits live
 * next to whatever will publish cmd_vel, not in a UI component that would
 * otherwise need re-editing every time a limit changes.
 */
export interface TeleopVector {
  vx: number;
  vy: number;
  wz: number;
}

/** A stick deflection in screen space: x grows right, y grows DOWN. */
export interface StickValue {
  x: number;
  y: number;
}

export type StickId = "left" | "right";

/**
 * Radial deadzone as a fraction of full travel. Exported so the Thumbstick can
 * draw the ring at the same radius the math uses — a ring that only decorates
 * would drift from the truth the first time this constant moves.
 */
export const DEADZONE = 0.12;

export interface JoystickState {
  /**
   * Knob display positions: clamped, but PRE-deadzone. The knob follows the
   * finger; the deadzone belongs to the command, not to the hand. Left stick is
   * clamped to the unit circle, right stick to the x axis (its y is always 0).
   */
  left: StickValue;
  right: StickValue;
  /** Pointer captured OR keys deflecting that stick — drives the cmd styling. */
  leftActive: boolean;
  rightActive: boolean;
  /** POST-deadzone commanded vector, rAF-coalesced — what the readouts show. */
  vector: TeleopVector;
  /**
   * Always-current mirror of `vector`, updated synchronously on every input
   * event (not just per animation frame — rAF stops in a hidden tab, and a
   * stale non-zero command surviving an alt-tab is exactly the bug the blur
   * handler below exists to prevent). This ref is the contract for the future
   * cmd_vel sender: a send loop has its own clock (say a 10 Hz interval), so it
   * reads this without ever re-rendering anything. A callback prop at pointer
   * rate was rejected — it would push send-rate policy onto every consumer.
   */
  vectorRef: React.RefObject<TeleopVector>;
  /** Called by each Thumbstick with its raw deflection; null on release. */
  setPointer: (stick: StickId, value: StickValue | null) => void;
}

interface Snapshot {
  left: StickValue;
  right: StickValue;
  leftActive: boolean;
  rightActive: boolean;
  vector: TeleopVector;
}

const AT_REST: Snapshot = {
  left: { x: 0, y: 0 },
  right: { x: 0, y: 0 },
  leftActive: false,
  rightActive: false,
  vector: { vx: 0, vy: 0, wz: 0 },
};

/**
 * Physical key codes, not `event.key`: ZQSD on an AZERTY board should drive by
 * position, the way every game does it. Left stick is WASD, right stick Q/E —
 * screen-space signs, so "left" is negative x and "up" is negative y.
 */
const KEY_AXES: Record<string, { stick: StickId; axis: "x" | "y"; sign: 1 | -1 }> = {
  KeyW: { stick: "left", axis: "y", sign: -1 },
  KeyS: { stick: "left", axis: "y", sign: 1 },
  KeyA: { stick: "left", axis: "x", sign: -1 },
  KeyD: { stick: "left", axis: "x", sign: 1 },
  KeyQ: { stick: "right", axis: "x", sign: -1 },
  KeyE: { stick: "right", axis: "x", sign: 1 },
};

/**
 * The left stick's reachable set is a disc, so the clamp is radial — clamping
 * x and y separately would let a diagonal command √2 times the straight-line
 * maximum. The right stick is one-dimensional by design (it commands wz only),
 * so its y is discarded rather than clamped.
 */
function clampStick(stick: StickId, raw: StickValue): StickValue {
  if (stick === "right") {
    return { x: Math.min(1, Math.max(-1, raw.x)), y: 0 };
  }
  const m = Math.hypot(raw.x, raw.y);
  return m > 1 ? { x: raw.x / m, y: raw.y / m } : { x: raw.x, y: raw.y };
}

/**
 * Radial deadzone with rescale, so the command is continuous from zero: a plain
 * cutoff would make the smallest possible command DEADZONE-sized, which on a
 * real robot is a visible lurch the moment the stick leaves the ring.
 */
function applyDeadzone(value: StickValue): StickValue {
  const m = Math.hypot(value.x, value.y);
  if (m < DEADZONE) return { x: 0, y: 0 };
  const scale = (m - DEADZONE) / (1 - DEADZONE) / m;
  return { x: value.x * scale, y: value.y * scale };
}

/**
 * Dual-thumbstick teleop state: left stick = planar translation, right stick =
 * rotation, pointer and keyboard merged into one commanded vector.
 *
 * **Everything in here is COMMANDED, nothing is measured** — cf. the epistemics
 * note in use-locomotion.ts. That is why the panel renders all of it in the cmd
 * hue: these numbers are what the operator is asking for, and once a sender
 * exists the robot's answer will arrive separately, through telemetry, in the
 * live hue. For now nothing is sent anywhere at all — this hook is the visual
 * half, built so the sender can be added without touching it.
 *
 * Input merging is per-stick, pointer wins: while a stick's pointer is
 * captured, that whole stick is pointer-owned and its keys are ignored;
 * otherwise the stick shows the keyboard deflection (opposing keys sum to
 * zero). Keyboard deflection is instant full-scale — a slew ramp was considered
 * and left for the sender, which is where acceleration limits belong.
 *
 * Rendering follows grid-canvas's rule: raw inputs live in refs, and one
 * rAF-coalesced publish turns them into at most one setState per frame. The
 * hook is meant to be called inside the small overlay panel, so pointer-rate
 * updates re-render a few hundred pixels of DOM and never the viewport.
 *
 * `enabled` is the arm switch, and while it is false the hook is deaf: no
 * keyboard listeners are attached and setPointer is a no-op. Deaf here rather
 * than hidden in the panel, because the dangerous half is the keyboard — a
 * window-level WASD hook that is always live turns typing-adjacent muscle
 * memory anywhere on the dashboard into stick deflection, which is merely
 * confusing today and becomes motion the day a sender lands. Disarming also
 * zeroes whatever input was live at that moment: an armed command must not
 * survive the operator saying stop listening.
 */
export function useJoystick(enabled: boolean): JoystickState {
  const pointerRef = React.useRef<Record<StickId, StickValue | null>>({
    left: null,
    right: null,
  });
  const keysRef = React.useRef<Set<string>>(new Set());
  const rafRef = React.useRef<number | null>(null);
  const snapRef = React.useRef<Snapshot>(AT_REST);
  const vectorRef = React.useRef<TeleopVector>(AT_REST.vector);
  const [snapshot, setSnapshot] = React.useState<Snapshot>(AT_REST);

  const publish = React.useCallback(() => {
    const compute = (stick: StickId): { value: StickValue; active: boolean } => {
      const pointer = pointerRef.current[stick];
      if (pointer) return { value: clampStick(stick, pointer), active: true };
      let x = 0;
      let y = 0;
      for (const [code, key] of Object.entries(KEY_AXES)) {
        if (key.stick !== stick || !keysRef.current.has(code)) continue;
        if (key.axis === "x") x += key.sign;
        else y += key.sign;
      }
      return { value: clampStick(stick, { x, y }), active: x !== 0 || y !== 0 };
    };

    const left = compute("left");
    const right = compute("right");
    const dzLeft = applyDeadzone(left.value);
    const dzRight = applyDeadzone(right.value);
    snapRef.current = {
      left: left.value,
      right: right.value,
      leftActive: left.active,
      rightActive: right.active,
      // Screen space → body frame: stick up (-y) is forward, stick left (-x)
      // is +vy (REP-103 y points left) and, on the right stick, +wz (CCW).
      vector: { vx: -dzLeft.y, vy: -dzLeft.x, wz: -dzRight.x },
    };
    // Synchronous, ahead of the rAF: see the vectorRef doc above.
    vectorRef.current = snapRef.current.vector;

    if (rafRef.current !== null) return;
    rafRef.current = requestAnimationFrame(() => {
      rafRef.current = null;
      setSnapshot(snapRef.current);
    });
  }, []);

  const setPointer = React.useCallback(
    (stick: StickId, value: StickValue | null) => {
      // Disarmed = deaf, even to a pointer already captured: a touch that was
      // dragging a stick when the arm toggle was hit keeps delivering moves
      // (its capture is on the Thumbstick's element), and they must land here
      // as nothing.
      if (!enabled) return;
      pointerRef.current[stick] = value;
      publish();
    },
    [enabled, publish],
  );

  React.useEffect(() => {
    if (!enabled) {
      // Zero out whatever was live at the moment of disarm — held keys, a
      // mid-drag deflection — so the readouts (and vectorRef, which a sender
      // will trust) drop to rest instead of freezing at the last command.
      if (
        keysRef.current.size > 0 ||
        pointerRef.current.left ||
        pointerRef.current.right
      ) {
        keysRef.current.clear();
        pointerRef.current.left = null;
        pointerRef.current.right = null;
        publish();
      }
      return;
    }

    // Copied from map-grid-editor: single-letter shortcuts are exactly what
    // silently eats typing, and dialogs (VertexMoveDialog) open over this view.
    const isTypingTarget = (target: EventTarget | null) => {
      const element = target as HTMLElement | null;
      return Boolean(
        element &&
          (element.isContentEditable ||
            /^(INPUT|TEXTAREA|SELECT)$/.test(element.tagName ?? "")),
      );
    };

    const onKeyDown = (event: KeyboardEvent) => {
      if (isTypingTarget(event.target)) return;
      // Chords stay the browser's: Ctrl+W must close the tab, not drive
      // forward. Only keydown checks this — a keyup must always release its
      // key, or W-down / Ctrl-down / W-up would leave the robot commanded.
      if (event.ctrlKey || event.metaKey || event.altKey) return;
      if (!(event.code in KEY_AXES) || event.repeat) return;
      keysRef.current.add(event.code);
      publish();
    };

    const onKeyUp = (event: KeyboardEvent) => {
      if (!keysRef.current.delete(event.code)) return;
      publish();
    };

    // A key held across an alt-tab must not leave a standing deflection —
    // same reasoning as map-grid-editor's spacePan reset on blur.
    const onBlur = () => {
      if (keysRef.current.size === 0) return;
      keysRef.current.clear();
      publish();
    };

    // On window, not the panel: the sticks are never focused, and driving
    // should work the moment the dashboard is on screen.
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
    };
  }, [enabled, publish]);

  React.useEffect(
    () => () => {
      if (rafRef.current !== null) cancelAnimationFrame(rafRef.current);
    },
    [],
  );

  return { ...snapshot, vectorRef, setPointer };
}
