import type { TeleopVector } from "@/hooks/use-joystick";

/**
 * Wire codec for the WS teleop channel (`/api/v1/robot/teleop`) — pure
 * functions only, no React, no config.ts (which touches process.env/window),
 * so `node --experimental-strip-types` can exercise this file directly.
 *
 * The wire carries [-1, 1] per axis, which the backend clamps again and
 * publishes as-is (full stick = 1.0 m/s / 1.0 rad/s — the scale-down below
 * the clamp was deliberately dropped). If a ceiling comes back, it belongs
 * backend-side in RobotGateway.teleop_cmd_vel, not here.
 */

function clampAxis(value: number): number {
  if (!Number.isFinite(value)) return 0;
  return Math.min(1, Math.max(-1, value));
}

/**
 * One command frame. Clamps to [-1, 1] and collapses NaN/±Infinity to 0 —
 * useJoystick already guarantees the range, but this is the last gate before
 * the wire, so the guarantee is re-made where it is actually needed.
 */
export function encodeTeleopFrame(v: TeleopVector): string {
  return JSON.stringify({
    vx: clampAxis(v.vx),
    vy: clampAxis(v.vy),
    wz: clampAxis(v.wz),
  });
}

/**
 * The backend only ever speaks to say no: `{"error": "<message>"}` frames
 * (teleop refused during an autonomous move, or a malformed frame skipped).
 * Anything else — binary payloads, malformed JSON, an error that is not a
 * string — answers null and is skipped, mirroring the telemetry stream's
 * malformed-frame policy: an unexpected frame must not kill the channel.
 */
export function parseTeleopError(data: string | ArrayBuffer | Blob): string | null {
  if (typeof data !== "string") return null;
  try {
    const parsed: unknown = JSON.parse(data);
    if (
      typeof parsed === "object" &&
      parsed !== null &&
      typeof (parsed as { error?: unknown }).error === "string"
    ) {
      return (parsed as { error: string }).error;
    }
  } catch {
    // fall through
  }
  return null;
}

/** Precomputed stop frame, sent best-effort on close (see teleop-channel). */
export const ZERO_FRAME = encodeTeleopFrame({ vx: 0, vy: 0, wz: 0 });
