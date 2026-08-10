import { wsUrl } from "@/lib/api/config";
import {
  encodeTeleopFrame,
  parseTeleopError,
  ZERO_FRAME,
} from "@/lib/ros/teleop-frame";
import type { TeleopVector } from "@/hooks/use-joystick";

/**
 * Outbound teleop channel — deliberately NOT createReconnectingSocket.
 *
 * That helper's whole purpose is the 2 s auto-reconnect, which is exactly
 * what a motion channel must not have: when the link drops mid-drive, the
 * backend watchdog stops the robot, and a silent reconnect would resume
 * motion with no operator act. The read streams' failure mode is a stale
 * pixel; this channel's is a moving robot. So: one socket, no retry, and any
 * close/error surfaces as `onDown` so the owner can drop to disarmed —
 * re-arming is one deliberate click, matching the panel's arm grammar.
 *
 * Send cadence is 10 Hz against the backend's 0.5 s stale-input watchdog
 * (five ticks of margin). Zero vectors are sent like any other — "stick
 * centered" is a command, and the stream going quiet is what the watchdog
 * treats as a fault. A background-throttled tab (intervals clamped to >= 1 s)
 * therefore trips the watchdog and stops the robot: the safe direction, and
 * useJoystick's window-blur handler zeroes the vector on alt-tab anyway.
 */

const SEND_PERIOD_MS = 100;

export interface TeleopChannelHandlers {
  onOpen?: () => void;
  /** A backend `{"error": ...}` frame, e.g. "autonomous move in progress". */
  onRefusal?: (message: string) => void;
  /** Socket closed or errored. NOT called after an explicit close(). */
  onDown?: () => void;
}

export interface TeleopChannel {
  /** Best-effort zero frame, then close. Idempotent. */
  close: () => void;
}

export function createTeleopChannel(
  vectorRef: { readonly current: TeleopVector },
  handlers: TeleopChannelHandlers,
  path = "/api/v1/robot/teleop",
): TeleopChannel {
  let sendTimer: ReturnType<typeof setInterval> | null = null;
  let closed = false;

  const ws = new WebSocket(wsUrl(path));

  const stopSending = () => {
    if (sendTimer !== null) clearInterval(sendTimer);
    sendTimer = null;
  };

  ws.onopen = () => {
    if (closed) return;
    // First frame immediately — the first command must not wait a tick.
    ws.send(encodeTeleopFrame(vectorRef.current));
    sendTimer = setInterval(() => {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(encodeTeleopFrame(vectorRef.current));
      }
    }, SEND_PERIOD_MS);
    handlers.onOpen?.();
  };

  ws.onmessage = (ev) => {
    const message = parseTeleopError(ev.data);
    if (message !== null) handlers.onRefusal?.(message);
  };

  // Unlike createReconnectingSocket, the final transition after an explicit
  // close() is suppressed: the owner closes *because* it already disarmed,
  // so echoing onDown back would be noise (there is no status badge behind
  // this channel that could go stale).
  ws.onclose = () => {
    stopSending();
    if (!closed) handlers.onDown?.();
  };
  ws.onerror = () => {
    // onclose follows onerror and carries the onDown; this handler exists so
    // the browser doesn't log an unhandled error event.
  };

  return {
    close: () => {
      if (closed) return;
      closed = true;
      stopSending();
      if (ws.readyState === WebSocket.OPEN) {
        // Defense in depth, not the safety mechanism — the backend zeroes on
        // disconnect and on watchdog regardless. The literal zero (not
        // vectorRef.current) is deliberate: deterministic no matter how the
        // disarm effects were ordered.
        ws.send(ZERO_FRAME);
      }
      ws.close();
    },
  };
}
