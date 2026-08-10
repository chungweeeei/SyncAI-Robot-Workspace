"use client";

import * as React from "react";

import { createTeleopChannel } from "@/lib/ros/teleop-channel";
import type { TeleopVector } from "@/hooks/use-joystick";

/** How long a backend refusal stays on screen after the frames stop being
 *  refused. The backend answers per refused frame (~10 Hz) and sends no
 *  "refusal ended" signal, so silence-plus-TTL is the only way to clear. */
const REFUSAL_TTL_MS = 2000;

export type TeleopLink =
  | { phase: "connecting" }
  | { phase: "sending"; refusal: string | null };

/**
 * Opens the teleop channel for the lifetime of the calling component.
 *
 * Lifecycle is the mount, not an `armed` parameter, on purpose: the owner
 * mounts the caller only while armed, so the initial useState IS the
 * per-session reset — no setState in an effect body (the Next 16 lint rule),
 * and no stale refusal from the previous session. Every subsequent setState
 * happens inside a WebSocket or timer callback, which the rule allows.
 *
 * `onDrop` fires when the socket goes down (never after our own cleanup);
 * the owner is expected to disarm in response. It is read through a
 * latest-value ref so the channel effect depends only on the stable
 * vectorRef — a mid-session identity change must not reopen the socket.
 *
 * Re-render profile: none while driving. The 10 Hz loop reads
 * vectorRef.current (exactly the contract that ref exists for — see
 * use-joystick.ts); state changes only on open, on a refusal edge, and on
 * the TTL clearing it.
 */
export function useTeleopSender(
  vectorRef: React.RefObject<TeleopVector>,
  onDrop: () => void,
): TeleopLink {
  const [link, setLink] = React.useState<TeleopLink>({ phase: "connecting" });

  const onDropRef = React.useRef(onDrop);
  React.useEffect(() => {
    onDropRef.current = onDrop;
  }, [onDrop]);

  React.useEffect(() => {
    let refusalTimer: ReturnType<typeof setTimeout> | null = null;

    const channel = createTeleopChannel(vectorRef, {
      onOpen: () => setLink({ phase: "sending", refusal: null }),
      onRefusal: (message) => {
        // The backend repeats the error per refused frame (~10 Hz); only a
        // *changed* message may allocate fresh state, or the panel would
        // re-render at frame rate for nothing. The TTL restarts either way.
        setLink((prev) =>
          prev.phase === "sending" && prev.refusal === message
            ? prev
            : { phase: "sending", refusal: message },
        );
        if (refusalTimer !== null) clearTimeout(refusalTimer);
        refusalTimer = setTimeout(() => {
          setLink((prev) =>
            prev.phase === "sending" ? { phase: "sending", refusal: null } : prev,
          );
        }, REFUSAL_TTL_MS);
      },
      onDown: () => onDropRef.current(),
    });

    return () => {
      if (refusalTimer !== null) clearTimeout(refusalTimer);
      channel.close();
    };
  }, [vectorRef]);

  return link;
}
