import * as React from "react";

import { fetchMapGrid } from "@/lib/api/map";
import type { GridSession } from "@/lib/map/session";
import { createGridSession, disposeGridSession } from "@/lib/map/session";

export type MapGridStatus = "loading" | "ok" | "error";

export interface UseMapGrid {
  /** The editable session, or null while loading and on failure. */
  session: GridSession | null;
  status: MapGridStatus;
  /** Why the load failed — shown verbatim, the messages are operator-facing. */
  error: string | null;
}

/**
 * One name's load outcome. Stored together so the hook can tell a result for the
 * *current* name from a leftover result for the previous one by comparing, rather
 * than by clearing state at the top of the effect — a synchronous setState in an
 * effect body is a cascading render, and the compiler lint rejects it.
 */
interface Loaded {
  name: string;
  session: GridSession | null;
  error: string | null;
}

/**
 * Loads one map's grid and wraps it in a session.
 *
 * The session (buffer + mirror canvas) is built inside the effect, never during
 * render: it touches `document.createElement`, and this page is prerendered. That
 * also makes a React 19 strict-mode double mount harmless — the second run
 * allocates a fresh buffer and the first is disposed on the way out, so no effect
 * ever mutates a grid it did not create.
 */
export function useMapGrid(name: string): UseMapGrid {
  const [loaded, setLoaded] = React.useState<Loaded | null>(null);

  React.useEffect(() => {
    let active = true;
    let created: GridSession | null = null;
    const abort = new AbortController();

    fetchMapGrid(name, abort.signal)
      .then(({ summary, grid }) => {
        if (!active || !summary.grid) return;
        created = createGridSession(name, summary.grid, grid);
        setLoaded({ name, session: created, error: null });
      })
      .catch((cause: unknown) => {
        if (!active || abort.signal.aborted) return;
        setLoaded({
          name,
          session: null,
          error: cause instanceof Error ? cause.message : "Failed to load the map.",
        });
      });

    return () => {
      active = false;
      abort.abort();
      if (created) disposeGridSession(created);
    };
  }, [name]);

  const current = loaded?.name === name ? loaded : null;

  return {
    session: current?.session ?? null,
    status: current ? (current.session ? "ok" : "error") : "loading",
    error: current?.error ?? null,
  };
}
