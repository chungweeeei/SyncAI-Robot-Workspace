"use client";

import * as React from "react";

import {
  createSavedTask,
  deleteSavedTask,
  listSavedTasks,
  updateSavedTask,
  type SavedTask,
  type SavedTaskChanges,
  type SavedTaskDraft,
} from "@/lib/api/saved-task";

export type SavedTasksStatus = "loading" | "ok" | "error";

export interface UseSavedTasks {
  /** Every saved task on the robot, by name. Scoping is the caller's business. */
  tasks: SavedTask[];
  status: SavedTasksStatus;
  /** The load failure, or the most recent write failure. Rendered verbatim. */
  error: string | null;
  /** True while a create / update / delete is in flight. */
  busy: boolean;
  /** The stored row, or null if the request failed (see `error`). */
  create: (draft: SavedTaskDraft) => Promise<SavedTask | null>;
  update: (id: string, changes: SavedTaskChanges) => Promise<SavedTask | null>;
  /** True when the row is gone. */
  remove: (id: string) => Promise<boolean>;
  /** Re-read, e.g. after a vertex moved on another screen. */
  refresh: () => void;
  clearError: () => void;
}

/**
 * The operator's library of re-dispatchable tasks, written through on every change.
 *
 * Shaped after useMapVertices rather than useSchedules, and the difference is the
 * response body: POST and PUT here answer with the stored row, so the list is
 * spliced instead of refetched — a GET would cost a round trip to learn nothing.
 * (useSchedules must refetch because its writes answer `{id, message}` and the
 * next run times are computed by Temporal.)
 *
 * `refresh` exists even though nothing here mutates behind our back, because
 * something else does: a saved MOVE step's coordinates are resolved server-side
 * against the vertex's *current* pose, so editing a vertex on /maps changes what
 * these rows say. Navigation remounts the page and re-reads, and this is the
 * in-page escape hatch.
 */
export function useSavedTasks(): UseSavedTasks {
  const [tasks, setTasks] = React.useState<SavedTask[] | null>(null);
  const [status, setStatus] = React.useState<SavedTasksStatus>("loading");
  const [busy, setBusy] = React.useState(false);
  const [nonce, setNonce] = React.useState(0);

  /**
   * Load and write failures are held apart, then merged on the way out.
   *
   * A single slot would work here — nothing refreshes after a write, so a
   * successful reload cannot clobber a write's message the way it can in
   * useSchedules — but keeping them separate means a stale write error does not
   * survive a successful refresh either.
   */
  const [loadError, setLoadError] = React.useState<string | null>(null);
  const [writeError, setWriteError] = React.useState<string | null>(null);

  React.useEffect(() => {
    let active = true;
    const abort = new AbortController();

    listSavedTasks(abort.signal)
      .then((data) => {
        if (!active) return;
        setTasks(data);
        setStatus("ok");
        setLoadError(null);
      })
      .catch((cause: unknown) => {
        if (!active || abort.signal.aborted) return;
        setStatus("error");
        setLoadError(
          cause instanceof Error ? cause.message : "Failed to load the saved tasks.",
        );
      });

    return () => {
      active = false;
      abort.abort();
    };
  }, [nonce]);

  const refresh = React.useCallback(() => setNonce((n) => n + 1), []);

  /**
   * Run one write, holding `busy` and turning a rejection into `error`.
   *
   * Rejections are swallowed rather than rethrown because every caller is wired
   * straight to an onClick — a rethrow would be an unhandled rejection, and the
   * components read the outcome off `error` and the returned value.
   */
  const run = React.useCallback(
    async <T,>(action: () => Promise<T>): Promise<T | null> => {
      setBusy(true);
      setWriteError(null);
      try {
        return await action();
      } catch (cause) {
        setWriteError(cause instanceof Error ? cause.message : String(cause));
        return null;
      } finally {
        setBusy(false);
      }
    },
    [],
  );

  const create = React.useCallback(
    async (draft: SavedTaskDraft) => {
      const created = await run(() => createSavedTask(draft));
      if (created) {
        // Inserted in the server's order (name, then created_at) rather than
        // appended, so the row does not jump on the next refresh.
        setTasks((current) => sortByName([...(current ?? []), created]));
      }
      return created;
    },
    [run],
  );

  const update = React.useCallback(
    async (id: string, changes: SavedTaskChanges) => {
      const updated = await run(() => updateSavedTask(id, changes));
      if (updated) {
        setTasks((current) =>
          sortByName(
            (current ?? []).map((task) => (task.id === id ? updated : task)),
          ),
        );
      }
      return updated;
    },
    [run],
  );

  const remove = React.useCallback(
    async (id: string) => {
      const done = await run(async () => {
        await deleteSavedTask(id);
        return true as const;
      });
      if (done) {
        setTasks((current) => (current ?? []).filter((task) => task.id !== id));
      }
      return done === true;
    },
    [run],
  );

  const clearError = React.useCallback(() => setWriteError(null), []);

  return {
    tasks: tasks ?? [],
    status,
    error: writeError ?? loadError,
    busy,
    create,
    update,
    remove,
    refresh,
    clearError,
  };
}

/** Mirrors the backend's `ORDER BY name, created_at` so a splice stays in place. */
function sortByName(tasks: SavedTask[]): SavedTask[] {
  return [...tasks].sort(
    (a, b) => a.name.localeCompare(b.name) || a.created_at.localeCompare(b.created_at),
  );
}
