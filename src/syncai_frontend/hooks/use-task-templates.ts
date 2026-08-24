"use client";

import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import { queryKeys } from "@/lib/api/query-keys";
import {
  createTaskTemplate,
  deleteTaskTemplate,
  listTaskTemplates,
  updateTaskTemplate,
  type TaskTemplate,
  type TaskTemplateChanges,
  type TaskTemplateDraft,
} from "@/lib/api/task-template";

export type TaskTemplatesStatus = "loading" | "ok" | "error";

export interface UseTaskTemplates {
  /** Every template on the robot, by name. Scoping is the caller's business. */
  templates: TaskTemplate[];
  status: TaskTemplatesStatus;
  /** The load failure, or the most recent write failure. Rendered verbatim. */
  error: string | null;
  /** True while a create / update / delete is in flight. */
  busy: boolean;
  /** The stored row, or null if the request failed (see `error`). */
  create: (draft: TaskTemplateDraft) => Promise<TaskTemplate | null>;
  update: (id: string, changes: TaskTemplateChanges) => Promise<TaskTemplate | null>;
  /** True when the row is gone. */
  remove: (id: string) => Promise<boolean>;
  /** Re-read, e.g. after a vertex moved on another screen. */
  refresh: () => void;
  clearError: () => void;
}

/**
 * The operator's library of re-dispatchable templates, written through on every
 * change.
 *
 * Shaped after useMapVertices rather than useSchedules, and the difference is the
 * response body: POST and PUT here answer with the stored row, so the list is
 * spliced instead of refetched — a GET would cost a round trip to learn nothing.
 * (useSchedules must refetch because its writes answer `{id, message}` and the
 * next run times are computed by Temporal.)
 *
 * `refresh` exists even though nothing here mutates behind our back, because
 * something else does: a stored MOVE step's coordinates are resolved server-side
 * against the vertex's *current* pose, so editing a vertex on /maps changes what
 * these rows say. Navigation remounts the page and re-reads, and this is the
 * in-page escape hatch. Write failures live in their own slot rather than the
 * query's, so the reload a refresh triggers cannot clear a sentence the
 * operator still needs to read.
 */
export function useTaskTemplates(): UseTaskTemplates {
  const queryClient = useQueryClient();
  const query = useQuery({
    queryKey: queryKeys.taskTemplates,
    queryFn: ({ signal }) => listTaskTemplates(signal),
  });

  const [busy, setBusy] = React.useState(false);
  const [writeError, setWriteError] = React.useState<string | null>(null);

  const refresh = React.useCallback(
    () =>
      void queryClient.invalidateQueries({ queryKey: queryKeys.taskTemplates }),
    [queryClient],
  );

  /** Rewrite the cached list; a no-op until the load has answered. */
  const setTemplates = React.useCallback(
    (next: (current: TaskTemplate[]) => TaskTemplate[]) => {
      queryClient.setQueryData<TaskTemplate[]>(
        queryKeys.taskTemplates,
        (current) => (current ? next(current) : current),
      );
    },
    [queryClient],
  );

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
    async (draft: TaskTemplateDraft) => {
      const created = await run(() => createTaskTemplate(draft));
      if (created) {
        // Inserted in the server's order (name, then created_at) rather than
        // appended, so the row does not jump on the next refresh.
        setTemplates((current) => sortByName([...current, created]));
      }
      return created;
    },
    [run, setTemplates],
  );

  const update = React.useCallback(
    async (id: string, changes: TaskTemplateChanges) => {
      const updated = await run(() => updateTaskTemplate(id, changes));
      if (updated) {
        setTemplates((current) =>
          sortByName(
            current.map((template) => (template.id === id ? updated : template)),
          ),
        );
      }
      return updated;
    },
    [run, setTemplates],
  );

  const remove = React.useCallback(
    async (id: string) => {
      const done = await run(async () => {
        await deleteTaskTemplate(id);
        return true as const;
      });
      if (done) {
        setTemplates((current) => current.filter((template) => template.id !== id));
      }
      return done === true;
    },
    [run, setTemplates],
  );

  const clearError = React.useCallback(() => setWriteError(null), []);

  return {
    templates: query.data ?? [],
    status: query.isPending ? "loading" : query.isError ? "error" : "ok",
    error: writeError ?? query.error?.message ?? null,
    busy,
    create,
    update,
    remove,
    refresh,
    clearError,
  };
}

/** Mirrors the backend's `ORDER BY name, created_at` so a splice stays in place. */
function sortByName(templates: TaskTemplate[]): TaskTemplate[] {
  return [...templates].sort(
    (a, b) => a.name.localeCompare(b.name) || a.created_at.localeCompare(b.created_at),
  );
}
