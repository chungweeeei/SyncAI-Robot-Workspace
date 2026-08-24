// Client for the task-template API
// (src/syncai_backend/syncai_backend/interfaces/rest/routers/task_template.py).
//
// Separate from lib/api/task.ts for the third time the same argument applies.
// That file's thesis is that creation *is* dispatch, there is no GET collection,
// and an id is a per-page-load throwaway. A template is the exact inverse: a
// durable row with a uuid identity, full CRUD, and no dispatch endpoint of its
// own. Merging them would leave one file whose header has to explain both.
//
// The endpoint is `/api/v1/task_templates`, deliberately not under
// `/api/v1/tasks/...`: GET /api/v1/tasks/{id} takes an unconstrained string (a
// Temporal workflow id), so any static sub-path is either shadowed by it or
// steals the workflow id "templates".
//
// **Vertex resolution is the server's job, not this client's.** A stored MOVE
// step keeps both a vertex reference and a coordinate snapshot, and every read
// reports `resolved_params` — what a dispatch should send *now*. Dispatching is
// therefore a projection of three keys (see `toDispatchSteps`) rather than a
// reimplementation of "prefer the vertex, fall back to the snapshot". That rule
// lives once, on the server, where a template for a *non-active* map can still
// be resolved — something this client cannot do, since it only ever holds the
// active map's vertices.

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";
import type { ScheduleTrigger } from "@/lib/api/schedule";
import type { MoveStepParams, StepType, TaskStepRequest } from "@/lib/api/task";

/** Mirrors the backend's `max_length=255` and the `String(255)` column. */
export const TASK_TEMPLATE_NAME_MAX = 255;

/** One step as it is *stored*: a task step plus where a MOVE's numbers came from. */
export type TemplateStepRequest = TaskStepRequest & {
  /** MOVE only — the backend 422s a posture step carrying one. */
  vertex_id?: string | null;
};

/**
 * Whether a stored MOVE step's vertex reference still resolves.
 *
 * `NONE` — the coordinates were typed by hand, so there is nothing to resolve.
 * `CURRENT` — the vertex exists; `resolved_params` and `vertex_name` are its live
 * values, so this step follows the map.
 * `MISSING` — the vertex was deleted; `resolved_params` falls back to the
 * snapshot and `vertex_name` is the label it had when saved. Dispatch still
 * works; the operator is told.
 */
export type VertexRefStatus = "NONE" | "CURRENT" | "MISSING";

/** `TemplateStepResponse`, verbatim. */
export interface TemplateStep {
  id: string;
  type: StepType;
  /** The snapshot, exactly as it was saved. Null for a posture step. */
  params: MoveStepParams | null;
  vertex_id: string | null;
  /** Live name when CURRENT, the snapshot label when MISSING, else null. */
  vertex_name: string | null;
  vertex_status: VertexRefStatus;
  /** What to dispatch right now. Null for a posture step. */
  resolved_params: MoveStepParams | null;
}

/** `TaskTemplateResponse`, verbatim. */
export interface TaskTemplate {
  /**
   * This row's uuid. **Not** a task id — it cannot be passed to
   * `fetchTaskState`, which wants the Temporal workflow id a dispatch returns.
   */
  id: string;
  name: string;
  description: string;
  /** Null when the template has no MOVE step and so runs anywhere. */
  map_name: string | null;
  steps: TemplateStep[];
  /** False only when this template names a map that is not the one the robot is on. */
  map_matches_active: boolean;
  missing_vertex_count: number;
  created_at: string;
  updated_at: string;
}

/** What the operator supplies. `id` and the resolution come from the server. */
export interface TaskTemplateDraft {
  name: string;
  description?: string;
  /** Required when any step is a MOVE; must be omitted otherwise. */
  map_name?: string | null;
  steps: readonly TemplateStepRequest[];
}

/** Fields to change. Omitted ones are left alone by the backend's `exclude_unset`. */
export type TaskTemplateChanges = Partial<TaskTemplateDraft>;

/**
 * Project a template's steps into the dispatch wire shape.
 *
 * `resolved_params` is what the server says to send, so this drops the
 * provenance and nothing else. A posture step gets no `params` key at all —
 * which is why the result is typed as the discriminated `TaskStepRequest` rather
 * than something with an optional `params`.
 */
export function toDispatchSteps(steps: readonly TemplateStep[]): TaskStepRequest[] {
  return steps.map((step) =>
    step.type === "MOVE"
      ? {
          id: step.id,
          type: "MOVE" as const,
          // Non-null by construction: the backend always resolves a MOVE to
          // either the vertex's pose or the snapshot. Falling back to an origin
          // would be a silent drive to (0, 0), so this throws instead.
          params: assertParams(step),
        }
      : { id: step.id, type: step.type },
  );
}

function assertParams(step: TemplateStep): MoveStepParams {
  if (!step.resolved_params) {
    throw new Error(`Template step "${step.id}" has no coordinates to dispatch.`);
  }
  return step.resolved_params;
}

function taskTemplatePath(id?: string): string {
  const base = "/api/v1/task_templates";
  return apiUrl(id ? `${base}/${encodeURIComponent(id)}` : base);
}

export function listTaskTemplates(signal?: AbortSignal): Promise<TaskTemplate[]> {
  // Deliberately unfiltered, though the endpoint accepts `?map_name=`. The
  // console scopes the library to the active map itself so it can also report how
  // many rows it hid — silently dropping another map's templates is how an
  // operator concludes their work was lost, which is the complaint this feature
  // answers.
  return requestJson<TaskTemplate[]>(taskTemplatePath(), { signal });
}

export function createTaskTemplate(draft: TaskTemplateDraft): Promise<TaskTemplate> {
  return requestJson<TaskTemplate>(taskTemplatePath(), {
    method: "POST",
    body: JSON.stringify(draft),
  });
}

export function updateTaskTemplate(
  id: string,
  changes: TaskTemplateChanges,
): Promise<TaskTemplate> {
  return requestJson<TaskTemplate>(taskTemplatePath(id), {
    method: "PUT",
    body: JSON.stringify(changes),
  });
}

/** Delete a template. The `{message}` envelope is dropped, as everywhere else. */
export function deleteTaskTemplate(id: string): Promise<void> {
  return requestJson<void>(taskTemplatePath(id), {
    method: "DELETE",
    parse: false,
  });
}

/**
 * Freeze a template's current resolution into a Temporal schedule.
 *
 * Stricter than an immediate dispatch, by design on the backend: it refuses a
 * template whose map is not the active one, and refuses one with a MISSING
 * vertex. A scheduled run is unattended, so it does not get the snapshot
 * fallback an operator watching the screen is allowed. Both come back as a 400
 * sentence.
 */
export function scheduleTaskTemplate(
  id: string,
  scheduleId: string,
  trigger: ScheduleTrigger,
): Promise<void> {
  return requestJson<void>(apiUrl(`/api/v1/task_templates/${encodeURIComponent(id)}/schedule`), {
    method: "POST",
    body: JSON.stringify({ id: scheduleId, trigger }),
    parse: false,
  });
}
