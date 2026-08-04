// The step vocabulary and the draft model behind the /tasks composer.
//
// It sits here rather than in the components because two surfaces send the same
// list — an immediate dispatch and a schedule registration — and the rule for
// what a fillable step *is* must not have two copies. The conversion to wire
// shape lives here too, so lib/api/task.ts never has to know what a half-typed
// coordinate field looks like.

import type { SavedStep, SavedStepRequest } from "@/lib/api/saved-task";
import {
  normalizeTheta,
  type MoveStepParams,
  type StepType,
  type TaskStepRequest,
} from "@/lib/api/task";

export interface StepTypeSpec {
  value: StepType;
  /** Condensed caps in the UI, so short. */
  label: string;
  /** One character in the row's leading column, mirroring the vertex glyphs. */
  glyph: string;
  /** One line under the add row, so the vocabulary is not guessed. */
  hint: string;
}

/** Order is the add row's order; MOVE first because it is the only one with a body. */
export const STEP_TYPES: readonly StepTypeSpec[] = [
  { value: "MOVE", label: "Move", glyph: "M", hint: "Drive to a pose in the map frame." },
  { value: "STANDUP", label: "Stand", glyph: "S", hint: "Stand up. Nothing to set." },
  { value: "LIEDOWN", label: "Lie", glyph: "L", hint: "Lie down. Nothing to set." },
];

const GLYPHS: Record<StepType, string> = STEP_TYPES.reduce(
  (all, spec) => ({ ...all, [spec.value]: spec.glyph }),
  {} as Record<StepType, string>,
);

export function stepGlyph(type: StepType): string {
  return GLYPHS[type] ?? "?";
}

export interface StepDraft {
  /** Client-only identity: the React key, and what add / remove / move address. */
  key: number;
  type: StepType;
  /** MOVE only. Raw text — see the note below. */
  x: string;
  y: string;
  theta: string;
  /**
   * The vertex the numbers were prefilled from, or null once they were hand
   * edited.
   *
   * Two consumers, and the distinction matters. **Saving** sends it, so a stored
   * task records which vertex a MOVE came from and can follow that vertex when it
   * moves. **Dispatching** never sends it: `POST /api/v1/tasks` has no notion of
   * provenance, and a step's coordinates are resolved before they get there.
   */
  vertexId: string | null;
  /**
   * Set when this row was loaded from a saved task whose vertex has since been
   * deleted — the coordinates are the snapshot taken at save time, not a live
   * pose. Display only; it does not stop the row being dispatched.
   */
  vertexMissing?: boolean;
}

/**
 * A monotonic key per row.
 *
 * Not the backend's `step.id`, which is derived from *position* (see
 * `stepIdFor`) and therefore changes when a row is reordered or a row above it
 * is deleted. As a React key that would reconcile row 2's DOM into row 1
 * mid-reorder and the focused input would jump. Not `crypto.randomUUID` either:
 * a module counter is the shape `lib/map/session.ts` already established, and
 * strict-mode double mounting merely skips a number.
 */
let nextKey = 0;

export function newStepDraft(type: StepType): StepDraft {
  nextKey += 1;
  // theta defaults to "0" rather than "": a heading of zero is a real, common
  // answer, whereas a position has no sensible default and must be supplied.
  return { key: nextKey, type, x: "", y: "", theta: "0", vertexId: null };
}

/**
 * Render a stored number into a draft field.
 *
 * Vertices come off the wire as raw doubles — `6.8344510000000005`,
 * `90.17178865852111` — and putting that verbatim into a 7-character-wide input
 * gives the operator a field they cannot read or edit. Rounding here does change
 * what gets dispatched, and that is the honest trade rather than a hidden one:
 * the draft is text, and what is in the box is exactly what is sent, so a field
 * showing a value it will not send would be the worse lie.
 *
 * The precisions are chosen to be irrelevant to the robot. 1 mm of position is
 * two orders of magnitude below the costmap resolution the planner works at, and
 * 0.1° of heading is far below what the gait controller tracks. `Number(...)`
 * around the `toFixed` is what drops the padding, so 0 reads "0" and not "0.000".
 */
function formatDraftNumber(value: number, decimals: number): string {
  return String(Number(value.toFixed(decimals)));
}

/** Metres, to the millimetre. */
export function formatDraftPosition(value: number): string {
  return formatDraftNumber(value, 3);
}

/** Degrees, to a tenth. */
export function formatDraftAngle(value: number): string {
  return formatDraftNumber(value, 1);
}

/**
 * The step id the backend sees: `1-move`, `2-standup`, `3-move`.
 *
 * Position plus type, because that is unique within the task by construction,
 * reads as a route in the Temporal UI and in `TaskStateResponse.steps[].id`, and
 * keeps the spirit of the single-gesture convention where the step id is the kind
 * (`goal`, `standup`). The rejected alternative — the client `key` — would call
 * the third step `7-move` because the counter had been bumped by deleted rows.
 *
 * The cost is that the ids only line up with the on-screen rows while the list
 * has not been edited, which is why the composer freezes the list for the
 * duration of a dispatched task.
 */
export function stepIdFor(index: number, type: StepType): string {
  return `${index + 1}-${type.toLowerCase()}`;
}

/**
 * Why x / y / theta are stored as strings.
 *
 * The operator hand-edits them after a prefill, and "", "-", "1." and "1e" are
 * all states a text input legitimately passes through mid-typing. `Number("")`
 * is 0 and `Number("-")` is NaN, so storing numbers means a half-typed minus
 * sign silently becomes a MOVE to the origin, or a NaN that JSON.stringify
 * writes as `null` and the backend rejects as a 422 whose detail is a validation
 * *array* rather than a sentence — the exact thing the vertex panel refuses to
 * put on an operator's screen. Text in, parsed once, here.
 */
function parseCoordinate(raw: string): number | null {
  const trimmed = raw.trim();
  if (!trimmed) return null;
  const value = Number(trimmed);
  return Number.isFinite(value) ? value : null;
}

function moveParams(draft: StepDraft): MoveStepParams | null {
  const x = parseCoordinate(draft.x);
  const y = parseCoordinate(draft.y);
  const theta = parseCoordinate(draft.theta);
  if (x === null || y === null || theta === null) return null;
  // Folded on the way out, not on change: rewriting the field under the cursor
  // would make typing "180" one character at a time impossible.
  return { x, y, theta: normalizeTheta(theta) };
}

/** The operator-facing reason this row cannot be sent, or null. */
export function stepDraftError(draft: StepDraft): string | null {
  if (draft.type !== "MOVE") return null;
  // No theta *range* check: out-of-range is what normalizeTheta is for, and
  // refusing 270° when the answer is -90° would be inventing a constraint.
  return moveParams(draft) ? null : "Needs a numeric X, Y and heading.";
}

/**
 * True when the whole list can be dispatched.
 *
 * The empty list is refused here rather than left to the backend, which accepts
 * `steps: []` and starts a workflow that completes immediately — a dispatch that
 * reports COMPLETED and moved nothing, which is worse than an error.
 */
export function stepDraftsSubmittable(drafts: readonly StepDraft[]): boolean {
  return drafts.length > 0 && drafts.every((draft) => !stepDraftError(draft));
}

export function toStepRequest(draft: StepDraft, index: number): TaskStepRequest {
  const id = stepIdFor(index, draft.type);
  if (draft.type !== "MOVE") return { id, type: draft.type };

  const params = moveParams(draft);
  // Callers gate on stepDraftsSubmittable, so this is a programming error rather
  // than an operator one — throwing beats sending a MOVE to (0, 0).
  if (!params) throw new Error(`Step ${id} has no usable coordinates.`);
  return { id, type: "MOVE", params };
}

export function toStepRequests(
  drafts: readonly StepDraft[],
): TaskStepRequest[] {
  return drafts.map(toStepRequest);
}

/**
 * The authored list in the shape the saved-task endpoint stores.
 *
 * Identical to `toStepRequests` except that a MOVE also carries its `vertex_id`.
 * That one extra field is the whole difference between "run this now" and "keep
 * this so it can follow the map later", which is why the two conversions sit next
 * to each other rather than one calling into the other with a flag.
 */
export function toSavedSteps(
  drafts: readonly StepDraft[],
): SavedStepRequest[] {
  return drafts.map((draft, index) => {
    const request = toStepRequest(draft, index);
    if (request.type !== "MOVE") return request;
    // Only ever attached to a MOVE: the backend 422s a posture step that carries
    // one, and the union above is what makes that unrepresentable here.
    return { ...request, vertex_id: draft.vertexId };
  });
}

/**
 * Load a saved task's steps back into the editor.
 *
 * The numbers come from `resolved_params`, **not** `params`: the server has
 * already applied "the vertex's current pose wins, the snapshot is the fallback",
 * so this is where a dock moved on the map shows up in the composer. Using
 * `params` would put the stale snapshot on screen and then dispatch something
 * else, which is the one genuinely confusing outcome available here.
 *
 * The formatters are the same ones the vertex picker prefills through, so a
 * round trip through save/load does not change what is in the fields.
 */
export function fromSavedSteps(steps: readonly SavedStep[]): StepDraft[] {
  return steps.map((step) => {
    const draft = newStepDraft(step.type);
    const pose = step.resolved_params;
    return {
      ...draft,
      x: pose ? formatDraftPosition(pose.x) : draft.x,
      y: pose ? formatDraftPosition(pose.y) : draft.y,
      theta: pose ? formatDraftAngle(pose.theta) : draft.theta,
      vertexId: step.vertex_id,
      vertexMissing: step.vertex_status === "MISSING",
    };
  });
}
