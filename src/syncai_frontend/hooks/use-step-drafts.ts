"use client";

import * as React from "react";

import {
  newStepDraft,
  type StepDraft,
} from "@/lib/task/step";
import type { StepType } from "@/lib/api/task";

export interface StepDrafts {
  steps: StepDraft[];
  add: (type: StepType) => void;
  remove: (key: number) => void;
  /** Swap with the neighbour. -1 is a no-op on the first row, +1 on the last. */
  move: (key: number, delta: -1 | 1) => void;
  patch: (key: number, changes: Partial<Omit<StepDraft, "key">>) => void;
  /**
   * Swap the whole list, for loading a template into the editor.
   *
   * The drafts must be freshly built (via `fromTemplateSteps`, which calls
   * `newStepDraft`) rather than reused, so every row arrives with a key that has
   * never been mounted — otherwise React would reconcile the incoming rows into
   * the outgoing ones and carry over the focused input.
   */
  replace: (drafts: StepDraft[]) => void;
  clear: () => void;
}

/**
 * The step list an operator is authoring. Pure state — no network, no error, no
 * busy flag.
 *
 * Kept apart from useTaskDispatch because the schedule path sends the identical
 * list and has nothing to do with task tracking: one combined hook would carry a
 * `taskStatus` into a schedule footer that has no status to show.
 *
 * Reordering is two buttons per row rather than drag-and-drop. Dragging needs
 * pointer capture, an insertion indicator, and a keyboard equivalent to stay
 * usable without a mouse — gridmap-editor-scale work for a list that is three to
 * six rows long, and the console has no drag-and-drop primitive to reuse.
 */
export function useStepDrafts(): StepDrafts {
  const [steps, setSteps] = React.useState<StepDraft[]>([]);

  const add = React.useCallback((type: StepType) => {
    setSteps((current) => [...current, newStepDraft(type)]);
  }, []);

  const remove = React.useCallback((key: number) => {
    setSteps((current) => current.filter((step) => step.key !== key));
  }, []);

  const move = React.useCallback((key: number, delta: -1 | 1) => {
    setSteps((current) => {
      const from = current.findIndex((step) => step.key === key);
      const to = from + delta;
      if (from < 0 || to < 0 || to >= current.length) return current;
      const next = [...current];
      // Swap rather than splice-and-insert: a neighbour exchange is what the two
      // buttons mean, and it keeps the other rows' identities untouched.
      [next[from], next[to]] = [next[to]!, next[from]!];
      return next;
    });
  }, []);

  /**
   * One addressed setter rather than a setter per field, because picking a vertex
   * writes four fields at once (x, y, theta and the provenance id) and that has
   * to be a single state update — two updates would render a frame whose numbers
   * are the new vertex's but whose provenance label is still the old one.
   */
  const patch = React.useCallback(
    (key: number, changes: Partial<Omit<StepDraft, "key">>) => {
      setSteps((current) =>
        current.map((step) => (step.key === key ? { ...step, ...changes } : step)),
      );
    },
    [],
  );

  const replace = React.useCallback((drafts: StepDraft[]) => setSteps(drafts), []);

  const clear = React.useCallback(() => setSteps([]), []);

  return { steps, add, remove, move, patch, replace, clear };
}
