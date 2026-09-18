import * as React from "react";
import { useQuery, useQueryClient } from "@tanstack/react-query";

import {
  fetchActiveRecording,
  fetchRecordings,
  type ActiveRecording,
  type RecordingSummary,
} from "@/lib/api/recording";
import { queryKeys } from "@/lib/api/query-keys";

/**
 * How often the live recorder is re-read while one is running.
 *
 * One second, because this poll *is* the elapsed clock — the readout renders
 * the robot's own `elapsed_seconds` rather than a local timer counting up from
 * `started_at`, so the number never claims a precision or a liveness the
 * connection does not have. A recorder that died takes at most a second to stop
 * reading as live, which is the same thing the operator would want anyway.
 */
const ACTIVE_POLL_MS = 1000;

/**
 * And how often while nothing is recording.
 *
 * Still polled, rather than fetched once: a recording can be started from a
 * shell on the robot or from a second console, and a page that only asked on
 * mount would offer Start against a robot that is already busy and get a 409
 * for it. Five seconds is a background check, not a clock.
 */
const IDLE_POLL_MS = 5000;

/**
 * And how often the catalogue is re-read while a recording is live.
 *
 * Only then: the set of directories under `record/` changes when somebody
 * starts, stops or deletes one, and all three are deliberate acts this console
 * already knows about and invalidates for. The exception is the live entry,
 * whose size grows on its own — so, like the maps catalogue's conversion poll,
 * this turns itself off with the recording that justified it.
 */
const RECORDING_POLL_MS = 2000;

function pollWhileRecording(
  recordings: RecordingSummary[] | undefined,
): number | false {
  return recordings?.some((entry) => entry.status === "recording")
    ? RECORDING_POLL_MS
    : false;
}

export type RecordingsStatus = "loading" | "ok" | "error";

export interface UseActiveRecording {
  /** The live recorder, or null for "nothing is recording" — a real answer. */
  active: ActiveRecording | null;
  /** "loading" until the first response; a later failure keeps the last answer. */
  status: RecordingsStatus;
}

/**
 * The live recorder, polled.
 *
 * Its own query entry rather than a derived view of the catalogue, and the
 * distinction is not cosmetic: this is the cheap question (one slot in the
 * backend's memory) and the catalogue is the expensive one (a directory walk
 * per bag). Ticking the elapsed readout off the catalogue would mean walking
 * every bag on the robot once a second to move one number.
 *
 * `null` from the server and `null` before the first response are told apart by
 * `status`, which is what stops the panel from flashing its idle form on mount.
 */
export function useActiveRecording(): UseActiveRecording {
  const { data, isPending, isError } = useQuery({
    queryKey: queryKeys.activeRecording,
    queryFn: ({ signal }) => fetchActiveRecording(signal),
    refetchInterval: (query) =>
      query.state.data ? ACTIVE_POLL_MS : IDLE_POLL_MS,
  });

  return {
    active: data ?? null,
    status: isPending ? "loading" : isError ? "error" : "ok",
  };
}

export interface UseRecordings {
  /** Latest successfully fetched catalogue, or null before the first success. */
  recordings: RecordingSummary[] | null;
  status: RecordingsStatus;
  /** Re-read the catalogue. For a caller holding the hook's result. */
  refresh: () => void;
}

/**
 * The bag catalogue, through the shared query cache.
 *
 * Same policy as useMaps: refetch on mount, no timer of its own, and a poll
 * that only runs while the server is changing something behind the console's
 * back — here, a live recording's size. Everything else that moves the list
 * (start, stop, delete) goes through this console and invalidates the key.
 */
export function useRecordings(): UseRecordings {
  const queryClient = useQueryClient();
  const { data, isPending, isError } = useQuery({
    queryKey: queryKeys.recordings,
    queryFn: ({ signal }) => fetchRecordings(signal),
    refetchInterval: (query) => pollWhileRecording(query.state.data),
  });

  // Invalidate rather than refetch(): the entry is shared, so a refresh must
  // update every mounted observer and not just this one.
  const refresh = React.useCallback(
    () => void queryClient.invalidateQueries({ queryKey: queryKeys.recordings }),
    [queryClient],
  );

  return {
    recordings: data ?? null,
    status: isPending ? "loading" : isError ? "error" : "ok",
    refresh,
  };
}
