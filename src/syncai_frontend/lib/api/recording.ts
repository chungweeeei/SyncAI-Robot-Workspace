// Client for the bag-recording surface: /api/v1/recordings.
// (backend: routers/recording.py, gateways/recording/recording.py)
//
// A recording is one `ros2 bag record` child on the robot writing into
// `record/<name>/`. Nothing in the stack reads those bags back — a bag is
// insurance, and what it insures against is a mapping run that ends without a
// save: pgo holds its keyframes in RAM, so replaying the lidar topics is the
// only way to rebuild a run that was lost.
//
// snake_case throughout, for the same reason lib/types/map.ts gives: these
// mirror the backend's field names so the fetchers are a pass-through rather
// than a rename table.

import { apiUrl } from "@/lib/api/config";
import { requestJson } from "@/lib/api/http";

/**
 * How a recording on disk stands — the backend's `RecordingStatus`.
 *
 * - `recording` — the live one. There is at most one per robot.
 * - `ok` — finished cleanly: rosbag2 wrote `metadata.yaml` and the bag plays.
 * - `interrupted` — the recorder went away without flushing, which is what a
 *   mode switch looks like (it tears down the byobu session the backend is a
 *   pane of) or a crash. The messages are still in the `.db3`; only the index
 *   is missing, and `ros2 bag reindex record/<name>` rebuilds it.
 *
 * Only two of the three are ever stored. `interrupted` is derived by the
 * backend from a directory with no metadata and no process behind it — the same
 * shape as a map's `grid_status`.
 */
export type RecordingStatus = "recording" | "ok" | "interrupted";

/** `ActiveRecordingResponse` — what is being written right now. */
export interface ActiveRecording {
  name: string;
  /** Absolute path of the bag directory on the robot. */
  path: string;
  /** Fully resolved topic names, i.e. already namespaced by the backend. */
  topics: string[];
  /** ISO 8601, UTC. */
  started_at: string;
  /**
   * Seconds since the recorder started, **as the robot measures it**. Rendered
   * straight rather than interpolated by a local timer: the poll below is what
   * ticks it, and a clock running ahead of the robot would be a number this
   * console made up.
   */
  elapsed_seconds: number;
  compression: boolean;
  /** Bytes written so far; the backend walks the bag directory per call. */
  size_bytes: number;
}

/** `StopRecordingResponse` — the finished recording, as the stop reported it. */
export interface StoppedRecording {
  name: string;
  path: string;
  topics: string[];
  started_at: string;
  elapsed_seconds: number;
  size_bytes: number;
  /**
   * Which rung of the stop ladder ended it: `sigint` is the normal one and the
   * only one that lets rosbag2 flush. `sigterm` / `sigkill` mean the recorder
   * was wedged, and `complete` is then false.
   */
  stopped_by: string;
  /** Whether `metadata.yaml` was written — i.e. whether the bag plays as-is. */
  complete: boolean;
}

/** `RecordingSummary` — one entry of the catalogue. */
export interface RecordingSummary {
  name: string;
  status: RecordingStatus;
  size_bytes: number;
  /** ISO 8601, most recently modified file in the bag directory. */
  modified_at: string;
  /** From the bag metadata; null until the recording finishes. */
  duration_seconds: number | null;
  /**
   * From the bag metadata; null until the recording finishes.
   *
   * **Zero on a finished bag is the symptom of a topic typo.** Nothing refuses
   * a topic that does not exist — the recorder subscribes when one appears, so
   * that it can be armed before bringup — which means a misspelled name records
   * silence rather than failing.
   */
  message_count: number | null;
  /** The topics actually recorded. Empty while live: rosbag2 writes the list on shutdown. */
  topics: string[];
  /** Compression format, or null for an uncompressed bag. */
  compression: string | null;
}

export interface StartRecordingRequest {
  /** Omitted for the backend's `rec_<UTC timestamp>` default. */
  name?: string;
  /**
   * Topics to record. A name without a leading slash is resolved under the
   * robot's namespace by the backend (`livox/lidar` → `/robot01/livox/lidar`),
   * which is why nothing here ever spells a robot id. An absolute name is
   * passed through, and that is how the fleet-wide `/tf` and `/tf_static` are
   * asked for.
   */
  topics?: string[];
  /** zstd on each closed 2 GB split. Off by default; see the control's copy. */
  compression?: boolean;
}

/**
 * Mirrors the backend catalogue's name rule so the Start button can refuse a
 * bad name before a request goes out. The server still validates — this is a
 * convenience, not the boundary. rosbag2 names its split files after the
 * directory, which is why the rule is this strict.
 */
export const RECORDING_NAME_RE = /^[A-Za-z0-9._-]{1,64}$/;

/**
 * The one name the backend refuses outright, because `GET
 * /api/v1/recordings/active` is its own route and a directory called that could
 * never be read back.
 */
export const RESERVED_RECORDING_NAME = "active";

/**
 * Start recording into `record/<name>/`.
 *
 * A 201 means the recorder survived its liveness probe, so it is genuinely
 * running — not that a bag will be any good. The refusals all land before
 * anything is spawned and all arrive as the backend's own sentence: one
 * recording at a time, a name already on disk, and less than 2 GB free where
 * bags are written (one split).
 */
export function startRecording(
  request: StartRecordingRequest,
): Promise<ActiveRecording> {
  return requestJson<ActiveRecording>(apiUrl("/api/v1/recordings"), {
    method: "POST",
    body: JSON.stringify(request),
  });
}

/** What is being recorded right now, or null. */
export function fetchActiveRecording(
  signal?: AbortSignal,
): Promise<ActiveRecording | null> {
  return requestJson<ActiveRecording | null>(
    apiUrl("/api/v1/recordings/active"),
    { signal },
  );
}

/**
 * Stop the live recorder and wait for the bag to be closed.
 *
 * Slow on purpose — up to ~22 s if the recorder has to be escalated past
 * SIGINT — because the one thing worth knowing is whether the bag plays, and
 * that is not knowable until the child is gone. Read `complete`, not the
 * status code.
 */
export function stopRecording(): Promise<StoppedRecording> {
  return requestJson<StoppedRecording>(apiUrl("/api/v1/recordings/stop"), {
    method: "POST",
  });
}

/** Every bag under `record/`, newest first. */
export function fetchRecordings(
  signal?: AbortSignal,
): Promise<RecordingSummary[]> {
  return requestJson<{ recordings: RecordingSummary[] }>(
    apiUrl("/api/v1/recordings"),
    { signal },
  ).then((body) => body.recordings);
}

/**
 * Delete `record/<name>/` and everything under it. There is no undo.
 *
 * 204, so nothing is parsed. Refused for the recording being written — an
 * rmtree under a live sqlite writer leaves the recorder writing into an
 * unlinked file.
 */
export function deleteRecording(name: string): Promise<void> {
  return requestJson<void>(
    apiUrl(`/api/v1/recordings/${encodeURIComponent(name)}`),
    { method: "DELETE", parse: false },
  );
}
