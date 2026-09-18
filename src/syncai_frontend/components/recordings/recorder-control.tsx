"use client";

import * as React from "react";
import { CircleIcon, SquareIcon } from "lucide-react";
import { useQueryClient } from "@tanstack/react-query";

import { Readout } from "@/components/console/instrument";
import { TopicPicker, DEFAULT_TOPICS } from "@/components/recordings/topic-picker";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { useActiveRecording } from "@/hooks/use-recordings";
import { queryKeys } from "@/lib/api/query-keys";
import {
  RECORDING_NAME_RE,
  RESERVED_RECORDING_NAME,
  startRecording,
  stopRecording,
  type ActiveRecording,
  type StoppedRecording,
} from "@/lib/api/recording";
import { formatDuration, formatSize, formatTimestamp } from "@/lib/recording/format";
import { cn } from "@/lib/utils";

/**
 * The record indicator: a filled dot that breathes while the recorder runs.
 *
 * The one animated thing on this screen, and it earns the exception — a bag
 * grows on disk with no other outward sign, and the single question an operator
 * opens this page to answer is whether the robot is still recording. A static
 * dot answers it too, but not from across a room. Reduced motion drops the
 * animation and keeps the dot, which is the whole signal; the pulse is
 * emphasis, never the message.
 */
function RecordDot() {
  return (
    <span className="relative flex size-2.5 shrink-0 items-center justify-center">
      <span
        aria-hidden
        className="absolute inline-flex size-full animate-ping rounded-full bg-signal-warn opacity-60 motion-reduce:hidden"
      />
      <span className="relative inline-flex size-2.5 rounded-full bg-signal-warn" />
    </span>
  );
}

/** The live recorder: what is being written, and the one way to end it. */
function LiveRecorder({
  active,
  busy,
  onStop,
}: {
  active: ActiveRecording;
  busy: boolean;
  onStop: () => void;
}) {
  return (
    <div className="space-y-3">
      <div className="flex items-start justify-between gap-4">
        <div>
          <span className="instrument-label flex items-center gap-2 text-signal-warn">
            <RecordDot />
            Recording
          </span>
          {/* The elapsed clock is the robot's own number, arriving once a
            * second. A local timer counting up from started_at would keep
            * ticking through a dropped connection and a dead recorder — the two
            * states this readout exists to expose. */}
          <p className="readout mt-2 text-3xl leading-none font-medium text-signal-warn">
            {formatDuration(active.elapsed_seconds)}
          </p>
        </div>

        {/* Not `destructive`: in this console red means faulted, and stopping
          * is the act that makes the bag playable. It is the commanded hue like
          * every other thing the operator does deliberately. */}
        <Button type="button" size="sm" disabled={busy} onClick={onStop}>
          <SquareIcon data-icon="inline-start" />
          {busy ? "Stopping…" : "Stop"}
        </Button>
      </div>

      <div className="space-y-1.5">
        <Readout label="Written" value={formatSize(active.size_bytes)} tone="live" />
        <Readout
          label="Topics"
          value={active.topics.length}
          tone="cmd"
          className="cursor-default"
        />
        <Readout label="Started" value={formatTimestamp(active.started_at)} />
        <Readout label="Bag" value={active.name} />
      </div>

      {/* The resolved names, which is what actually went to the recorder — the
        * picker showed the relative spelling and the backend expanded it. Worth
        * the four lines: a topic namespaced onto the wrong robot is invisible
        * anywhere else until the bag comes back empty. */}
      <ul className="space-y-0.5">
        {active.topics.map((topic) => (
          <li
            key={topic}
            className="readout text-[11px] leading-tight break-all text-muted-foreground"
          >
            {topic}
          </li>
        ))}
      </ul>

      <p className="text-[11px] leading-tight text-muted-foreground">
        Stopping flushes the bag&apos;s index, which is what makes it playable.
        Leaving this page does not stop the recorder.
      </p>
    </div>
  );
}

/** How the last recording ended. Held until the next one starts. */
function StoppedLine({ stopped }: { stopped: StoppedRecording }) {
  if (!stopped.complete) {
    return (
      <p className="text-[11px] leading-snug text-signal-caution">
        {stopped.name} was stopped with {stopped.stopped_by} and has no index —
        the messages are there, but it needs{" "}
        <span className="readout">ros2 bag reindex record/{stopped.name}</span>{" "}
        on the robot before it will play.
      </p>
    );
  }
  return (
    <p className="text-[11px] leading-snug text-signal-live">
      Saved {stopped.name} — {formatDuration(stopped.elapsed_seconds)},{" "}
      {formatSize(stopped.size_bytes)}.
    </p>
  );
}

/**
 * Start and stop the robot's bag recorder.
 *
 * The page's one live instrument, and the only control on it: the catalogue
 * below is a record of what this panel has done. It has two faces rather than
 * one with disabled parts, because the two states share no fields — idle asks
 * what to record, live reports what is being recorded — and a start form greyed
 * out under a running recorder would invite filling in a name that cannot be
 * used.
 *
 * Which face is shown comes from the server, not from a local "we pressed
 * Start" flag. That is what makes the panel correct about a recording started
 * from a shell on the robot or from a second console, and about one that died
 * on its own — the backend reaps a dead recorder on the same read.
 *
 * Refusals are the backend's sentences verbatim, as everywhere else in this
 * console: one recording at a time, a name already on disk, and less than 2 GB
 * free. The last one is the only one that asks the operator to go and do
 * something, and what it asks for — delete a recording — is the list below.
 */
export function RecorderControl() {
  const queryClient = useQueryClient();
  const { active, status } = useActiveRecording();

  const [name, setName] = React.useState("");
  const [topics, setTopics] = React.useState<string[]>(DEFAULT_TOPICS);
  const [compression, setCompression] = React.useState(false);
  const [busy, setBusy] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);
  const [stopped, setStopped] = React.useState<StoppedRecording | null>(null);

  // Empty is valid — the backend names the bag `rec_<UTC timestamp>` and that
  // is the right default for the run somebody is about to drive.
  const nameValid =
    name.length === 0 ||
    (RECORDING_NAME_RE.test(name) && name !== RESERVED_RECORDING_NAME);

  const start = React.useCallback(async () => {
    setBusy(true);
    setError(null);
    setStopped(null);
    try {
      const started = await startRecording({
        name: name.trim() || undefined,
        topics,
        compression,
      });
      setName("");
      // Both entries: the panel switches face off the first, the list gains a
      // row from the second. Setting the active entry directly rather than
      // waiting for the next poll is what makes the switch feel like the press.
      queryClient.setQueryData(queryKeys.activeRecording, started);
      void queryClient.invalidateQueries({ queryKey: queryKeys.recordings });
    } catch (cause) {
      setError(cause instanceof Error ? cause.message : String(cause));
    } finally {
      setBusy(false);
    }
  }, [compression, name, queryClient, topics]);

  const stop = React.useCallback(async () => {
    setBusy(true);
    setError(null);
    try {
      const result = await stopRecording();
      setStopped(result);
      queryClient.setQueryData(queryKeys.activeRecording, null);
      void queryClient.invalidateQueries({ queryKey: queryKeys.recordings });
    } catch (cause) {
      // A 409 here means the recorder was already gone — reaped after a crash,
      // or stopped from somewhere else. The catalogue is what says which.
      setError(cause instanceof Error ? cause.message : String(cause));
      void queryClient.invalidateQueries({ queryKey: queryKeys.activeRecording });
    } finally {
      setBusy(false);
    }
  }, [queryClient]);

  return (
    <section className="rounded-md border border-hairline bg-panel px-4 py-3.5">
      <h2 className="instrument-label mb-3 text-muted-foreground">Recorder</h2>

      {/* Nothing committal until the first answer: showing the start form while
        * the robot might already be recording would offer a button that can
        * only 409. */}
      {status === "loading" ? (
        <p className="text-[11px] leading-tight text-muted-foreground">
          Reading the recorder…
        </p>
      ) : active ? (
        <LiveRecorder active={active} busy={busy} onStop={() => void stop()} />
      ) : (
        <form
          onSubmit={(event) => {
            event.preventDefault();
            if (!busy && nameValid && topics.length > 0) void start();
          }}
          className="space-y-3"
        >
          <div className="flex items-center gap-2">
            <Input
              value={name}
              onChange={(event) => setName(event.target.value)}
              placeholder="bag name (optional)"
              aria-label="Bag name"
              disabled={busy}
              className="h-8 flex-1 text-sm"
            />
            <Button type="submit" size="sm" disabled={busy || !nameValid || topics.length === 0}>
              <CircleIcon data-icon="inline-start" />
              {busy ? "Starting…" : "Start recording"}
            </Button>
          </div>

          {/* Shown only while the name breaks the rule — a resting hint would
            * be chrome on a field that is usually left empty. */}
          {!nameValid && (
            <p className="text-[11px] leading-snug text-signal-caution">
              {name === RESERVED_RECORDING_NAME
                ? "“active” is reserved — it is the name of this page's own status route."
                : "Letters, digits, dot, dash and underscore only, up to 64 characters."}
            </p>
          )}

          <TopicPicker value={topics} onChange={setTopics} disabled={busy} />

          {topics.length === 0 && (
            <p className="text-[11px] leading-snug text-signal-caution">
              Pick at least one topic. A bag of nothing is still a directory.
            </p>
          )}

          <div className="flex items-start justify-between gap-4 border-t border-hairline pt-3">
            <div>
              <Label htmlFor="bag-compression">Compress with zstd</Label>
              <p className="mt-0.5 text-[11px] leading-tight text-muted-foreground">
                Roughly halves a lidar bag, in a burst on the recorder&apos;s own
                thread each time a 2 GB split closes. Leave it off during
                mapping — the Tegra is already busy with LIO.
              </p>
            </div>
            <Switch
              id="bag-compression"
              checked={compression}
              onCheckedChange={setCompression}
              disabled={busy}
            />
          </div>
        </form>
      )}

      {stopped && (
        <div className={cn(active ? "mt-3" : "mt-3 border-t border-hairline pt-3")}>
          <StoppedLine stopped={stopped} />
        </div>
      )}

      {error && (
        <p
          role="alert"
          className="mt-3 text-[11px] leading-snug break-words text-signal-warn"
        >
          {error}
        </p>
      )}
    </section>
  );
}
