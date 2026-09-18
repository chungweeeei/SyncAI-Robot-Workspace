"use client";

import * as React from "react";
import { PlusIcon, XIcon } from "lucide-react";

import { Input } from "@/components/ui/input";
import { cn } from "@/lib/utils";

/**
 * The topics offered as one tap, and why each is on the list.
 *
 * Hardcoded rather than discovered: the backend has no topic-list route, and
 * adding one to serve a convenience row would put a ROS graph query behind
 * every visit to this page. The cost is that this list can drift from what the
 * robot actually publishes — which is what the free-text row below is for, and
 * why a recording's message count is on every row of the catalogue.
 *
 * Relative names, with two deliberate exceptions. Everything the robot owns is
 * namespaced and the backend expands it (`livox/lidar` →
 * `/robot01/livox/lidar`), so nothing here spells a robot id; `/tf` and
 * `/tf_static` are genuinely fleet-wide and are written absolute, which is the
 * escape hatch the API documents.
 */
const PRESETS: readonly { topic: string; hint: string }[] = [
  // The pair a lost mapping run is replayed from — the reason this page exists.
  { topic: "livox/lidar", hint: "Lidar points, straight off the MID360" },
  { topic: "livox/imu", hint: "Lidar IMU — LIO needs it alongside the points" },
  { topic: "odom", hint: "LIO odometry, projected to 2D" },
  { topic: "robot_state", hint: "The aggregate telemetry, 1 Hz" },
  { topic: "cmd_vel", hint: "Velocity commands sent to the gait controller" },
  { topic: "pointlio/body_cloud", hint: "The deskewed cloud the console draws" },
  { topic: "/tf", hint: "Transforms — fleet-wide, not namespaced" },
  { topic: "/tf_static", hint: "Static transforms — fleet-wide, not namespaced" },
];

/** The default selection: the LIO inputs, matching the backend's own default. */
export const DEFAULT_TOPICS = ["livox/lidar", "livox/imu"];

const PRESET_TOPICS = new Set(PRESETS.map((preset) => preset.topic));

/**
 * A topic name the recorder can be given without the request being obviously
 * wasted. Deliberately permissive — ROS name rules are stricter than this and
 * the backend does not check them either, because a topic that does not exist
 * *yet* is a legitimate thing to arm a recorder against.
 */
function usable(topic: string): boolean {
  return topic.length > 0 && !/\s/.test(topic);
}

/**
 * One selectable topic. The `Segmented` idiom widened to multi-select: chosen
 * segments take the commanded hue because what is recorded is the operator's
 * choice, not a measurement.
 *
 * The label is the topic name itself, in the readout face. A friendlier word
 * ("Lidar") would read better and be the wrong trade: these exact strings are
 * what ends up in the bag's metadata and what `ros2 bag play` will name, so the
 * chip has to be verifiable against them. The plain-language half is the
 * tooltip.
 */
function TopicChip({
  topic,
  hint,
  selected,
  disabled,
  onToggle,
  onRemove,
}: {
  topic: string;
  hint?: string;
  selected: boolean;
  disabled: boolean;
  onToggle: () => void;
  /** Present only for a hand-added topic — a preset is never removed, only unpicked. */
  onRemove?: () => void;
}) {
  return (
    <span
      className={cn(
        "inline-flex items-center rounded-sm border transition-colors",
        selected
          ? "border-signal-cmd/50 bg-signal-cmd/12 text-signal-cmd"
          : "border-hairline text-muted-foreground",
        disabled && "opacity-40",
      )}
    >
      <button
        type="button"
        aria-pressed={selected}
        disabled={disabled}
        onClick={onToggle}
        title={hint ?? topic}
        className={cn(
          "readout h-6 px-1.5 text-[11px] leading-none",
          !disabled && !selected && "hover:bg-elevated hover:text-foreground",
        )}
      >
        {topic}
      </button>
      {onRemove && (
        <button
          type="button"
          disabled={disabled}
          onClick={onRemove}
          aria-label={`Forget ${topic}`}
          title={`Forget ${topic}`}
          className="flex size-5 items-center justify-center rounded-sm opacity-60 transition-opacity hover:opacity-100"
        >
          <XIcon className="size-3" aria-hidden />
        </button>
      )}
    </span>
  );
}

/**
 * Pick the topics a recording will subscribe to.
 *
 * Presets as toggles plus a free-text row, rather than the bare text field the
 * hand-pasted `ros2 bag record` command was: the common case here is one tap
 * (the LIO pair, already selected), and the failure mode of typing is silent —
 * a misspelled topic records nothing and says nothing, because the recorder
 * waits for topics rather than refusing unknown ones.
 *
 * Hand-added topics join the same row and keep an X, so a typo can be taken
 * back rather than sitting in the list unpicked. A preset has no X: its place
 * in the row is fixed and unpicking it is what "not this one" means.
 */
export function TopicPicker({
  value,
  onChange,
  disabled = false,
}: {
  value: string[];
  onChange: (topics: string[]) => void;
  disabled?: boolean;
}) {
  const [draft, setDraft] = React.useState("");
  // Hand-added names, kept so an unpicked one stays on screen to be re-picked.
  // Selection alone could not hold them: unpicking would make the chip vanish.
  const [extra, setExtra] = React.useState<string[]>([]);

  const selected = React.useMemo(() => new Set(value), [value]);

  const toggle = React.useCallback(
    (topic: string) => {
      onChange(
        selected.has(topic)
          ? value.filter((entry) => entry !== topic)
          : [...value, topic],
      );
    },
    [onChange, selected, value],
  );

  const add = React.useCallback(() => {
    const topic = draft.trim();
    if (!usable(topic)) return;
    setDraft("");
    // An existing name is selected rather than duplicated — typing a preset's
    // name is a reasonable way to reach for it.
    if (!selected.has(topic)) onChange([...value, topic]);
    if (!PRESET_TOPICS.has(topic) && !extra.includes(topic)) {
      setExtra((current) => [...current, topic]);
    }
  }, [draft, extra, onChange, selected, value]);

  const forget = React.useCallback(
    (topic: string) => {
      setExtra((current) => current.filter((entry) => entry !== topic));
      onChange(value.filter((entry) => entry !== topic));
    },
    [onChange, value],
  );

  return (
    <div className="space-y-2">
      <div className="flex flex-wrap gap-1.5">
        {PRESETS.map((preset) => (
          <TopicChip
            key={preset.topic}
            topic={preset.topic}
            hint={preset.hint}
            selected={selected.has(preset.topic)}
            disabled={disabled}
            onToggle={() => toggle(preset.topic)}
          />
        ))}
        {extra.map((topic) => (
          <TopicChip
            key={topic}
            topic={topic}
            selected={selected.has(topic)}
            disabled={disabled}
            onToggle={() => toggle(topic)}
            onRemove={() => forget(topic)}
          />
        ))}
      </div>

      <div className="flex items-center gap-2">
        <Input
          value={draft}
          onChange={(event) => setDraft(event.target.value)}
          onKeyDown={(event) => {
            // Enter adds rather than submits: this control sits inside the
            // recorder's form, where the default would start the recording with
            // the topic still sitting unread in the box.
            if (event.key === "Enter") {
              event.preventDefault();
              add();
            }
          }}
          placeholder="another topic"
          aria-label="Add a topic"
          disabled={disabled}
          className="readout h-7 flex-1 text-[11px]"
        />
        <button
          type="button"
          onClick={add}
          disabled={disabled || !usable(draft.trim())}
          className={cn(
            "instrument-label flex h-7 shrink-0 items-center gap-1 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors",
            "hover:bg-elevated hover:text-foreground",
            "disabled:pointer-events-none disabled:opacity-40",
          )}
        >
          <PlusIcon className="size-3" aria-hidden />
          Add
        </button>
      </div>

      <p className="text-[11px] leading-tight text-muted-foreground">
        Names are resolved under this robot&apos;s namespace. Write a leading
        slash for a fleet-wide topic.
      </p>
    </div>
  );
}
