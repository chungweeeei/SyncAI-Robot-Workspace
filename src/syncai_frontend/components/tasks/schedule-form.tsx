"use client";

import * as React from "react";
import { CalendarPlusIcon } from "lucide-react";

import { Segmented } from "@/components/console/instrument";
import { Input } from "@/components/ui/input";
import type { ScheduleTrigger } from "@/lib/api/schedule";

type TriggerKind = "interval" | "cron";

const TRIGGER_OPTIONS = [
  { value: "interval", label: "Every" },
  { value: "cron", label: "Cron" },
] as const satisfies readonly { value: TriggerKind; label: string }[];

export interface ScheduleFormProps {
  /** Already-registered ids, so a duplicate is refused before the 400. */
  existingIds: readonly string[];
  /** False when the step list itself is not sendable. */
  ready: boolean;
  /** Why not, as a muted line under the button. Null when ready. */
  reason: string | null;
  busy: boolean;
  error: string | null;
  onCreate: (id: string, trigger: ScheduleTrigger) => void;
}

/**
 * Register the authored step list to run on a timer.
 *
 * The trigger is a Segmented rather than two always-visible fields, which is what
 * makes the backend's cron-XOR-interval validator unreachable: only the selected
 * kind is ever built into the request, so "provide exactly one" cannot fail.
 *
 * The id is operator-authored with no prefill, unlike a task id. A schedule is a
 * durable named thing they have to recognise in the list a week later, and
 * `robot01-sched-1782786519` is not that.
 *
 * Reset is by remounting — TaskConsole keys this component and bumps the key
 * after a successful create — rather than by clearing five fields in an effect.
 */
export function ScheduleForm({
  existingIds,
  ready,
  reason,
  busy,
  error,
  onCreate,
}: ScheduleFormProps) {
  const [id, setId] = React.useState("");
  const [kind, setKind] = React.useState<TriggerKind>("interval");
  const [intervalText, setIntervalText] = React.useState("1800");
  const [cron, setCron] = React.useState("");
  const [timezone, setTimezone] = React.useState("");

  const trimmedId = id.trim();
  const trimmedCron = cron.trim();
  const interval = Number(intervalText.trim());

  // `id: str` has no min_length on the backend, so a blank one would reach
  // Temporal and come back as a 502 rather than a sentence about the name.
  const duplicate = trimmedId.length > 0 && existingIds.includes(trimmedId);
  const triggerOk =
    kind === "cron"
      ? trimmedCron.length > 0
      : Number.isInteger(interval) && interval > 0;

  const localReason = !trimmedId
    ? "Name the schedule."
    : duplicate
      ? "A schedule with this id already exists."
      : kind === "cron" && !trimmedCron
        ? "Enter a cron expression."
        : !triggerOk
          ? "The interval must be a whole number of seconds above zero."
          : null;

  const submittable = ready && !busy && !localReason;

  return (
    <form
      className="space-y-2"
      onSubmit={(event) => {
        event.preventDefault();
        if (!submittable) return;
        onCreate(
          trimmedId,
          kind === "cron"
            ? // timezone is omitted rather than sent-and-ignored when blank, and
              // never sent at all for an interval: the backend applies it only to
              // the cron path, and a request carrying a field with no effect is a
              // lie in the log.
              timezone.trim()
              ? { cron: trimmedCron, timezone: timezone.trim() }
              : { cron: trimmedCron }
            : { interval_seconds: interval },
        );
      }}
    >
      <label className="block">
        <span className="instrument-label text-muted-foreground">Schedule id</span>
        <Input
          value={id}
          disabled={busy}
          onChange={(event) => setId(event.target.value)}
          placeholder="robot01-daily-patrol"
          className="readout mt-0.5 h-7 rounded-sm text-[13px]"
        />
      </label>

      <div>
        <span className="instrument-label text-muted-foreground">Trigger</span>
        <div className="mt-0.5 flex items-center gap-1.5">
          <Segmented
            value={kind}
            options={TRIGGER_OPTIONS}
            disabled={busy}
            onChange={setKind}
          />
          {kind === "interval" ? (
            <>
              <Input
                inputMode="numeric"
                value={intervalText}
                disabled={busy}
                onChange={(event) => setIntervalText(event.target.value)}
                className="readout h-7 w-20 rounded-sm text-[13px]"
              />
              <span className="instrument-label text-muted-foreground">seconds</span>
            </>
          ) : (
            <Input
              value={cron}
              disabled={busy}
              onChange={(event) => setCron(event.target.value)}
              placeholder="0 9 * * 1-5"
              className="readout h-7 flex-1 rounded-sm text-[13px]"
            />
          )}
        </div>
      </div>

      {kind === "cron" && (
        <label className="block">
          <span className="instrument-label text-muted-foreground">
            Timezone <span className="font-normal">(optional, IANA)</span>
          </span>
          <Input
            value={timezone}
            disabled={busy}
            onChange={(event) => setTimezone(event.target.value)}
            placeholder="Asia/Taipei"
            className="readout mt-0.5 h-7 rounded-sm text-[13px]"
          />
        </label>
      )}

      {error && (
        <p
          role="alert"
          className="text-[11px] leading-snug break-words text-signal-warn"
        >
          {error}
        </p>
      )}

      <button
        type="submit"
        disabled={!submittable}
        className="instrument-label flex h-7 w-full items-center justify-center gap-1.5 rounded-sm bg-primary text-primary-foreground transition-opacity hover:opacity-90 disabled:opacity-40"
      >
        <CalendarPlusIcon className="size-3.5" aria-hidden />
        Create schedule
      </button>

      {(localReason ?? reason) && (
        <p className="text-[11px] leading-tight text-muted-foreground">
          {localReason ?? reason}
        </p>
      )}
    </form>
  );
}
