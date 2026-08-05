import * as React from "react";

import { cn } from "@/lib/utils";

/**
 * The console's instrument vocabulary — the parts every telemetry surface is
 * built from. Deliberately not shadcn Cards: a card's padding, border and title
 * chrome around four numbers is most of the pixels spent on the frame rather
 * than the value. A group here is a condensed caps label, a hairline, and rows.
 *
 * `tone` is the EFIS colour semantic from globals.css, not a visual choice:
 * pick it by what the value *is* (measured / commanded / degraded / faulted),
 * and the same hue will mean the same thing on the map canvas.
 */
export type Tone = "neutral" | "live" | "cmd" | "active" | "caution" | "warn";

/** Text colour per tone. Exported for surfaces that colour a value themselves
 *  (the motor grid) instead of going through Readout. */
export const TONE_TEXT: Record<Tone, string> = {
  neutral: "text-foreground",
  live: "text-signal-live",
  cmd: "text-signal-cmd",
  active: "text-signal-active",
  caution: "text-signal-caution",
  warn: "text-signal-warn",
};

const TONE_FILL: Record<Tone, string> = {
  neutral: "bg-muted-foreground",
  live: "bg-signal-live",
  cmd: "bg-signal-cmd",
  active: "bg-signal-active",
  caution: "bg-signal-caution",
  warn: "bg-signal-warn",
};

const TONE_CHIP: Record<Tone, string> = {
  neutral: "border-hairline text-muted-foreground",
  live: "border-signal-live/40 text-signal-live bg-signal-live/8",
  cmd: "border-signal-cmd/40 text-signal-cmd bg-signal-cmd/8",
  active: "border-signal-active/40 text-signal-active bg-signal-active/8",
  caution: "border-signal-caution/40 text-signal-caution bg-signal-caution/8",
  warn: "border-signal-warn/40 text-signal-warn bg-signal-warn/8",
};

export function InstrumentGroup({
  label,
  caption,
  action,
  className,
  children,
}: {
  label: string;
  /** One muted line under the rows — use it to say where a number came from. */
  caption?: string;
  /** Controls that belong to the group, right-aligned on the label row. */
  action?: React.ReactNode;
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <section
      className={cn("border-b border-hairline px-4 py-3.5 last:border-b-0", className)}
    >
      <header className="mb-2.5 flex h-4 items-center justify-between gap-2">
        <h2 className="instrument-label text-muted-foreground">{label}</h2>
        {action}
      </header>
      <div className="space-y-1.5">{children}</div>
      {caption && (
        <p className="mt-2.5 text-[11px] leading-tight text-muted-foreground">
          {caption}
        </p>
      )}
    </section>
  );
}

export function Readout({
  label,
  value,
  unit,
  tone = "neutral",
  className,
}: {
  label: string;
  value: React.ReactNode;
  unit?: string;
  tone?: Tone;
  className?: string;
}) {
  return (
    <div className={cn("flex items-baseline justify-between gap-3", className)}>
      <span className="instrument-label shrink-0 text-muted-foreground">
        {label}
      </span>
      <span
        className={cn(
          "readout min-w-0 truncate text-[13px] font-medium",
          TONE_TEXT[tone],
        )}
      >
        {value}
        {unit && (
          <span className="ml-1 text-[11px] font-normal text-muted-foreground">
            {unit}
          </span>
        )}
      </span>
    </div>
  );
}

/** The one big number in a group. Used for the pose the operator actually reads. */
export function PrimaryReadout({
  label,
  value,
  unit,
  tone = "live",
}: {
  label: string;
  value: string;
  unit?: string;
  tone?: Tone;
}) {
  return (
    <div>
      <span className="instrument-label text-muted-foreground">{label}</span>
      <p
        className={cn(
          "readout mt-0.5 text-2xl leading-none font-medium",
          TONE_TEXT[tone],
        )}
      >
        {value}
        {unit && (
          <span className="ml-1 text-xs font-normal text-muted-foreground">
            {unit}
          </span>
        )}
      </p>
    </div>
  );
}

export function Chip({
  tone = "neutral",
  title,
  className,
  children,
}: {
  tone?: Tone;
  /** Hover text, for a chip whose two words stand in for a longer fact. */
  title?: string;
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <span
      title={title}
      className={cn(
        "instrument-label inline-flex h-5 items-center rounded-sm border px-1.5",
        TONE_CHIP[tone],
        className,
      )}
    >
      {children}
    </span>
  );
}

/** Four-bar RSSI meter. Bars, not a number, is how signal strength is read. */
export function SignalBars({
  bars,
  tone = "live",
  className,
}: {
  bars: number;
  tone?: Tone;
  className?: string;
}) {
  return (
    <span
      className={cn("flex items-end gap-[2px]", className)}
      aria-label={`Signal ${bars} of 4`}
    >
      {[1, 2, 3, 4].map((level) => (
        <span
          key={level}
          className={cn(
            "w-[3px] rounded-[1px]",
            level <= bars ? TONE_FILL[tone] : "bg-hairline",
          )}
          style={{ height: `${3 + level * 2.5}px` }}
        />
      ))}
    </span>
  );
}

/**
 * Segmented level meter (battery). Discrete cells rather than a smooth bar:
 * a continuous fill invites reading a precision the 1 Hz integer percentage
 * does not have, and cells are countable out of the corner of an eye.
 */
export function SegmentMeter({
  value,
  segments = 5,
  tone = "live",
  className,
}: {
  /** 0–100. */
  value: number;
  segments?: number;
  tone?: Tone;
  className?: string;
}) {
  const filled = Math.round((Math.min(Math.max(value, 0), 100) / 100) * segments);

  return (
    <span
      className={cn("flex items-center gap-[2px]", className)}
      aria-label={`${Math.round(value)} percent`}
    >
      {Array.from({ length: segments }, (_, i) => (
        <span
          key={i}
          className={cn(
            "h-3 w-[5px] rounded-[1px]",
            i < filled ? TONE_FILL[tone] : "bg-hairline",
          )}
        />
      ))}
    </span>
  );
}

/**
 * Segmented selector for view state (2D / 3D, camera mode, motion). Commanded
 * hue on the active segment: it is a value the operator set, not one measured.
 */
export function Segmented<T extends string>({
  value,
  options,
  onChange,
  stretch = false,
  disabled = false,
  className,
}: {
  /**
   * `null` lights no segment.
   *
   * For a control whose current value can be something none of the options
   * represents. The locomotion policy row is the motivating case: it lights what
   * the robot *reports*, and the robot can report a policy the command surface
   * deliberately does not expose (CHAMP / ISSAC), or none at all. Lighting the
   * nearest option instead would be a claim about the robot.
   */
  value: T | null;
  options: readonly { value: T; label: string }[];
  onChange: (value: T) => void;
  /**
   * Fill the container, splitting it into equal segments.
   *
   * The default hugs its labels, which is right for a control sized by its
   * content. Inside a fixed-width panel it is wrong: the row does not wrap, so a
   * set of long labels (FREE / UNKNOWN / OBSTACLE in the gridmap editor) simply
   * overflows the panel's border.
   */
  stretch?: boolean;
  /** Dim the whole control and refuse every segment. */
  disabled?: boolean;
  className?: string;
}) {
  return (
    <div
      className={cn(
        "overflow-hidden rounded-sm border border-hairline",
        stretch ? "flex w-full" : "inline-flex",
        disabled && "opacity-40",
        className,
      )}
    >
      {options.map((option) => {
        const active = option.value === value;
        return (
          <button
            key={option.value}
            type="button"
            aria-pressed={active}
            disabled={disabled}
            onClick={() => onChange(option.value)}
            className={cn(
              "instrument-label h-6 px-2 transition-colors",
              "border-l border-hairline first:border-l-0",
              // The dimming lives on the container, so a disabled segment only
              // has to stop reacting — two stacked opacities would be muddy.
              disabled && "hover:bg-transparent hover:text-muted-foreground",
              // min-w-0 so a segment can shrink below its label; truncate rather
              // than let one long label push the row past the container. The
              // padding drops to px-1 because flex-1 is already doing the spacing,
              // and px-2 on four segments is what pushes the last label over.
              stretch && "min-w-0 flex-1 truncate px-1",
              active
                ? "bg-signal-cmd/12 text-signal-cmd"
                : "text-muted-foreground hover:bg-elevated hover:text-foreground",
            )}
          >
            {option.label}
          </button>
        );
      })}
    </div>
  );
}

/**
 * Shared chrome for controls that float on top of the viewport. They sit over a
 * canvas that can be any colour, so they need their own opaque-ish ground —
 * this is the only place in the console that uses a blur.
 */
export const overlayPanel =
  "rounded-sm border border-hairline bg-panel/85 shadow-sm backdrop-blur-sm";

/** Vertical hairline between strip sections. */
export function StripDivider({ className }: { className?: string }) {
  return (
    <span
      aria-hidden
      className={cn("h-5 w-px shrink-0 bg-hairline", className)}
    />
  );
}
