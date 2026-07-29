"use client";

import { LocateFixedIcon, RadioTowerIcon } from "lucide-react";

import { Chip, Readout, overlayPanel } from "@/components/console/instrument";
import { cn } from "@/lib/utils";
import type { InitialPoseEstimate } from "@/hooks/use-initial-pose";

/**
 * Operator controls for the drag-an-initial-pose flow (RViz's "2D Pose
 * Estimate"): arm the mode, read back the staged pose, publish it to the
 * localizer.
 *
 * Sits under the goal control and mirrors its shape, because it is the same
 * gesture producing a different kind of pose — but everything it draws is in the
 * caution hue, not the commanded cyan a goal uses. That is the point of the
 * distinction: a goal is a place to drive to, while this overrides the robot's
 * own estimate of where it already is. Getting it wrong does not send the robot
 * somewhere unintended, it makes the whole map frame wrong, which is worse and
 * quieter. Amber, and a separate button, keep the two from being confused
 * mid-drag.
 *
 * All state lives in `useInitialPose` (plus the view's pick mode); this
 * component is presentation only.
 */
export function InitialPoseControl({
  estimate,
  armed,
  onArm,
  className,
}: {
  estimate: InitialPoseEstimate;
  /** True while a drag on the viewport will produce an initial pose. */
  armed: boolean;
  onArm: () => void;
  className?: string;
}) {
  const { pose, published, busy, error } = estimate;

  return (
    <div className={cn("flex flex-col items-start gap-2", className)}>
      <button
        type="button"
        onClick={onArm}
        className={cn(
          overlayPanel,
          "instrument-label flex h-7 items-center gap-1.5 px-2 transition-colors",
          armed
            ? "border-signal-caution/50 bg-signal-caution/12 text-signal-caution"
            : "hover:bg-elevated",
        )}
      >
        <LocateFixedIcon className="size-3.5" />
        {armed ? "Drag where the robot is" : "Set initial pose"}
      </button>

      {(pose || error) && (
        <div className={cn(overlayPanel, "w-full p-2.5")}>
          {pose && (
            <div className="space-y-1">
              <Readout
                label="Est X"
                value={pose.x.toFixed(2)}
                unit="m"
                tone="caution"
              />
              <Readout
                label="Est Y"
                value={pose.y.toFixed(2)}
                unit="m"
                tone="caution"
              />
              <Readout
                label="Heading"
                value={pose.theta.toFixed(1)}
                unit="°"
                tone="caution"
              />
            </div>
          )}

          {/* "Published", not "Localized": the topic has no ack and the
            * localizer only takes this as an ICP seed, so the honest claim is
            * that the sample went out. Whether it converged is answered by the
            * robot in the viewport moving to the marker. */}
          {published && (
            <div className="mt-2.5 flex items-center justify-between gap-2">
              <span className="instrument-label text-muted-foreground">
                Estimate
              </span>
              <Chip tone="live">PUBLISHED</Chip>
            </div>
          )}

          {error && (
            <p className="mt-2.5 text-[11px] leading-snug break-words text-signal-warn">
              {error}
            </p>
          )}

          <div className="mt-2.5 flex gap-1.5">
            <button
              type="button"
              disabled={busy || !pose}
              onClick={estimate.publish}
              className="instrument-label flex h-7 flex-1 items-center justify-center gap-1.5 rounded-sm border border-signal-caution/50 bg-signal-caution/12 text-signal-caution transition-colors hover:bg-signal-caution/20 disabled:opacity-50"
            >
              <RadioTowerIcon className="size-3.5" />
              {published ? "Publish again" : "Publish"}
            </button>
            <button
              type="button"
              disabled={busy}
              onClick={estimate.clear}
              className="instrument-label h-7 rounded-sm border border-hairline px-2 text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-50"
            >
              Clear
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
