"use client";

import * as React from "react";

import { cn } from "@/lib/utils";

/**
 * A bare square icon button for a list row's action cluster.
 *
 * Extracted when the third copy was about to be written (step rows, schedule
 * rows, library rows). Not in `components/ui/`: that directory is generated
 * shadcn output, and this is a local idiom the console already had twice —
 * right-aligned, unlabelled, `aria-label` + `title` carrying the meaning, sized
 * for a touch target rather than for text.
 *
 * `label` sets both the accessible name and the tooltip, because an icon-only
 * control with no title is unusable with a mouse and one with no aria-label is
 * unusable without a screen.
 */
export function IconButton({
  label,
  disabled,
  onClick,
  className,
  children,
}: {
  label: string;
  disabled: boolean;
  onClick: () => void;
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <button
      type="button"
      aria-label={label}
      title={label}
      disabled={disabled}
      onClick={onClick}
      className={cn(
        "flex size-6 items-center justify-center rounded-sm text-muted-foreground transition-colors hover:bg-elevated hover:text-foreground disabled:opacity-30 disabled:hover:bg-transparent",
        className,
      )}
    >
      {children}
    </button>
  );
}
