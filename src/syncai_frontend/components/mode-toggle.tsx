"use client";

import * as React from "react";
import { MoonIcon, SunIcon } from "lucide-react";
import { useTheme } from "next-themes";

const subscribeNoop = () => () => {};

/**
 * Day / night, one click. The rail is 56 px wide, which is no place for a
 * three-item dropdown; the full light / dark / system choice lives in Settings →
 * Appearance, and this flips between the two an operator actually switches
 * between when they walk outside.
 */
export function ModeToggle() {
  const { resolvedTheme, setTheme } = useTheme();

  // The resolved theme is unknowable during SSR, so *everything* derived from it
  // — the icon and the label that says which way the switch goes — has to wait
  // for mount. Gating only the icon still mismatched on the aria-label, because
  // the server assumed light and the client resolved dark.
  const mounted = React.useSyncExternalStore(
    subscribeNoop,
    () => true,
    () => false,
  );

  const dark = mounted && resolvedTheme === "dark";

  return (
    <button
      type="button"
      onClick={() => setTheme(dark ? "light" : "dark")}
      className="flex size-9 items-center justify-center rounded-sm text-muted-foreground transition-colors hover:bg-elevated/60 hover:text-foreground"
      aria-label={
        mounted
          ? dark
            ? "Switch to daylight mode"
            : "Switch to night mode"
          : "Switch day or night mode"
      }
    >
      {mounted ? (
        dark ? (
          <SunIcon className="size-[18px]" />
        ) : (
          <MoonIcon className="size-[18px]" />
        )
      ) : (
        <span className="size-[18px]" />
      )}
    </button>
  );
}
