"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { MapIcon, RadarIcon, SlidersHorizontalIcon } from "lucide-react";

import { ModeToggle } from "@/components/mode-toggle";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";

// /model-preview is intentionally absent: it is a developer tool for checking
// the GLB, not an operator screen. Open it by hand. (See its page comment.)
const navItems = [
  { title: "Dashboard", href: "/", icon: RadarIcon },
  { title: "Maps", href: "/maps", icon: MapIcon },
  { title: "Settings", href: "/settings", icon: SlidersHorizontalIcon },
];

/** Emanating scan arcs — the console's mark, drawn from the sensor it runs on. */
function ConsoleMark({ className }: { className?: string }) {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      aria-hidden
      className={cn("size-6", className)}
    >
      <circle cx="7.5" cy="12" r="1.75" fill="currentColor" />
      <path
        d="M11.5 6.6a7.2 7.2 0 0 1 0 10.8"
        stroke="currentColor"
        strokeWidth="1.6"
        strokeLinecap="round"
      />
      <path
        d="M15.4 3.6a11.6 11.6 0 0 1 0 16.8"
        stroke="currentColor"
        strokeWidth="1.6"
        strokeLinecap="round"
        opacity="0.5"
      />
    </svg>
  );
}

/**
 * A 56 px icon rail, vertical on desktop and a bottom bar on narrow screens.
 *
 * This replaced the collapsible shadcn sidebar: two routes did not justify a
 * 256 px drawer, its trigger, and a mobile sheet — and on a console whose job is
 * to hold a map as large as possible, a quarter of the width spent on two links
 * was the worst trade on the screen.
 */
export function NavRail() {
  const pathname = usePathname();

  return (
    <nav
      aria-label="Console sections"
      className={cn(
        "flex shrink-0 items-center border-hairline bg-panel",
        // Mobile: a bottom bar, below the viewport it must not cover.
        "order-last h-14 flex-row justify-center gap-1 border-t px-2",
        // Desktop: the rail.
        "lg:order-first lg:h-auto lg:w-14 lg:flex-col lg:justify-start lg:gap-1 lg:border-t-0 lg:border-r lg:px-0 lg:py-3",
      )}
    >
      <Link
        href="/"
        aria-label="SyncAI Robot Console"
        className="hidden size-9 shrink-0 items-center justify-center text-foreground transition-opacity hover:opacity-70 lg:mb-4 lg:flex"
      >
        <ConsoleMark />
      </Link>

      <TooltipProvider delay={200}>
        {navItems.map((item) => {
          // Prefix match, so /maps/<name>/edit keeps the Maps tick lit. "/" has to
          // be exact or it would match every route.
          const active =
            item.href === "/" ? pathname === "/" : pathname.startsWith(item.href);
          return (
            <Tooltip key={item.href}>
              <TooltipTrigger
                render={
                  <Link
                    href={item.href}
                    aria-current={active ? "page" : undefined}
                    className={cn(
                      "relative flex h-9 items-center justify-center gap-2 rounded-sm px-3 transition-colors lg:w-9 lg:px-0",
                      active
                        ? "bg-elevated text-foreground"
                        : "text-muted-foreground hover:bg-elevated/60 hover:text-foreground",
                    )}
                  >
                    <item.icon className="size-[18px] shrink-0" />
                    {/* Touch targets get a label instead of a hover tooltip. */}
                    <span className="instrument-label lg:hidden">
                      {item.title}
                    </span>
                    {/* Location marker: a commanded-hue tick on the panel edge
                     * the rail is attached to. */}
                    {active && (
                      <span
                        aria-hidden
                        className="absolute -bottom-[9px] left-1/2 h-[2px] w-5 -translate-x-1/2 rounded-full bg-signal-cmd lg:-left-[9px] lg:bottom-auto lg:h-5 lg:w-[2px] lg:translate-x-0"
                      />
                    )}
                  </Link>
                }
              />
              <TooltipContent side="right" className="hidden lg:inline-flex">
                {item.title}
              </TooltipContent>
            </Tooltip>
          );
        })}
      </TooltipProvider>

      <div className="ml-auto lg:mt-auto lg:ml-0">
        <ModeToggle />
      </div>
    </nav>
  );
}
