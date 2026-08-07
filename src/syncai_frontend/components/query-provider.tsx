"use client";

import * as React from "react";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";

/**
 * One QueryClient for the whole console.
 *
 * Created in state rather than at module scope: these pages are prerendered,
 * and a module-level client would be shared across server renders — per-mount
 * state is the shape the TanStack SSR guide prescribes.
 *
 * Two defaults deviate from the library, both to keep the semantics the
 * hand-rolled fetch hooks had before the migration:
 *
 * - `retry: false`. Every query here is either on a poll interval (the next
 *   tick *is* the retry) or has an explicit Refresh escape hatch. The default
 *   three exponential retries would hold "loading" for ~5 s before the
 *   console's error tone appears, and the status indicators exist precisely to
 *   report a failure the moment it happens.
 * - `refetchOnWindowFocus: false`. The poll economics are deliberate —
 *   useActiveTasks outwaits the backend's 1.5 s snapshot TTL on purpose, and
 *   the schedules list costs a Temporal RPC per read — so freshness is the
 *   interval's job, not the window manager's.
 */
export function QueryProvider({ children }: { children: React.ReactNode }) {
  const [client] = React.useState(
    () =>
      new QueryClient({
        defaultOptions: {
          queries: { retry: false, refetchOnWindowFocus: false },
        },
      }),
  );

  return <QueryClientProvider client={client}>{children}</QueryClientProvider>;
}
