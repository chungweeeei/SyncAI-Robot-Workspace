import type { Metadata, Viewport } from "next";
import { Archivo, IBM_Plex_Mono } from "next/font/google";
import "./globals.css";

import { ActiveTaskProvider } from "@/components/console/active-task-context";
import { NavRail } from "@/components/console/nav-rail";
import { RobotStateProvider } from "@/components/console/robot-state-context";
import { StatusStrip } from "@/components/console/status-strip";
import { QueryProvider } from "@/components/query-provider";
import { ThemeProvider } from "@/components/theme-provider";

// `axes: ["wdth"]` is what makes the condensed instrument labels possible —
// without it next/font ships the wght-only subset and `instrument-label` in
// globals.css silently renders at normal width.
const archivo = Archivo({
  variable: "--font-archivo",
  subsets: ["latin"],
  axes: ["wdth"],
});

const plexMono = IBM_Plex_Mono({
  variable: "--font-plex-mono",
  subsets: ["latin"],
  weight: ["400", "500", "600"],
});

export const metadata: Metadata = {
  title: "SyncAI Robot Console",
  description: "Live telemetry, map and navigation control for one robot",
};

// The console is a fixed-frame instrument panel: no page zoom-scroll, and the
// viewport is sized in dvh so mobile browser chrome does not clip the rail.
export const viewport: Viewport = {
  themeColor: [
    { media: "(prefers-color-scheme: light)", color: "#e9eef2" },
    { media: "(prefers-color-scheme: dark)", color: "#0b1014" },
  ],
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html
      lang="en"
      className={`${archivo.variable} ${plexMono.variable} antialiased`}
      suppressHydrationWarning
    >
      <body className="overflow-hidden">
        <ThemeProvider
          attribute="class"
          defaultTheme="system"
          enableSystem
          disableTransitionOnChange
        >
          {/*
           * Two polls for the whole console, and only two. The status strip and
           * whichever page is mounted read the same RobotState snapshot, so the
           * header clock can never disagree with the pose in the rail — which it
           * would if each of them called useRobotState() on its own interval.
           *
           * The second one answers "is the robot executing a task", which is a
           * fact about the machine rather than about any one screen: it has to
           * survive navigation, and it has to be true for runs this browser
           * never started. They stay separate providers because they fail
           * differently — see ActiveTaskProvider.
           *
           * QueryProvider sits outside both because both polls now live in the
           * TanStack Query cache it owns.
           */}
          <QueryProvider>
            <RobotStateProvider>
              <ActiveTaskProvider>
                <div className="flex h-dvh flex-col lg:flex-row">
                  <NavRail />
                  <div className="flex min-h-0 min-w-0 flex-1 flex-col">
                    <StatusStrip />
                    {/* Pages own their own scrolling: the dashboard must not
                     * scroll (the viewport is sized to what is left), settings
                     * must. */}
                    <main className="min-h-0 flex-1 overflow-hidden">
                      {children}
                    </main>
                  </div>
                </div>
              </ActiveTaskProvider>
            </RobotStateProvider>
          </QueryProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
