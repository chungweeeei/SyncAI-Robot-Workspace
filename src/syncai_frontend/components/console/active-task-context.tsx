"use client";

import * as React from "react";

import { useActiveTasks, type UseActiveTasks } from "@/hooks/use-active-tasks";

const ActiveTaskContext = React.createContext<UseActiveTasks | null>(null);

/**
 * Holds the console's single GET /api/v1/active_tasks poll, mounted once in the
 * root layout so it survives route changes — the same arrangement, for the same
 * reason, as RobotStateProvider beside it.
 *
 * Route survival is the whole point here rather than a nicety. "Is the robot
 * executing something" is not a fact about the dashboard; an operator on
 * /settings or /maps needs it just as much, and the answer must not blink out
 * because they navigated. That is also why this does not ride on the telemetry
 * WebSocket, which exists only while the dashboard's viewport is mounted.
 *
 * Deliberately a second provider rather than another field on RobotStateProvider:
 * different endpoint, different rate, and different failure semantics — robot
 * state 404s until the robot has localized, while this only fails when Temporal
 * is unreachable. One shared `status` would have to stand for two unrelated
 * links, and the operator would not be able to tell which one broke.
 */
export function ActiveTaskProvider({
  children,
}: {
  children: React.ReactNode;
}) {
  const value = useActiveTasks();

  return (
    <ActiveTaskContext.Provider value={value}>
      {children}
    </ActiveTaskContext.Provider>
  );
}

export function useConsoleActiveTasks(): UseActiveTasks {
  const value = React.useContext(ActiveTaskContext);
  if (!value) {
    throw new Error(
      "useConsoleActiveTasks must be used inside <ActiveTaskProvider>.",
    );
  }
  return value;
}
