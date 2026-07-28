"use client";

import * as React from "react";

import { useRobotState, type UseRobotState } from "@/hooks/use-robot-state";

const RobotStateContext = React.createContext<UseRobotState | null>(null);

/**
 * Holds the console's single GET /api/v1/robot/state poll, mounted once in the
 * root layout so it survives route changes. Before this existed every screen
 * called useRobotState() itself, which meant the header and the page each ran
 * their own interval against a 1 Hz topic and could show different frames.
 */
export function RobotStateProvider({
  children,
}: {
  children: React.ReactNode;
}) {
  const value = useRobotState();

  return (
    <RobotStateContext.Provider value={value}>
      {children}
    </RobotStateContext.Provider>
  );
}

export function useConsoleRobotState(): UseRobotState {
  const value = React.useContext(RobotStateContext);
  if (!value) {
    throw new Error(
      "useConsoleRobotState must be used inside <RobotStateProvider>.",
    );
  }
  return value;
}
