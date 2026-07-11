"use client";

import * as React from "react";
import { PaletteIcon } from "lucide-react";
import { useTheme } from "next-themes";

import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Label } from "@/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";

const themeOptions = [
  { value: "light", label: "Light" },
  { value: "dark", label: "Dark" },
  { value: "system", label: "System" },
];

const subscribeNoop = () => () => {};

export function AppearanceSettings() {
  const { theme, setTheme } = useTheme();
  const [showWaypointLabels, setShowWaypointLabels] = React.useState(true);
  // Hydration-safe mounted check: the theme is unknown until the client mounts
  const mounted = React.useSyncExternalStore(
    subscribeNoop,
    () => true,
    () => false,
  );

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <PaletteIcon className="size-4 text-muted-foreground" />
          Appearance & General
        </CardTitle>
        <CardDescription>
          Customize how the console looks and behaves.
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex items-center justify-between">
          <div className="space-y-0.5">
            <Label>Theme</Label>
            <p className="text-sm text-muted-foreground">
              Select the console color scheme.
            </p>
          </div>
          {mounted ? (
            <Select
              items={themeOptions}
              value={theme ?? "system"}
              onValueChange={(value) => setTheme(value as string)}
            >
              <SelectTrigger className="w-32">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {themeOptions.map((option) => (
                  <SelectItem key={option.value} value={option.value}>
                    {option.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          ) : (
            <div className="h-8 w-32 rounded-lg border border-input" />
          )}
        </div>

        <Separator />

        <div className="flex items-center justify-between">
          <div className="space-y-0.5">
            <Label htmlFor="waypoint-labels">Waypoint labels</Label>
            <p className="text-sm text-muted-foreground">
              Show waypoint names on the dashboard map.
            </p>
          </div>
          <Switch
            id="waypoint-labels"
            checked={showWaypointLabels}
            onCheckedChange={setShowWaypointLabels}
          />
        </div>

        <Separator />

        <div className="flex items-center justify-between">
          <div className="space-y-0.5">
            <Label>Language</Label>
            <p className="text-sm text-muted-foreground">
              Console display language.
            </p>
          </div>
          <Select items={[{ value: "en", label: "English" }]} value="en" disabled>
            <SelectTrigger className="w-32">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="en">English</SelectItem>
            </SelectContent>
          </Select>
        </div>
      </CardContent>
    </Card>
  );
}
