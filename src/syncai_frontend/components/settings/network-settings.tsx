"use client";

import * as React from "react";
import { RefreshCwIcon, WifiIcon } from "lucide-react";

import { Chip, SignalBars, rssiToBars } from "@/components/console/instrument";
import { PasswordInput } from "@/components/settings/password-input";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
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
import { useWifiConnect } from "@/hooks/use-wifi-connect";
import { useWifiScan } from "@/hooks/use-wifi-scan";
import type { WifiNetwork } from "@/lib/api/network";
import { cn } from "@/lib/utils";

/**
 * The robot's WiFi: pick a network the robot can see, give it a password, join.
 *
 * Takes no props. The card used to receive `network_status` from the page and
 * was hidden entirely until the first state frame — but scanning and joining
 * do not depend on the state poll, and a console that has *never* heard from
 * the robot is exactly the one that needs to fix its WiFi. So the card always
 * renders, and only the current-network readout degrades to dashes.
 *
 * There is no DHCP / static-IP block any more: the backend has no endpoint for
 * addressing (only scan + connect), and a form that looked settable but saved
 * nothing was worse than none. The IP and MAC stay as a readout.
 *
 * The dropdown is the whole story of "which network": the backend's scan
 * skips hidden SSIDs and there is deliberately no free-text entry — an
 * operator typing an SSID by hand was the bug the picker replaces.
 */
export function NetworkSettings() {
  const scan = useWifiScan();
  const wifi = useWifiConnect();

  const [picked, setPicked] = React.useState<string | null>(null);
  const [password, setPassword] = React.useState("");

  const current = wifi.current;
  const currentSsid = current?.ssid || null;
  // The dropdown defaults to the connected network with no effect to sync it:
  // `picked` is only what the operator changed it to.
  const ssid = picked ?? currentSsid;

  // The scan does not mark the network the robot is on, and may not even list
  // it (a rescan mid-roam, or the AP falling below nmcli's floor). The
  // controlled Select needs an item for every value it can hold, or the
  // trigger shows nothing for the very network the operator cares most about
  // — so the connected network is synthesised into the list from the state
  // poll when the scan missed it, and always sorts first.
  const options = React.useMemo<WifiNetwork[]>(() => {
    const scanned = scan.networks ?? [];
    const list =
      currentSsid && current && !scanned.some((n) => n.ssid === currentSsid)
        ? [{ ssid: current.ssid, bssid: current.bssid, rssi: current.rssi }, ...scanned]
        : [...scanned];
    return list.sort((a, b) => {
      if (a.ssid === currentSsid) return -1;
      if (b.ssid === currentSsid) return 1;
      return b.rssi - a.rssi;
    });
  }, [scan.networks, current, currentSsid]);

  const isCurrent = ssid !== null && ssid === currentSsid;
  const canConnect = ssid !== null && !wifi.busy && !isCurrent;

  const submit = async () => {
    if (!canConnect || ssid === null) return;
    const outcome = await wifi.connect(ssid, password);
    // A secret should not sit in React state once it has been used. Kept only
    // on a refusal, so a typo can be fixed and resubmitted without retyping.
    if (outcome !== "error") setPassword("");
  };

  const placeholder = scan.scanning
    ? "Scanning…"
    : options.length
      ? "Pick a network"
      : "No networks found";

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <WifiIcon className="size-4 text-muted-foreground" />
          Network
        </CardTitle>
        <CardDescription>
          Choose a WiFi network for the robot. The password is optional for an
          open network.
        </CardDescription>
      </CardHeader>
      <form
        // A form so Enter in the password field connects, which is what a
        // pick-and-type row is expected to do.
        onSubmit={(event) => {
          event.preventDefault();
          void submit();
        }}
      >
        <CardContent className="space-y-4">
          <div className="grid gap-2">
            <Label htmlFor="wifi-ssid">Network</Label>
            <div className="flex items-center gap-2">
              <Select
                // `items` is what makes SelectValue render the *label* — here the
                // bars + name + dBm row — instead of the raw SSID string.
                items={options.map((network) => ({
                  value: network.ssid,
                  label: (
                    <WifiOptionLabel
                      network={network}
                      connected={network.ssid === currentSsid}
                    />
                  ),
                }))}
                value={ssid}
                disabled={wifi.busy || (scan.scanning && options.length === 0)}
                onValueChange={(next) => setPicked(next as string)}
              >
                <SelectTrigger id="wifi-ssid" className="w-full flex-1">
                  <SelectValue placeholder={placeholder} />
                </SelectTrigger>
                <SelectContent>
                  {options.map((network) => (
                    <SelectItem key={network.ssid} value={network.ssid}>
                      <WifiOptionLabel
                        network={network}
                        connected={network.ssid === currentSsid}
                      />
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              <Button
                type="button"
                variant="outline"
                size="sm"
                disabled={scan.scanning || wifi.busy}
                onClick={scan.rescan}
              >
                <RefreshCwIcon
                  data-icon="inline-start"
                  className={cn(scan.scanning && "animate-spin")}
                />
                Rescan
              </Button>
            </div>
            {scan.scanning ? (
              <Hint>Scanning for networks — this takes up to 45 seconds.</Hint>
            ) : scan.status === "error" && scan.error ? (
              // The backend's own sentence, verbatim. A failed rescan keeps
              // the previous list in the dropdown, so this is a note, not a wall.
              <Hint className="text-signal-warn">{scan.error}</Hint>
            ) : null}
          </div>

          <div className="grid gap-2">
            <Label htmlFor="wifi-password">Password</Label>
            <PasswordInput
              id="wifi-password"
              value={password}
              disabled={wifi.busy}
              placeholder="Leave empty for an open network"
              onChange={(event) => setPassword(event.target.value)}
            />
          </div>

          <Separator />

          <dl className="space-y-1 text-sm">
            <div className="flex items-center justify-between gap-4">
              <dt className="text-muted-foreground">Connected to</dt>
              <dd className="flex min-w-0 items-center gap-2 font-medium">
                {currentSsid && current ? (
                  <>
                    <SignalBars bars={rssiToBars(current.rssi)} />
                    <span className="truncate">{current.ssid}</span>
                    <span className="readout text-[11px] text-muted-foreground">
                      {current.rssi} dBm
                    </span>
                  </>
                ) : (
                  "—"
                )}
              </dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-muted-foreground">Current IP</dt>
              <dd className="font-medium tabular-nums">
                {current?.ip_address || "—"}
              </dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-muted-foreground">MAC address</dt>
              <dd className="font-medium tabular-nums">
                {current?.mac_address || "—"}
              </dd>
            </div>
          </dl>
          {!current && (
            <Hint>
              {wifi.stateStatus === "loading"
                ? "Waiting for the first state frame."
                : "The robot has not published a state frame, so its current network cannot be read."}
            </Hint>
          )}

          {/* Result lines, in place rather than a toast: the operator is
              looking at this card for the next minute anyway. */}
          {wifi.pending && (
            <Hint className="text-signal-caution">
              Connecting to <span className="readout">{wifi.pending}</span>… the
              robot reports its network once a second.
            </Hint>
          )}
          {wifi.message && (
            <Hint className="text-signal-live">{wifi.message}</Hint>
          )}
          {wifi.error && (
            <Hint role="alert" className="text-signal-warn">
              {wifi.error}
            </Hint>
          )}
        </CardContent>
        <CardFooter className="justify-end">
          <Button type="submit" disabled={!canConnect}>
            <WifiIcon data-icon="inline-start" />
            {wifi.busy ? "Connecting…" : isCurrent ? "Connected" : "Connect"}
          </Button>
        </CardFooter>
      </form>
    </Card>
  );
}

/**
 * One network as a row: bars, name, dBm, and a Connected chip for the one the
 * robot is on. Used for both the Select's `items[].label` (what the trigger
 * shows) and each SelectItem's children (what the list shows), so the two can
 * never drift apart.
 */
function WifiOptionLabel({
  network,
  connected,
}: {
  network: WifiNetwork;
  connected: boolean;
}) {
  return (
    <span className="flex min-w-0 items-center gap-2">
      <SignalBars
        bars={rssiToBars(network.rssi)}
        tone={connected ? "live" : "neutral"}
      />
      <span className="truncate">{network.ssid}</span>
      <span className="readout text-[11px] text-muted-foreground">
        {network.rssi} dBm
      </span>
      {connected && <Chip tone="live">Connected</Chip>}
    </span>
  );
}

function Hint({
  className,
  children,
  ...props
}: React.ComponentProps<"p">) {
  return (
    <p
      className={cn("text-[11px] leading-snug text-muted-foreground", className)}
      {...props}
    >
      {children}
    </p>
  );
}
