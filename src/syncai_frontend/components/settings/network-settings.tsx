"use client";

import * as React from "react";
import { WifiIcon } from "lucide-react";

import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { Switch } from "@/components/ui/switch";
import type { RobotNetworkStatus } from "@/lib/types/robot";

export function NetworkSettings({
  network,
}: {
  network: RobotNetworkStatus;
}) {
  const [ssid, setSsid] = React.useState(network.ssid);
  const [password, setPassword] = React.useState("");
  const [dhcp, setDhcp] = React.useState(true);
  const [staticIp, setStaticIp] = React.useState(network.ip_address);
  const [gateway, setGateway] = React.useState("192.168.0.1");
  const [dns, setDns] = React.useState("8.8.8.8");

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <WifiIcon className="size-4 text-muted-foreground" />
          Network
        </CardTitle>
        <CardDescription>
          Configure the robot&apos;s WiFi connection and IP addressing.
        </CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="grid gap-2">
          <Label htmlFor="wifi-ssid">WiFi SSID</Label>
          <Input
            id="wifi-ssid"
            value={ssid}
            onChange={(e) => setSsid(e.target.value)}
            placeholder="Network name"
          />
        </div>
        <div className="grid gap-2">
          <Label htmlFor="wifi-password">WiFi Password</Label>
          <Input
            id="wifi-password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="••••••••"
          />
        </div>

        <Separator />

        <div className="flex items-center justify-between">
          <div className="space-y-0.5">
            <Label htmlFor="dhcp">DHCP</Label>
            <p className="text-sm text-muted-foreground">
              Obtain an IP address automatically.
            </p>
          </div>
          <Switch id="dhcp" checked={dhcp} onCheckedChange={setDhcp} />
        </div>
        <div className="grid gap-2">
          <Label htmlFor="static-ip">Static IP</Label>
          <Input
            id="static-ip"
            value={staticIp}
            onChange={(e) => setStaticIp(e.target.value)}
            disabled={dhcp}
          />
        </div>
        <div className="grid gap-4 sm:grid-cols-2">
          <div className="grid gap-2">
            <Label htmlFor="gateway">Gateway</Label>
            <Input
              id="gateway"
              value={gateway}
              onChange={(e) => setGateway(e.target.value)}
              disabled={dhcp}
            />
          </div>
          <div className="grid gap-2">
            <Label htmlFor="dns">DNS</Label>
            <Input
              id="dns"
              value={dns}
              onChange={(e) => setDns(e.target.value)}
              disabled={dhcp}
            />
          </div>
        </div>

        <Separator />

        <dl className="space-y-1 text-sm">
          <div className="flex justify-between">
            <dt className="text-muted-foreground">Current IP</dt>
            <dd className="font-medium tabular-nums">{network.ip_address}</dd>
          </div>
          <div className="flex justify-between">
            <dt className="text-muted-foreground">MAC address</dt>
            <dd className="font-medium tabular-nums">{network.mac_address}</dd>
          </div>
        </dl>
      </CardContent>
      <CardFooter className="justify-end">
        {/* Stub — wire to POST /api/v1/network once backend integration lands */}
        <Button onClick={() => console.log("save network settings (stub)")}>
          Save changes
        </Button>
      </CardFooter>
    </Card>
  );
}
