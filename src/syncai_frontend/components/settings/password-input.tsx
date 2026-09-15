"use client";

import * as React from "react";
import { EyeIcon, EyeOffIcon } from "lucide-react";

import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { cn } from "@/lib/utils";

/**
 * A password field with a show/hide toggle.
 *
 * The console's Input has no adornment slot, so this is the plain wrapper: a
 * relative box, the input padded on the right, and an icon button sitting in
 * that padding. Two details are load-bearing:
 *
 * - `type="button"` on the toggle. It lives inside the Connect form, and
 *   base-ui's Button does not promise a `type` default, so without it a click
 *   on the eye would submit the form — sending a half-typed password to nmcli.
 * - `autoComplete="off"`. Browsers otherwise offer to save the robot's WiFi
 *   password against the console origin, which is the wrong place for it.
 *
 * The toggle stays in the tab order on purpose. The common `tabIndex={-1}`
 * trick keeps Tab flowing password → Connect, but it makes the eye unreachable
 * from a keyboard; one extra stop on a two-field form is the cheaper trade.
 */
export function PasswordInput({
  className,
  disabled,
  ...props
}: Omit<React.ComponentProps<typeof Input>, "type">) {
  const [visible, setVisible] = React.useState(false);

  return (
    <div className="relative">
      <Input
        {...props}
        type={visible ? "text" : "password"}
        autoComplete="off"
        disabled={disabled}
        className={cn("pr-9", className)}
      />
      <Button
        type="button"
        variant="ghost"
        size="icon-sm"
        disabled={disabled}
        aria-label={visible ? "Hide password" : "Show password"}
        aria-pressed={visible}
        className="absolute top-1/2 right-0.5 -translate-y-1/2 text-muted-foreground"
        onClick={() => setVisible((prev) => !prev)}
      >
        {visible ? <EyeOffIcon /> : <EyeIcon />}
      </Button>
    </div>
  );
}
