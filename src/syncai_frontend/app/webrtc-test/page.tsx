"use client";

import * as React from "react";
import { MicIcon, MicOffIcon, PlayIcon, SquareIcon, Volume2Icon, VolumeXIcon } from "lucide-react";

import { Chip, InstrumentGroup, Readout, Segmented } from "@/components/console/instrument";
import { Button } from "@/components/ui/button";
import { apiUrl } from "@/lib/api/config";
import { startDuplexSession } from "@/lib/video/duplex";
import { readWhepStats, startWhepSession, type WhepStats } from "@/lib/video/whep";
import {
  INSECURE_CONTEXT_MESSAGE,
  microphoneAvailable,
  readWhipStats,
  startWhipSession,
  type WhipSession,
  type WhipStats,
} from "@/lib/video/whip";
import { cn } from "@/lib/utils";

/**
 * Backend-free-standing test bench for the robot's WHEP camera stream.
 *
 * The dashboard has no camera panel yet, so without this route the only way to
 * exercise POST /api/v1/webrtc/whep is curl — which cannot complete an ICE
 * handshake, so it can prove the 400s and the 502s and nothing else. This page
 * is the first client that can actually establish a session, and it is built to
 * answer the questions that go wrong in that order:
 *
 *   1. Did signalling work?      — the session id and the Location URL
 *   2. Is CORS right?            — "Location readable" is the expose_headers
 *                                  check; without it teardown silently breaks
 *   3. Is video arriving?        — resolution, fps, kbps, frames decoded
 *   4. Is audio arriving?        — a level meter, because the Opus track rides
 *                                  the same GStreamer pipeline as the camera
 *                                  and speakers are the last thing to trust
 *   5. How is the path?          — candidate pair and RTT
 *
 * Deliberately not in the nav rail, same as /model-preview: a developer tool,
 * opened by hand at /webrtc-test.
 *
 * Both directions are here now. The robot's microphone reaches the browser as
 * the WHEP session's second track; the browser's microphone reaches the
 * robot's speaker over WHIP, which is a separate session with its own button
 * because the two are independent -- listening without talking is the common
 * case, and a single control would tie the camera to the microphone
 * permission.
 *
 * The WHIP half needs a secure context and this page is usually opened over
 * plain http, so it degrades to an explanation rather than a broken button;
 * see lib/video/whip.ts.
 */

const STATS_POLL_MS = 500;

/** A useSyncExternalStore subscription for a value that cannot change. */
const neverChanges = () => () => {};
const BACKEND_STATUS_POLL_MS = 2000;

/** GET /api/v1/webrtc/status, the backend's view of its own session slots. */
interface SlotStatus {
  session_id: string;
  age_seconds: number;
}
interface BackendStatus {
  library_loaded: boolean;
  video: SlotStatus | null;
  audio: SlotStatus | null;
  duplex: SlotStatus | null;
}

type Phase = "idle" | "connecting" | "live" | "failed";

/**
 * Which negotiation this page is exercising.
 *
 * "split" is two independent sessions — WHEP for the camera, WHIP for the
 * microphone — and is the default because watching without talking needs no
 * microphone permission. "duplex" is one peer connection carrying both, which
 * trades that independence for a single handshake.
 *
 * They contend: a duplex session holds the camera and the speaker together, so
 * the backend preempts whichever kind is live when the other is created. That
 * is why the switch is disabled while anything is connected rather than
 * silently stealing the devices from the session you are watching.
 */
type Mode = "split" | "duplex";

const MODE_OPTIONS = [
  { value: "split" as const, label: "WHEP + WHIP" },
  { value: "duplex" as const, label: "Duplex" },
];

/**
 * The camera-bearing session, whichever kind produced it.
 *
 * Both clients return the same four things the page needs, so normalising here
 * keeps the video element, the stats pump and the teardown path from each
 * growing a branch on the mode.
 */
interface ActiveSession {
  kind: Mode;
  stream: MediaStream;
  pc: RTCPeerConnection;
  sessionUrl: string | null;
  close: () => Promise<void>;
}

export default function WebRtcTestPage() {
  const videoRef = React.useRef<HTMLVideoElement>(null);
  const sessionRef = React.useRef<ActiveSession | null>(null);

  const [mode, setMode] = React.useState<Mode>("split");
  const [phase, setPhase] = React.useState<Phase>("idle");
  const [error, setError] = React.useState<string | null>(null);
  const [connectionState, setConnectionState] = React.useState<string>("—");
  const [iceState, setIceState] = React.useState<string>("—");
  const [sessionUrl, setSessionUrl] = React.useState<string | null>(null);
  const [locationReadable, setLocationReadable] = React.useState<boolean | null>(null);
  const [stats, setStats] = React.useState<WhepStats | null>(null);
  const [muted, setMuted] = React.useState(false);
  const [backend, setBackend] = React.useState<BackendStatus | null>(null);
  const [backendError, setBackendError] = React.useState<string | null>(null);

  const whipRef = React.useRef<WhipSession | null>(null);
  const [talkPhase, setTalkPhase] = React.useState<Phase>("idle");
  const [talkError, setTalkError] = React.useState<string | null>(null);
  const [whipStats, setWhipStats] = React.useState<WhipStats | null>(null);
  // Resolved on the client only: navigator does not exist during SSR, and a
  // server-rendered "unavailable" would be wrong on an https console. Via
  // useSyncExternalStore rather than an effect because the value never changes
  // after mount -- there is nothing to synchronise, only a browser fact to
  // read once the browser exists. null is the server's answer, and the UI
  // treats it as "not known yet" rather than as "unavailable".
  const micAvailable = React.useSyncExternalStore(
    neverChanges,
    microphoneAvailable,
    () => null,
  );

  const stop = React.useCallback(async () => {
    const session = sessionRef.current;
    sessionRef.current = null;
    if (videoRef.current) videoRef.current.srcObject = null;
    setPhase("idle");
    setStats(null);
    setConnectionState("—");
    setIceState("—");
    setSessionUrl(null);
    setLocationReadable(null);
    await session?.close();
  }, []);

  const start = React.useCallback(async () => {
    if (sessionRef.current) return;
    setPhase("connecting");
    setError(null);

    const handlers = {
      onConnectionState: setConnectionState,
      onIceState: setIceState,
    };

    try {
      // The only place the two negotiations differ for this page: what the
      // camera stream is called on the way back. Everything downstream — the
      // <video> element, both stats pumps, teardown — works off the
      // normalised session.
      const session: ActiveSession =
        mode === "duplex"
          ? await startDuplexSession(handlers).then((s) => ({
              kind: "duplex" as const,
              stream: s.remote,
              pc: s.pc,
              sessionUrl: s.sessionUrl,
              close: s.close,
            }))
          : await startWhepSession(handlers).then((s) => ({
              kind: "split" as const,
              stream: s.stream,
              pc: s.pc,
              sessionUrl: s.sessionUrl,
              close: s.close,
            }));

      sessionRef.current = session;
      setSessionUrl(session.sessionUrl);
      setLocationReadable(session.sessionUrl !== null);

      const video = videoRef.current;
      if (video) {
        video.srcObject = session.stream;
        // play() is called from inside the click handler's task, so the
        // autoplay policy lets the audio track through unmuted. Starting this
        // page on load instead would get a silent stream and no error.
        try {
          await video.play();
        } catch {
          /* the element is also controls-enabled; the operator can press play */
        }
      }
      setPhase("live");
    } catch (e) {
      await stop();
      setPhase("failed");
      setError(e instanceof Error ? e.message : String(e));
    }
  }, [mode, stop]);

  const stopTalking = React.useCallback(async () => {
    const session = whipRef.current;
    whipRef.current = null;
    setTalkPhase("idle");
    setWhipStats(null);
    await session?.close();
  }, []);

  const startTalking = React.useCallback(async () => {
    if (whipRef.current) return;
    setTalkPhase("connecting");
    setTalkError(null);

    try {
      whipRef.current = await startWhipSession();
      setTalkPhase("live");
    } catch (e) {
      await stopTalking();
      setTalkPhase("failed");
      setTalkError(e instanceof Error ? e.message : String(e));
    }
  }, [stopTalking]);

  // Teardown on unmount and on tab close. Without the pagehide half, closing
  // the tab leaves the session (and the camera) held on the robot until the
  // next POST preempts it -- there is no callback from the worker.
  React.useEffect(() => {
    const onPageHide = () => {
      void sessionRef.current?.close();
      sessionRef.current = null;
      // The WHIP session holds the robot's speaker, so it matters at least as
      // much as the viewer slot that this fires on unload.
      void whipRef.current?.close();
      whipRef.current = null;
    };
    window.addEventListener("pagehide", onPageHide);
    return () => {
      window.removeEventListener("pagehide", onPageHide);
      onPageHide();
    };
  }, []);

  // Stats pump. Outside TanStack Query for the same reason the telemetry
  // socket is: there is nothing to refetch, and the RTCPeerConnection is the
  // source of truth rather than a URL.
  React.useEffect(() => {
    if (phase !== "live") return;
    let previous: Parameters<typeof readWhepStats>[1] = {};
    let cancelled = false;

    const tick = async () => {
      const pc = sessionRef.current?.pc;
      if (!pc) return;
      try {
        const result = await readWhepStats(pc, previous);
        if (cancelled) return;
        previous = result.samples;
        setStats(result.stats);
      } catch {
        /* the connection closed between the check and the call */
      }
    };

    void tick();
    const timer = setInterval(() => void tick(), STATS_POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [phase]);

  // The outbound half. In duplex mode the microphone rides the same peer
  // connection as the camera, so the stats come off that one — getStats()
  // reports both directions and readWhipStats only looks at the outbound and
  // media-source entries.
  const micLive = mode === "duplex" ? phase === "live" : talkPhase === "live";

  React.useEffect(() => {
    if (!micLive) return;
    let previous: Parameters<typeof readWhipStats>[1];
    let cancelled = false;

    const tick = async () => {
      const pc = mode === "duplex" ? sessionRef.current?.pc : whipRef.current?.pc;
      if (!pc) return;
      try {
        const result = await readWhipStats(pc, previous);
        if (cancelled) return;
        previous = result.sample;
        setWhipStats(result.stats);
      } catch {
        /* the connection closed between the check and the call */
      }
    };

    void tick();
    const timer = setInterval(() => void tick(), STATS_POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [micLive, mode]);

  // The backend's own view, polled independently: it is what says whether the
  // .so has been loaded at all, and it keeps answering after a session drops,
  // which is exactly when the browser side has nothing left to report.
  React.useEffect(() => {
    let cancelled = false;
    const poll = async () => {
      try {
        const res = await fetch(apiUrl("/api/v1/webrtc/status"));
        if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
        const body = (await res.json()) as BackendStatus;
        if (cancelled) return;
        setBackend(body);
        setBackendError(null);
      } catch (e) {
        if (cancelled) return;
        setBackend(null);
        setBackendError(e instanceof Error ? e.message : String(e));
      }
    };
    void poll();
    const timer = setInterval(() => void poll(), BACKEND_STATUS_POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, []);

  const live = phase === "live";
  const duplex = mode === "duplex";
  // In duplex mode the microphone is part of the one session, so "talking" is
  // whatever the camera session is doing.
  const talking = duplex ? live : talkPhase === "live";
  const audioFlowing = (stats?.audioPacketsReceived ?? 0) > 0;
  const videoFlowing = (stats?.framesDecoded ?? 0) > 0;

  return (
    <div className="flex h-full flex-col overflow-y-auto lg:flex-row lg:overflow-hidden">
      <section
        aria-label="Camera viewport"
        className="relative h-[55vh] shrink-0 bg-black lg:h-full lg:flex-1"
      >
        <video
          ref={videoRef}
          // playsInline so iOS Safari does not hijack it into a fullscreen
          // player; controls as the fallback when the autoplay policy refuses
          // the programmatic play().
          playsInline
          controls
          muted={muted}
          className="h-full w-full object-contain"
        />

        {!live && (
          <div className="absolute inset-0 flex items-center justify-center p-8">
            <div className="max-w-md text-center">
              <h1 className="instrument-label text-muted-foreground">
                {phase === "connecting" ? "Negotiating" : "Camera idle"}
              </h1>
              <p className="mt-2 text-sm text-white/80">
                {phase === "connecting"
                  ? "Gathering ICE candidates, then exchanging SDP."
                  : duplex
                    ? "Nothing is streaming. Connect to open one bidirectional session — camera down, microphone up."
                    : "Nothing is streaming. Connect to open a WHEP session."}
              </p>
              {error && (
                <p className="readout mt-3 text-left text-xs break-words text-signal-warn">
                  {error}
                </p>
              )}
              <p className="readout mt-3 text-xs text-white/40">
                POST {duplex ? "/api/v1/webrtc/duplex" : "/api/v1/webrtc/whep"}
              </p>
            </div>
          </div>
        )}

        <div className="absolute top-3 left-3 flex gap-2">
          <Button
            size="sm"
            variant={live ? "destructive" : "default"}
            disabled={phase === "connecting"}
            onClick={() => void (live ? stop() : start())}
          >
            {live ? <SquareIcon data-icon="inline-start" /> : <PlayIcon data-icon="inline-start" />}
            {live ? "Disconnect" : phase === "connecting" ? "Connecting…" : "Connect"}
          </Button>
          <Button
            size="sm"
            variant="outline"
            aria-pressed={muted}
            onClick={() => setMuted((m) => !m)}
          >
            {muted ? <VolumeXIcon data-icon="inline-start" /> : <Volume2Icon data-icon="inline-start" />}
            {muted ? "Unmute" : "Mute"}
          </Button>
          {/* Duplex has no separate Talk control: the microphone is part of
              the one session, so a second button would imply an independence
              the negotiation does not have. */}
          {!duplex && (
            <Button
              size="sm"
              variant={talking ? "destructive" : "outline"}
              aria-pressed={talking}
              // Independent of the camera on purpose: talking without watching
              // is a real case, and gating it on `live` would hide the control
              // exactly when someone is debugging the speaker.
              disabled={micAvailable === false || talkPhase === "connecting"}
              title={micAvailable === false ? INSECURE_CONTEXT_MESSAGE : undefined}
              onClick={() => void (talking ? stopTalking() : startTalking())}
            >
              {talking ? <MicIcon data-icon="inline-start" /> : <MicOffIcon data-icon="inline-start" />}
              {talking ? "Stop talking" : talkPhase === "connecting" ? "Connecting…" : "Talk"}
            </Button>
          )}
        </div>
      </section>

      <aside
        aria-label="Stream diagnostics"
        className="w-full shrink-0 border-t border-hairline bg-panel lg:h-full lg:w-80 lg:overflow-y-auto lg:border-t-0 lg:border-l"
      >
        <InstrumentGroup
          label="Negotiation"
          caption={
            duplex
              ? "One peer connection: m=video recvonly, m=audio sendrecv. One handshake, and both directions live or die together. Holds the camera and the speaker, so it preempts any WHEP or WHIP session."
              : "Two independent sessions. Watching needs no microphone permission, and each direction can fail on its own."
          }
        >
          <Segmented
            value={mode}
            options={MODE_OPTIONS}
            stretch
            // Switching mid-session would have the backend preempt the very
            // session being watched, from a control that looks like a view
            // filter. Disconnect first.
            disabled={phase !== "idle" || talkPhase !== "idle"}
            onChange={setMode}
          />
        </InstrumentGroup>

        <InstrumentGroup
          label="Session"
          caption="Location must be readable for teardown to work; if it is not, the backend is missing expose_headers."
        >
          <Readout
            label="Phase"
            value={phase}
            tone={live ? "live" : phase === "failed" ? "warn" : "neutral"}
          />
          <Readout label="Connection" value={connectionState} />
          <Readout label="ICE" value={iceState} />
          <Readout
            label="Location"
            value={
              locationReadable === null
                ? "—"
                : locationReadable
                  ? "readable"
                  : "hidden by CORS"
            }
            tone={locationReadable === false ? "warn" : "neutral"}
          />
          <Readout
            label="Session URL"
            value={sessionUrl ? sessionUrl.replace(/^https?:\/\/[^/]+/, "") : "—"}
          />
        </InstrumentGroup>

        <InstrumentGroup
          label="Video"
          action={
            <Chip tone={videoFlowing ? "live" : "neutral"}>
              {videoFlowing ? "flowing" : "no frames"}
            </Chip>
          }
        >
          <Readout
            label="Resolution"
            value={stats?.width && stats?.height ? `${stats.width}×${stats.height}` : "—"}
            tone="live"
          />
          <Readout label="Rate" value={fmt(stats?.fps, 0)} unit="fps" tone="live" />
          <Readout label="Bitrate" value={fmt(stats?.videoKbps, 0)} unit="kbps" />
          <Readout label="Frames decoded" value={fmt(stats?.framesDecoded, 0)} />
          <Readout
            label="Packets lost"
            value={fmt(stats?.videoPacketsLost, 0)}
            tone={(stats?.videoPacketsLost ?? 0) > 0 ? "caution" : "neutral"}
          />
        </InstrumentGroup>

        <InstrumentGroup
          label="Audio"
          caption="The Opus track shares one GStreamer pipeline with the camera, so an ALSA failure shows up as no video at all rather than as silence."
          action={
            <Chip tone={audioFlowing ? "live" : "neutral"}>
              {audioFlowing ? "flowing" : "no packets"}
            </Chip>
          }
        >
          <LevelMeter level={stats?.audioLevel ?? null} />
          <Readout label="Bitrate" value={fmt(stats?.audioKbps, 0)} unit="kbps" />
          <Readout label="Packets" value={fmt(stats?.audioPacketsReceived, 0)} />
          <Readout
            label="Packets lost"
            value={fmt(stats?.audioPacketsLost, 0)}
            tone={(stats?.audioPacketsLost ?? 0) > 0 ? "caution" : "neutral"}
          />
        </InstrumentGroup>

        <InstrumentGroup label="Path">
          <Readout label="Candidates" value={stats?.candidatePair ?? "—"} />
          <Readout label="Round trip" value={fmt(stats?.roundTripMs, 1)} unit="ms" />
        </InstrumentGroup>

        <InstrumentGroup
          label="Backend"
          caption="GET /api/v1/webrtc/status — what the robot believes. It has no callback from the worker, so an abandoned session reads as live until the next connect preempts it."
        >
          <Readout
            label="Worker library"
            value={
              backendError ? "unreachable" : backend?.library_loaded ? "loaded" : "not loaded"
            }
            tone={backendError ? "warn" : backend?.library_loaded ? "live" : "neutral"}
          />
          <Readout label="Viewer (WHEP)" value={backend?.video?.session_id ?? "none"} />
          <Readout label="Age" value={fmt(backend?.video?.age_seconds, 0)} unit="s" />
          <Readout label="Talker (WHIP)" value={backend?.audio?.session_id ?? "none"} />
          <Readout label="Age" value={fmt(backend?.audio?.age_seconds, 0)} unit="s" />
          <Readout label="Duplex" value={backend?.duplex?.session_id ?? "none"} />
          <Readout label="Age" value={fmt(backend?.duplex?.age_seconds, 0)} unit="s" />
          {backendError && (
            <p className="readout text-[11px] break-words text-signal-warn">{backendError}</p>
          )}
        </InstrumentGroup>

        <InstrumentGroup
          label={duplex ? "Microphone (duplex)" : "Microphone (WHIP)"}
          caption={
            duplex
              ? "Riding the camera session's peer connection. The stats below come off that same connection's outbound half."
              : "Out of the robot's USB speaker. One talker at a time — the speaker is one ALSA device, so a new session preempts the old one, and a TTS /speak during a session fails with the device busy."
          }
          action={
            <Chip tone={talking ? "live" : "neutral"}>
              {talking ? "talking" : talkPhase === "failed" ? "failed" : "idle"}
            </Chip>
          }
        >
          {micAvailable === false ? (
            <p className="text-[11px] leading-snug text-signal-warn">
              {INSECURE_CONTEXT_MESSAGE}
            </p>
          ) : (
            <>
              <LevelMeter level={whipStats?.audioLevel ?? null} />
              <Readout label="Bitrate" value={fmt(whipStats?.audioKbps, 0)} unit="kbps" />
              <Readout label="Packets sent" value={fmt(whipStats?.packetsSent, 0)} />
              <Readout label="Round trip" value={fmt(whipStats?.roundTripMs, 1)} unit="ms" />
            </>
          )}
          {talkError && (
            <p className="readout text-[11px] break-words text-signal-warn">{talkError}</p>
          )}
        </InstrumentGroup>
      </aside>
    </div>
  );
}

function fmt(value: number | null | undefined, digits: number): string {
  if (value == null || Number.isNaN(value)) return "—";
  return value.toFixed(digits);
}

/**
 * Inbound audio level as a bar.
 *
 * A meter rather than a number because the question it answers is "is the
 * robot's microphone picking anything up", which is a shape over time, not a
 * reading. Browsers that do not report audioLevel on inbound-rtp fall back to
 * the packet counter above — packets arriving with no level is still proof the
 * track is alive.
 */
function LevelMeter({ level }: { level: number | null }) {
  const pct = level == null ? 0 : Math.min(100, Math.round(level * 100));
  return (
    <div>
      <div className="flex items-baseline justify-between gap-3">
        <span className="instrument-label shrink-0 text-muted-foreground">Level</span>
        <span className="readout text-[13px] font-medium text-signal-live">
          {level == null ? "unreported" : `${pct}%`}
        </span>
      </div>
      <div
        className="mt-1.5 h-1.5 w-full overflow-hidden rounded-[1px] bg-hairline"
        role="meter"
        aria-valuenow={pct}
        aria-valuemin={0}
        aria-valuemax={100}
        aria-label="Inbound audio level"
      >
        <div
          className={cn("h-full bg-signal-live transition-[width] duration-150")}
          style={{ width: `${pct}%` }}
        />
      </div>
    </div>
  );
}
