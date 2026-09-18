import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import { resolveSessionUrl, waitForIceGathering } from "@/lib/video/signalling";

/**
 * WHIP client: the operator's microphone out of the robot's speaker.
 *
 * The mirror image of the WHEP client, with one difference that decides
 * whether it can run at all. WHEP only receives, so a plain-http LAN console
 * can open it; WHIP needs getUserMedia, and `navigator.mediaDevices` does not
 * exist outside a secure context. On http://<robot>:3001 the microphone is
 * therefore unavailable, and the failure is worth naming precisely because the
 * symptom — a property that is simply `undefined` — reads as a browser bug.
 *
 * Ways out, in the order they are worth trying:
 *   1. Chrome, started with
 *      --unsafely-treat-insecure-origin-as-secure=http://<robot>:3001
 *      (and --user-data-dir=/tmp/whip-test so it does not fight your profile)
 *   2. Firefox, media.devices.insecure.enabled + media.getusermedia.insecure.enabled
 *   3. Serve the console over https, which is the real fix and not one this
 *      page can make.
 */

/** Whether getUserMedia is reachable at all. See the module comment. */
export function microphoneAvailable(): boolean {
  return typeof navigator !== "undefined" && navigator.mediaDevices?.getUserMedia != null;
}

export const INSECURE_CONTEXT_MESSAGE =
  "the microphone needs a secure context; this page is plain http, so the browser " +
  "hides navigator.mediaDevices (see lib/video/whip.ts for the flags that work around it)";

export interface WhipSession {
  /** The captured microphone, kept so the caller can mute or meter it. */
  readonly stream: MediaStream;
  /** Absolute URL from the response's Location header, for the DELETE. */
  readonly sessionUrl: string | null;
  readonly pc: RTCPeerConnection;
  /** Best-effort DELETE, stop the microphone, close the peer connection. */
  close: () => Promise<void>;
}

export interface WhipHandlers {
  onConnectionState?: (state: RTCPeerConnectionState) => void;
  onIceState?: (state: RTCIceConnectionState) => void;
}

export async function startWhipSession(
  handlers: WhipHandlers = {},
  path = "/api/v1/webrtc/whip",
): Promise<WhipSession> {
  if (!microphoneAvailable()) throw new Error(INSECURE_CONTEXT_MESSAGE);

  // Echo cancellation and noise suppression on: the operator is likely
  // listening to the robot's own microphone through WHEP at the same time, and
  // without them the loop howls.
  const stream = await navigator.mediaDevices.getUserMedia({
    audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    video: false,
  });

  const pc = new RTCPeerConnection({ iceServers: [] });

  let sessionUrl: string | null = null;
  let closed = false;

  const close = async () => {
    if (closed) return;
    closed = true;
    // The DELETE first, while the page is still alive to make it: the robot's
    // speaker stays claimed by this session until something releases it, and
    // closing the peer connection locally tells it nothing.
    if (sessionUrl) {
      try {
        await fetch(sessionUrl, { method: "DELETE", keepalive: true });
      } catch {
        /* the robot went away, or the tab is unloading — nothing to do */
      }
    }
    // Stopping the tracks is what turns the browser's recording indicator off.
    // Skipping it leaves a tab that looks like it is still listening.
    stream.getTracks().forEach((track) => track.stop());
    pc.close();
  };

  try {
    // sendonly, one audio track: the worker receives and offers nothing back.
    stream.getAudioTracks().forEach((track) => {
      pc.addTransceiver(track, { direction: "sendonly", streams: [stream] });
    });

    if (handlers.onConnectionState) {
      pc.addEventListener("connectionstatechange", () =>
        handlers.onConnectionState?.(pc.connectionState),
      );
    }
    if (handlers.onIceState) {
      pc.addEventListener("iceconnectionstatechange", () =>
        handlers.onIceState?.(pc.iceConnectionState),
      );
    }

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await waitForIceGathering(pc);

    const url = apiUrl(path);
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      body: pc.localDescription?.sdp ?? offer.sdp ?? "",
    });

    if (!response.ok) throw new Error(await errorDetail(response));

    sessionUrl = resolveSessionUrl(response, url);
    const answer = await response.text();
    await pc.setRemoteDescription({ type: "answer", sdp: answer });
  } catch (error) {
    await close();
    throw error;
  }

  return {
    stream,
    get sessionUrl() {
      return sessionUrl;
    },
    pc,
    close,
  };
}

export interface WhipStats {
  /** Outbound audio. audioLevel is 0..1, from the local media-source stats. */
  audioLevel: number | null;
  audioKbps: number | null;
  packetsSent: number | null;
  roundTripMs: number | null;
}

interface ByteSample {
  bytes: number;
  at: number;
}

/**
 * Turn two getStats() snapshots into readable numbers for the outbound side.
 *
 * The level comes from `media-source`, not from `outbound-rtp`: the latter
 * reports what was sent, which is silence-suppressed, so a quiet room reads as
 * a dead microphone. media-source is the capture itself.
 */
export async function readWhipStats(
  pc: RTCPeerConnection,
  previous: ByteSample | undefined,
): Promise<{ stats: WhipStats; sample: ByteSample | undefined }> {
  const report = await pc.getStats();
  const now = performance.now();

  const stats: WhipStats = {
    audioLevel: null,
    audioKbps: null,
    packetsSent: null,
    roundTripMs: null,
  };
  let sample: ByteSample | undefined;

  report.forEach((entry) => {
    if (entry.type === "outbound-rtp" && entry.kind === "audio") {
      stats.packetsSent = entry.packetsSent ?? null;
      const bytes = entry.bytesSent ?? 0;
      if (previous && now > previous.at) {
        stats.audioKbps = ((bytes - previous.bytes) * 8) / (now - previous.at);
      }
      sample = { bytes, at: now };
    }
    if (entry.type === "media-source" && entry.kind === "audio") {
      stats.audioLevel = entry.audioLevel ?? null;
    }
    if (entry.type === "candidate-pair" && entry.state === "succeeded" && entry.nominated) {
      stats.roundTripMs =
        entry.currentRoundTripTime != null ? entry.currentRoundTripTime * 1000 : null;
    }
  });

  return { stats, sample };
}
