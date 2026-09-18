import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import { resolveSessionUrl, waitForIceGathering } from "@/lib/video/signalling";

/**
 * WHEP client for the robot camera — deliberately vanilla ICE, not trickle.
 * The negotiation shape and its reasons live in lib/video/signalling.ts.
 *
 * Receive-only, which is also why this works from a plain-http origin: a LAN
 * console on http://<robot>:3001 is not a secure context, and getUserMedia
 * would be blocked there. RTCPeerConnection is not — which is exactly where
 * the WHIP client parts company with this one.
 */

export interface WhepSession {
  /** The inbound tracks. Attach to a <video> element's srcObject. */
  readonly stream: MediaStream;
  /** Absolute URL from the response's Location header, for the DELETE. */
  readonly sessionUrl: string | null;
  readonly pc: RTCPeerConnection;
  /** Best-effort DELETE, then close the peer connection. Idempotent. */
  close: () => Promise<void>;
}

export interface WhepHandlers {
  /** RTCPeerConnection.connectionState on every change. */
  onConnectionState?: (state: RTCPeerConnectionState) => void;
  onIceState?: (state: RTCIceConnectionState) => void;
}

export async function startWhepSession(
  handlers: WhepHandlers = {},
  path = "/api/v1/webrtc/whep",
): Promise<WhepSession> {
  const pc = new RTCPeerConnection({
    // No iceServers: the robot and the console share a LAN, so host
    // candidates are what connect them. The worker's own STUN/TURN config is
    // a separate, server-side matter.
    iceServers: [],
  });

  const stream = new MediaStream();
  pc.addEventListener("track", (event) => stream.addTrack(event.track));

  // Two recvonly transceivers, in this order, because the worker offers
  // exactly two sendonly tracks: H264 video then Opus audio. The m-lines have
  // to be there for it to attach them to anything.
  pc.addTransceiver("video", { direction: "recvonly" });
  pc.addTransceiver("audio", { direction: "recvonly" });

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

  let sessionUrl: string | null = null;
  let closed = false;

  const close = async () => {
    if (closed) return;
    closed = true;
    // The DELETE first, while the page is still alive to make it: closing the
    // peer connection locally tells the robot nothing, and the camera stays
    // open on its side until something reclaims it.
    if (sessionUrl) {
      try {
        await fetch(sessionUrl, { method: "DELETE", keepalive: true });
      } catch {
        /* the robot went away, or the tab is unloading — nothing to do */
      }
    }
    pc.close();
  };

  try {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await waitForIceGathering(pc);

    const url = apiUrl(path);
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      // localDescription, not `offer`: it is the one that carries the
      // candidates gathered above.
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

export interface WhepStats {
  /** Inbound video, or null before the first frame arrives. */
  width: number | null;
  height: number | null;
  fps: number | null;
  videoKbps: number | null;
  framesDecoded: number | null;
  videoPacketsLost: number | null;
  /** Inbound audio. audioLevel is 0..1 and is not reported by every browser. */
  audioLevel: number | null;
  audioKbps: number | null;
  audioPacketsReceived: number | null;
  audioPacketsLost: number | null;
  /** The selected candidate pair, e.g. "host -> host". */
  candidatePair: string | null;
  roundTripMs: number | null;
}

interface ByteSample {
  bytes: number;
  at: number;
}

/**
 * Turn two getStats() snapshots into readable numbers.
 *
 * Bitrate has to be differenced across calls because RTCStats reports
 * cumulative bytes, so the caller keeps the previous sample and hands it back.
 * That is also why this is a plain function rather than a hook: the test page
 * polls it on an interval and re-renders from the result, but a dashboard
 * panel would want the same arithmetic without the React.
 */
export async function readWhepStats(
  pc: RTCPeerConnection,
  previous: { video?: ByteSample; audio?: ByteSample },
): Promise<{ stats: WhepStats; samples: { video?: ByteSample; audio?: ByteSample } }> {
  const report = await pc.getStats();
  const now = performance.now();

  const stats: WhepStats = {
    width: null,
    height: null,
    fps: null,
    videoKbps: null,
    framesDecoded: null,
    videoPacketsLost: null,
    audioLevel: null,
    audioKbps: null,
    audioPacketsReceived: null,
    audioPacketsLost: null,
    candidatePair: null,
    roundTripMs: null,
  };
  const samples: { video?: ByteSample; audio?: ByteSample } = {};

  const kbps = (bytes: number, prior: ByteSample | undefined) => {
    if (!prior || now <= prior.at) return null;
    return ((bytes - prior.bytes) * 8) / (now - prior.at);
  };

  report.forEach((entry) => {
    if (entry.type === "inbound-rtp" && entry.kind === "video") {
      stats.width = entry.frameWidth ?? null;
      stats.height = entry.frameHeight ?? null;
      stats.fps = entry.framesPerSecond ?? null;
      stats.framesDecoded = entry.framesDecoded ?? null;
      stats.videoPacketsLost = entry.packetsLost ?? null;
      const bytes = entry.bytesReceived ?? 0;
      stats.videoKbps = kbps(bytes, previous.video);
      samples.video = { bytes, at: now };
    }
    if (entry.type === "inbound-rtp" && entry.kind === "audio") {
      stats.audioLevel = entry.audioLevel ?? null;
      stats.audioPacketsReceived = entry.packetsReceived ?? null;
      stats.audioPacketsLost = entry.packetsLost ?? null;
      const bytes = entry.bytesReceived ?? 0;
      stats.audioKbps = kbps(bytes, previous.audio);
      samples.audio = { bytes, at: now };
    }
    if (entry.type === "candidate-pair" && entry.state === "succeeded" && entry.nominated) {
      stats.roundTripMs =
        entry.currentRoundTripTime != null ? entry.currentRoundTripTime * 1000 : null;
    }
  });

  // Candidate types come from the local/remote candidate entries the winning
  // pair points at, which is a second lookup — worth it because "host -> host"
  // versus anything else is the whole answer to "why is this stream slow".
  report.forEach((entry) => {
    if (entry.type === "candidate-pair" && entry.state === "succeeded" && entry.nominated) {
      const local = report.get(entry.localCandidateId);
      const remote = report.get(entry.remoteCandidateId);
      if (local && remote) {
        stats.candidatePair = `${local.candidateType} → ${remote.candidateType}`;
      }
    }
  });

  return { stats, samples };
}
