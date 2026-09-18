import { apiUrl } from "@/lib/api/config";
import { errorDetail } from "@/lib/api/http";
import { resolveSessionUrl, waitForIceGathering } from "@/lib/video/signalling";
import { INSECURE_CONTEXT_MESSAGE, microphoneAvailable } from "@/lib/video/whip";

/**
 * Both directions over one peer connection: the robot's camera in, the
 * operator's microphone out.
 *
 * Not WHEP and not WHIP — both of those specify a single direction, so this
 * talks to its own endpoint (`/api/v1/webrtc/duplex`) rather than overloading
 * one of theirs. What it buys over holding a WHEP and a WHIP session at once
 * is one ICE/DTLS handshake instead of two, one gathering wait instead of two,
 * and a failure mode where both directions live or die together.
 *
 * The offer is built in a specific shape, and the order matters because the
 * worker matches its two outgoing tracks to the m-lines by kind, in order:
 *
 *   m-line 0   video   recvonly    the camera
 *   m-line 1   audio   sendrecv    microphone up, robot audio down
 *
 * The audio transceiver carries both directions, which is what a browser
 * produces naturally when a track is added with direction "sendrecv". Two
 * separate audio m-lines would also be expressible, but no WebRTC client emits
 * that by default and the worker does not answer it.
 *
 * Like WHIP, this needs a secure context for getUserMedia — see
 * lib/video/whip.ts for what that means on a plain-http LAN console.
 */

export interface DuplexSession {
  /** Inbound camera and robot audio. Attach to a <video> element. */
  readonly remote: MediaStream;
  /** The captured microphone, kept so the caller can mute or meter it. */
  readonly local: MediaStream;
  readonly sessionUrl: string | null;
  readonly pc: RTCPeerConnection;
  close: () => Promise<void>;
}

export interface DuplexHandlers {
  onConnectionState?: (state: RTCPeerConnectionState) => void;
  onIceState?: (state: RTCIceConnectionState) => void;
}

export async function startDuplexSession(
  handlers: DuplexHandlers = {},
  path = "/api/v1/webrtc/duplex",
): Promise<DuplexSession> {
  if (!microphoneAvailable()) throw new Error(INSECURE_CONTEXT_MESSAGE);

  // Echo cancellation matters more here than in the one-way case: the robot's
  // audio is arriving on the same peer connection and coming out of the local
  // speakers, so without it the operator feeds their own output back.
  const local = await navigator.mediaDevices.getUserMedia({
    audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    video: false,
  });

  const pc = new RTCPeerConnection({ iceServers: [] });

  const remote = new MediaStream();
  pc.addEventListener("track", (event) => remote.addTrack(event.track));

  let sessionUrl: string | null = null;
  let closed = false;

  const close = async () => {
    if (closed) return;
    closed = true;
    // The DELETE first: this session holds the camera *and* the speaker, so
    // dropping it locally without telling the robot leaves both claimed until
    // something else preempts them.
    if (sessionUrl) {
      try {
        await fetch(sessionUrl, { method: "DELETE", keepalive: true });
      } catch {
        /* the robot went away, or the tab is unloading — nothing to do */
      }
    }
    local.getTracks().forEach((track) => track.stop());
    pc.close();
  };

  try {
    // Order is the contract. Video first, receive-only.
    pc.addTransceiver("video", { direction: "recvonly" });
    // Then one audio transceiver carrying both directions.
    const [micTrack] = local.getAudioTracks();
    if (!micTrack) throw new Error("getUserMedia returned no audio track");
    pc.addTransceiver(micTrack, { direction: "sendrecv", streams: [local] });

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
    remote,
    local,
    get sessionUrl() {
      return sessionUrl;
    },
    pc,
    close,
  };
}
