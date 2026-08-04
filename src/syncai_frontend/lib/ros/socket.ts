import { wsUrl } from "@/lib/api/config";
import type { StreamStatus } from "@/lib/types/stream";

/**
 * Fixed, not exponential. The failure this recovers from is a backend restart
 * on the same LAN, where the right behaviour is to be back within a couple of
 * seconds; there is one operator console per robot, so there is no
 * thundering-herd to back off from either.
 */
const RECONNECT_DELAY_MS = 2000;

export interface ReconnectingSocketOptions {
  /**
   * Raw frame payload. Typed as the union WebSocket can actually deliver
   * rather than narrowed here: each stream knows which half it wants and
   * discards the other (the cloud checks `instanceof ArrayBuffer`, telemetry
   * checks `typeof === "string"`), which is also what keeps a malformed or
   * unexpected frame from killing the connection.
   */
  onMessage: (data: string | ArrayBuffer | Blob) => void;
  onStatus?: (status: StreamStatus) => void;
  /** Set to "arraybuffer" for binary streams; omit for JSON text frames. */
  binaryType?: BinaryType;
}

export interface ReconnectingSocket {
  close: () => void;
}

/**
 * Open a backend WebSocket that reconnects until closed.
 *
 * This is the connect/retry/status plumbing that createPointCloudStream and
 * createTelemetryStream had each grown their own line-for-line identical copy
 * of. Only the payload decoding differs between them, so only that stays in
 * the per-stream modules.
 *
 * WHY THERE ARE TWO OF THESE, i.e. why the backend serves the cloud and the
 * telemetry feed on separate endpoints: **backpressure**, not the payload
 * format. One WebSocket is a single ordered TCP stream, so a few-hundred-KB
 * cloud frame in flight would head-of-line block the 20 Hz pose queued behind
 * it. Both backend loops poll a single-slot cache and skip anything the client
 * has already seen (`after_seq`), which means a slow client is *supposed* to
 * lose frames rather than accumulate them — the send blocking is the signal
 * that makes it skip. Sharing one socket would quietly convert "drop the stale
 * pose" into "deliver a late pose once the cloud is through", which is the one
 * thing a 3D viewer must not do. Mixing text and binary frames on one
 * connection would otherwise be trivial (`typeof ev.data`), so the format is
 * not the reason.
 */
export function createReconnectingSocket(
  path: string,
  opts: ReconnectingSocketOptions,
): ReconnectingSocket {
  let ws: WebSocket | null = null;
  let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  let closed = false;

  const connect = () => {
    if (closed) return;
    opts.onStatus?.("connecting");
    ws = new WebSocket(wsUrl(path));
    if (opts.binaryType) ws.binaryType = opts.binaryType;

    ws.onopen = () => opts.onStatus?.("open");
    ws.onmessage = (ev) => opts.onMessage(ev.data);
    ws.onerror = () => opts.onStatus?.("error");
    ws.onclose = () => {
      // Reported on an explicit close() as well as on an unexpected drop, on
      // purpose. A caller can stop a stream without unmounting —
      // PointCloudCanvas's `liveStream` prop does exactly that, and its effect
      // returns early rather than reopening — so suppressing the final
      // transition would leave the status badge reading "open" with no socket
      // behind it.
      opts.onStatus?.("closed");
      if (!closed) {
        reconnectTimer = setTimeout(connect, RECONNECT_DELAY_MS);
      }
    };
  };

  connect();

  return {
    close: () => {
      closed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
    },
  };
}
